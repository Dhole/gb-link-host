#[macro_use]
extern crate enum_primitive;

extern crate num;
extern crate serial;
extern crate clap;
extern crate bufstream;
extern crate image;
extern crate time;
extern crate bit_vec;
extern crate itertools;

use std::io;
use std::io::{Error, ErrorKind};
// use std::io::{BufReader, BufWriter};
use std::cmp;
use std::thread;
use std::time::Duration;
use std::process;
// use std::fs::File;
use std::path::Path;
use std::process::Command;
use std::num::Wrapping;

use num::FromPrimitive;
use std::io::prelude::*;
use serial::prelude::*;
use bufstream::BufStream;
use clap::{App, Arg};
use image::ImageBuffer;
use image::GenericImage;
use bit_vec::BitVec;
use itertools::Itertools;

#[derive(Debug, PartialEq)]
enum Mode {
    Sniff,
    Printer,
    Print,
}

#[derive(Debug, PartialEq)]
enum Board {
    Generic,
    St,
}

fn mode_char(mode: &Mode) -> u8 {
    match mode {
        &Mode::Sniff => b's',
        &Mode::Printer => b'p',
        &Mode::Print => b'P',
    }
}

fn main() {
    let matches = App::new("gb-link")
        .version("0.1")
        .about("Interface for Gameboy link via serial")
        .author("Dhole")
        .arg(
            Arg::with_name("baud")
            .help("Set the baud rate")
            .short("b")
            .long("baud")
            .value_name("RATE")
            //.default_value("115200")
            .default_value("1000000")
            .takes_value(true)
            .required(false)
            .validator(|baud| match baud.parse::<usize>() {
                Ok(_) => Ok(()),
                Err(e) => Err(format!("{}", e)),
            }),
        )
        .arg(
            Arg::with_name("serial")
                .help("Set the serial device")
                .short("s")
                .long("serial")
                .value_name("DEVICE")
                .default_value("/dev/ttyACM0")
                .takes_value(true)
                .required(false),
        )
        .arg(
            Arg::with_name("mode")
                .help("Set the device mode: sniff, printer, print")
                .short("m")
                .long("mode")
                .value_name("MODE")
                .default_value("sniff")
                .takes_value(true)
                .required(false)
                .validator(|mode| match mode.as_str() {
                    "sniff" => Ok(()),
                    "printer" => Ok(()),
                    "print" => Ok(()),
                    mode => Err(format!("Invalid mode: {}", mode)),
                }),
        )
        .arg(
            Arg::with_name("board")
                .help("Set the development board: generic, st")
                .short("d")
                .long("board")
                .value_name("BOARD")
                .default_value("st")
                .takes_value(true)
                .required(false)
                .validator(|board| match board.as_str() {
                    "generic" => Ok(()),
                    "st" => Ok(()),
                    board => Err(format!("Invalid development board: {}", board)),
                }),
        )
        .arg(
            Arg::with_name("file")
                .help("Image file to print")
                .short("f")
                .long("file")
                .value_name("FILE")
                .takes_value(true)
                .required(false),
        )
        .get_matches();

    let serial = matches.value_of("serial").unwrap();
    let baud = matches.value_of("baud").unwrap().parse::<usize>().unwrap();
    let mode = match matches.value_of("mode").unwrap() {
        "sniff" => Mode::Sniff,
        "printer" => Mode::Printer,
        "print" => Mode::Print,
        mode => panic!("Invalid mode: {}", mode),
    };
    let filename = match matches.value_of("file") {
        Some(filename) => filename,
        None => {
            println!("Please, select a file to print with `-f FILE`");
            return ();
        }
    };
    let board = match matches.value_of("board").unwrap() {
        "generic" => Board::Generic,
        "st" => Board::St,
        board => panic!("Invalid board: {}", board),
    };
    println!("Development board is: {:?}", board);
    println!("Using serial device: {} at baud rate: {}", serial, baud);

    let mut port_raw = match serial::open(serial) {
        Ok(port) => port,
        Err(e) => {
            println!("Error opening {}: {}", serial, e);
            process::exit(1);
        }
    };
    port_raw
        .configure(&serial::PortSettings {
            baud_rate: serial::BaudRate::from_speed(baud),
            char_size: serial::Bits8,
            parity: serial::ParityNone,
            stop_bits: serial::Stop1,
            flow_control: serial::FlowNone,
        })
        .unwrap_or_else(|e| {
            println!("Error configuring {}: {}", serial, e);
            process::exit(1);
        });
    port_raw
        .set_timeout(Duration::from_secs(3600 * 24))
        .unwrap_or_else(|e| {
            println!("Error setting timeout for {}: {}", serial, e);
            process::exit(1);
        });

    dev_reset(board).unwrap_or_else(|e| {
        println!("Error resetting development board: {}", e);
        process::exit(1);
    });

    let mut port = BufStream::new(port_raw);
    gb_link(&mut port, mode, filename).unwrap_or_else(|e| {
        println!("Error from serial device {}: {}", serial, e);
        process::exit(1);
    });
}

fn dev_reset(board: Board) -> Result<(), io::Error> {
    match board {
        Board::Generic => {
            println!("Press the reset button on the board");
        }
        Board::St => {
            println!("\nResetting board using st-flash utility...");
            let output = Command::new("st-flash").arg("reset").output()?;
            println!("{}", String::from_utf8_lossy(&output.stderr));
            if !output.status.success() {
                return Err(Error::new(
                    ErrorKind::Other,
                    format!(
                        "st-flash returned with error code {:?}",
                        output.status.code()
                    ),
                ));
            }
        }
    }
    Ok(())
}

fn gb_link<T: SerialPort>(
    mut port: &mut BufStream<T>,
    mode: Mode,
    filename: &str,
) -> Result<(), io::Error> {
    let mut buf = Vec::new();
    loop {
        try!(port.read_until(b'\n', &mut buf));
        if buf == b"HELLO\n" {
            break;
        }
        buf.clear();
    }
    println!("Connected!");

    println!("Setting {:?} mode", mode);
    try!(port.write_all(&vec![mode_char(&mode)]));
    try!(port.flush());

    match mode {
        Mode::Sniff => mode_sniff(&mut port),
        Mode::Printer => mode_printer(&mut port),
        Mode::Print => mode_print(&mut port, filename),
    }
}

fn mode_sniff<T: SerialPort>(port: &mut BufStream<T>) -> Result<(), io::Error> {
    let mut buf = vec![0, 0];
    loop {
        try!(port.read_exact(&mut buf));
        let sdout = buf[0];
        let sdin = buf[1];
        let sdout_s = if sdout == 0 {
            format!("")
        } else {
            format!("{:02x}", sdout)
        };
        let sdin_s = if sdin == 0 {
            format!("")
        } else {
            format!("{:02x}", sdin)
        };
        print!("{}:{} ", sdout_s, sdin_s);
        io::stdout().flush()?;
    }
}

const PRINT_MAGIC: [u8; 2] = [0x88, 0x33];
const PRINT_ACK: u8 = 0x81;

enum_from_primitive! {
#[derive(Debug, PartialEq, Copy, Clone)]
enum PrintCommand {
    Init = 0x01,
    Print = 0x02,
    Data = 0x04,
    Status = 0x0f,
}
}

#[derive(Debug, PartialEq)]
struct PrinterStatus {
    checksum_error: bool,
    printer_busy: bool,
    image_data_full: bool,
    unprocessed_data: bool,
    packet_error: bool,
    paper_jam: bool,
    other_error: bool,
    battery_too_low: bool,
}

impl PrinterStatus {
    fn any_info(&self) -> bool {
        return self.checksum_error || self.printer_busy || self.image_data_full ||
            self.unprocessed_data || self.packet_error || self.paper_jam ||
            self.other_error || self.battery_too_low;
    }
    fn any_error(&self) -> bool {
        return self.checksum_error ||
               //self.printer_busy ||
               //self.image_data_full ||
               //self.unprocessed_data ||
               self.packet_error || self.paper_jam || self.other_error ||
            self.battery_too_low;
    }
}

impl From<u8> for PrinterStatus {
    fn from(b: u8) -> Self {
        PrinterStatus {
            checksum_error: (b & (0x01 << 0)) != 0,
            printer_busy: (b & (0x01 << 1)) != 0,
            image_data_full: (b & (0x01 << 2)) != 0,
            unprocessed_data: (b & (0x01 << 3)) != 0,
            packet_error: (b & (0x01 << 4)) != 0,
            paper_jam: (b & (0x01 << 5)) != 0,
            other_error: (b & (0x01 << 6)) != 0,
            battery_too_low: (b & (0x01 << 7)) != 0,
        }
    }
}

fn read_until_magic<T: SerialPort>(port: &mut BufStream<T>, magic: &[u8]) -> Result<(), io::Error> {
    let mut buf = vec![0];
    let mut idx = 0;
    loop {
        try!(port.read_exact(&mut buf));
        // println!("until_magic: {:02x}", buf[0]);
        if buf[0] == magic[idx] {
            idx += 1;
        }
        if idx == magic.len() {
            return Ok(());
        }
    }
}

fn mode_printer<T: SerialPort>(mut port: &mut BufStream<T>) -> Result<(), io::Error> {
    // The gameboy camera only checks the ACK on the first request (Init).
    // try!(port.write_all(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x00]));
    // try!(port.flush());
    let mut tile_rows = Vec::<Vec<u8>>::new();
    loop {
        // Wait for the magic bytes.
        try!(read_until_magic(&mut port, &PRINT_MAGIC));
        let mut buf = vec![0; 4];
        // Read cmd, arg1, arg2, arg3
        try!(port.read_exact(&mut buf));
        let cmd = buf[0];
        let args = &buf[1..4];
        // println!("cmd, args: {:02x} {:02x} {:02x} {:02x}", cmd, args[0], args[1], args[2]);
        let len = (args[1] as u16) + ((args[2] as u16) << 8);
        let mut payload = vec![0; len as usize];
        let mut checksum = vec![0; 2];
        try!(port.read_exact(&mut payload));
        try!(port.read_exact(&mut checksum));
        // println!("payload: {:?}", payload);
        // println!("checksum: {:02x} {:02x}", checksum[0], checksum[1]);
        match PrintCommand::from_u8(cmd) {
            Some(PrintCommand::Init) => {
                println!("Receiving data...");
                tile_rows.clear();
            }
            Some(PrintCommand::Print) => {
                let palette = &payload[2];
                let filename = format!(
                    "gb_printer_{}.png",
                    time::now().strftime("%FT%H%M%S").unwrap()
                );
                println!("Saving image at {}", filename);
                try!(printer_save_image(&tile_rows, palette, filename));
            }
            Some(PrintCommand::Data) => {
                if len != 0 {
                    tile_rows.push(payload);
                }
            }
            Some(PrintCommand::Status) => {}
            None => {}
        }
        let mut ack_status = vec![0; 2];
        try!(port.read_exact(&mut ack_status));
        // println!("ack, status = {:02x} {:02x}", ack_status[0], ack_status[1]);
        // try!(port.write_all(&[0x80, 0x00]));
        // try!(port.flush());
    }
}

fn printer_save_image(
    tile_rows: &Vec<Vec<u8>>,
    palette_byte: &u8,
    filename: String,
) -> Result<(), io::Error> {
    // let mut img: ImageBuffer<image::Luma<u8>, Vec<<image::Luma<u8> as image::Pixel>::Subpixel>> =
    //    ImageBuffer::new(160, 16 * tile_rows.len() as u32);
    let palette: Vec<u8> = BitVec::from_bytes(&[*palette_byte])
        .iter()
        .tuples()
        .map(|(h, l)| (l as u8) + 2 * (h as u8))
        .map(|v| v * (255 / 3))
        .collect();
    let mut img = ImageBuffer::new(160, 16 * tile_rows.len() as u32);
    img.put_pixel(0, 0, image::Luma([255u8]));

    let mut pixel_rows = Vec::new();
    for tile_row in tile_rows {
        let (tile_row_a, tile_row_b) = tile_row.split_at(tile_row.len() / 2 as usize);
        let mut pixel_rows_a = tile_row_to_pixel_rows(tile_row_a);
        let mut pixel_rows_b = tile_row_to_pixel_rows(tile_row_b);
        pixel_rows.append(&mut pixel_rows_a);
        pixel_rows.append(&mut pixel_rows_b);
    }

    for (y, pixel_row) in pixel_rows.iter().enumerate() {
        for (x, val) in pixel_row.iter().enumerate() {
            img.put_pixel(x as u32, y as u32, image::Luma([palette[*val as usize]]));
            // image::Luma([((*val as f32) / 3.0 * 255.0) as u8]));
        }
    }

    img.save(&Path::new(&filename))?;
    return Ok(());
}

fn tile_row_to_pixel_rows(tile_row: &[u8]) -> Vec<Vec<u8>> {
    // let mut pixel_rows: Vec<Vec<u8>> = Vec::new();
    let mut pixel_rows: Vec<Vec<u8>> = (0..8).map(|_| vec![0u8; 160]).collect();
    // let mut pixel_rows = Vec::new();
    for i in 0..(tile_row.len() / 16 as usize) {
        let tile = &tile_row[i * 16..i * 16 + 16];
        for j in 0..8 {
            let tile_pixel_row = BitVec::from_bytes(&[tile[j * 2]])
                .iter()
                .zip(BitVec::from_bytes(&[tile[j * 2 + 1]]).iter())
                .map(|(l, h)| (l as u8) + 2 * (h as u8))
                .collect::<Vec<u8>>();
            // pixel_rows.push(tile_pixel_row);
            for k in 0..8 {
                pixel_rows[j][i * 8 + k] = tile_pixel_row[k];
            }
        }
    }
    return pixel_rows;
}

fn u16_to_low_high(w: u16) -> [u8; 2] {
    return [(w & 0xff) as u8, ((w & 0xff00) >> 8) as u8];
}

fn gen_crc(cmd: PrintCommand, payload: &[u8]) -> [u8; 2] {
    let mut crc = Wrapping(0u16);
    crc += Wrapping(cmd as u16);
    let len = u16_to_low_high(payload.len() as u16);
    crc += Wrapping(len[0] as u16);
    crc += Wrapping(len[1] as u16);
    for b in payload {
        crc += Wrapping(*b as u16);
    }
    return u16_to_low_high(crc.0);
}

fn send_print_cmd<T: SerialPort>(
    port: &mut BufStream<T>,
    cmd: PrintCommand,
    payload: &[u8],
) -> Result<Option<PrinterStatus>, io::Error> {
    // Tell stm32f411 the data length
    port.write_all(&u16_to_low_high(10 + payload.len() as u16))?;
    // write magic
    port.write_all(&PRINT_MAGIC)?;
    // write cmd
    port.write_all(&[cmd as u8])?;
    // write arg
    port.write_all(&[0x00])?;
    // write len
    port.write_all(&u16_to_low_high(payload.len() as u16))?;
    // write payload
    port.write_all(payload)?;
    // write crc
    port.write_all(&gen_crc(cmd, payload))?;
    // write empty array to receive ACK and STATUS
    port.write_all(&[0; 2])?;
    port.flush()?;

    let mut ack_status = vec![0; 2];
    try!(port.read_exact(&mut ack_status));
    //println!("{:?} -> ACK: {:x}, STATUS: {:x}", cmd, ack_status[0], ack_status[1]);
    if ack_status[0] == PRINT_ACK {
        return Ok(Some(PrinterStatus::from(ack_status[1])));
    } else {
        return Ok(None);
    }
}

fn img_to_tile_rows(img: image::GrayImage) -> Vec<Vec<u8>> {
    let mut tile_rows: Vec<Vec<u8>> = (0..img.height() / 16).map(|_| vec![0u8; 640]).collect();
    for row in 0..img.height() / 8 {
        for col in 0..img.width() / 8 {
            for y in 0..8 {
                let mut lsb = 0 as u8;
                let mut msb = 0 as u8;
                for x in 0..8 {
                    let p = img.get_pixel(col * 8 + x, row * 8 + y).data[0];
                    let (low, high) = if p < 64 {
                        (1, 1)
                    } else if p < 128 {
                        (0, 1)
                    } else if p < 192 {
                        (1, 0)
                    } else {
                        (0, 0)
                    };
                    lsb = lsb | (low << (7 - x));
                    msb = msb | (high << (7 - x));
                }
                tile_rows[(row / 2) as usize][((row % 2) * 320 + col * 16 + y * 2) as usize] = lsb;
                tile_rows[(row / 2) as usize][((row % 2) * 320 + col * 16 + y * 2 + 1) as usize] =
                    msb;
            }
        }
    }
    return tile_rows;
}

macro_rules! check_ack {
    ($a:expr) => {
        match $a {
            None => {
                println!("Gameboy Printer didn't ACK, exiting...");
                return Ok(());
            },
            Some(status) => status,
        }
    }
}

macro_rules! check_status_error {
    ($a:expr) => {
        if $a.any_error() {
            println!("Error: Gameboy Printer: {:?}", $a);
            return Ok(());
        }
    }
}

// Print up to 9 tile_rows
fn print<T: SerialPort>(
    port: &mut BufStream<T>,
    tile_rows: &[Vec<u8>],
    margins: (u8, u8),
) -> Result<(), io::Error> {
    // Init printer
    println!("Initializing printer...");
    let status = check_ack!(send_print_cmd(port, PrintCommand::Init, &[])?);
    if status.any_info() {
        return Err(Error::new(ErrorKind::Other, format!("{:?}", status)));
    }
    // Send data
    println!("Sending data to printer...");
    for tile_row in tile_rows {
        //println!("{:?}", tile_row);
        check_status_error!(check_ack!(
            send_print_cmd(port, PrintCommand::Data, &tile_row)?
        ));
        check_status_error!(check_ack!(send_print_cmd(port, PrintCommand::Status, &[])?));
    }
    // Send 0 length data to notify the Printer that we've sent all data
    check_status_error!(check_ack!(send_print_cmd(port, PrintCommand::Data, &[])?));
    // Print
    println!("Printing...");
    check_status_error!(check_ack!(send_print_cmd(
        port,
        PrintCommand::Print,
        &[0x01, margins.0 << 4 | margins.1, 0xE4, 0x40],
    )?));
    // Query status
    let sleep_time = Duration::from_millis(500);
    loop {
        thread::sleep(sleep_time);
        let status = check_ack!(send_print_cmd(port, PrintCommand::Status, &[])?);
        check_status_error!(status);
        if !status.any_info() {
            println!("Printing successfull!");
            break;
        } else if !status.printer_busy {
            return Err(Error::new(ErrorKind::Other, format!("{:?}", status)));
        }
    }
    return Ok(());
}

fn mode_print<T: SerialPort>(port: &mut BufStream<T>, filename: &str) -> Result<(), io::Error> {
    let img = match image::open(&Path::new(filename)) {
        Ok(img) => img,
        Err(err) => {
            println!("Error opening image at file {:?}: {:?}", filename, err);
            return Ok(());
        }
    };
    let img = img.grayscale();
    let img = if img.height() == 160 {
        img.rotate90()
    } else {
        img
    };
    if img.width() != 160 {
        println!(
            "Error: image {:?} width should be 160 instead of {:?}",
            filename,
            img.width()
        );
        return Ok(());
    }

    if img.height() % 16 != 0 {
        panic!("height is not a multiple of 16!");
    }

    let img = img.to_luma();
    let tile_rows = img_to_tile_rows(img);

    //println!("Turn on the GameBoy Printer and then press any key to continue...");
    //let _ = io::stdin().read(&mut [0u8]).unwrap();

    // Send confirmation to notify that the printer has ben turned on
    try!(port.write_all(&[0x00]));
    try!(port.flush());

    let k = ((tile_rows.len() as f64) / 9.0).ceil() as usize;
    for i in 0..k {
        let margin = if i == 0 {
            (0x00, 0x00)
        } else if i == (k - 1) {
            (0x00, 0x03)
        } else {
            (0x00, 0x00)
        };
        let (a, b) = (i * 9, cmp::min(i * 9 + 9, tile_rows.len()));
        if k != 1 {
            println!("\tPrinting part [{:?}/{:?}]", i + 1, k);
        }
        print(port, &tile_rows[a..b], margin)?;
        if i != (k - 1) {
            thread::sleep(Duration::from_millis(400));;
        }
    }

    return Ok(());
}
