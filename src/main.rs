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
// use std::io::{BufReader, BufWriter};
use std::time::Duration;
use std::process;
// use std::fs::File;
use std::path::Path;

use num::FromPrimitive;
use std::io::prelude::*;
use serial::prelude::*;
use bufstream::BufStream;
use clap::{App, Arg};
use image::ImageBuffer;
use bit_vec::BitVec;
use itertools::Itertools;

#[derive(Debug, PartialEq)]
enum Mode {
    Sniff,
    Printer,
}

fn mode_char(mode: &Mode) -> u8 {
    match mode {
        &Mode::Sniff => b's',
        &Mode::Printer => b'b',
    }
}

fn main() {
    let matches = App::new("gb-link")
        .version("0.1")
        .about("Interface for Gameboy link via serial")
        .author("Dhole")
        .arg(Arg::with_name("baud")
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
            }))
        .arg(Arg::with_name("serial")
            .help("Set the serial device")
            .short("s")
            .long("serial")
            .value_name("DEVICE")
            .default_value("/dev/ttyACM0")
            .takes_value(true)
            .required(false))
        .arg(Arg::with_name("mode")
            .help("Set the device mode: sniff, printer")
            .short("m")
            .long("mode")
            .value_name("MODE")
            .default_value("sniff")
            .takes_value(true)
            .required(false)
            .validator(|mode| match mode.as_str() {
                "sniff" => Ok(()),
                "printer" => Ok(()),
                mode => Err(format!("Invalid mode: {}", mode)),
            }))
        .get_matches();

    let serial = matches.value_of("serial").unwrap();
    let baud = matches.value_of("baud").unwrap().parse::<usize>().unwrap();
    let mode = match matches.value_of("mode").unwrap() {
        "sniff" => Mode::Sniff,
        "printer" => Mode::Printer,
        mode => panic!("Invalid mode: {}", mode),
    };
    println!("Using serial device: {} at baud rate: {}", serial, baud);

    let mut port_raw = match serial::open(serial) {
        Ok(port) => port,
        Err(e) => {
            println!("Error opening {}: {}", serial, e);
            process::exit(1);
        }
    };
    port_raw.configure(&serial::PortSettings {
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
    port_raw.set_timeout(Duration::from_secs(3600 * 24)).unwrap_or_else(|e| {
        println!("Error setting timeout for {}: {}", serial, e);
        process::exit(1);
    });

    let mut port = BufStream::new(port_raw);
    gb_link(&mut port, mode).unwrap_or_else(|e| {
        println!("Error from serial device {}: {}", serial, e);
        process::exit(1);
    });
}

fn gb_link<T: SerialPort>(mut port: &mut BufStream<T>, mode: Mode) -> Result<(), io::Error> {
    println!("Press the reset button on the board");
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
    }
}

fn mode_sniff<T: SerialPort>(port: &mut BufStream<T>) -> Result<(), io::Error> {
    let mut buf = vec![0, 0];
    loop {
        try!(port.read_exact(&mut buf));
        let sdin = buf[0];
        let sdout = buf[1];
        print!("{:02x}:{:02x} ", sdin, sdout);
        io::stdout().flush()?;
    }
}

const PRINT_MAGIC: [u8; 2] = [0x88, 0x33];

enum_from_primitive! {
#[derive(Debug, PartialEq)]
enum PrintCommand {
    Init = 0x01,
    Print = 0x02,
    Data = 0x04,
    Status = 0x0f,
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
    try!(port.write_all(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x00]));
    try!(port.flush());
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
                let filename = format!("gb_printer_{}.png",
                                       time::now().strftime("%FT%H%M%S").unwrap());
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

fn printer_save_image(tile_rows: &Vec<Vec<u8>>,
                      palette_byte: &u8,
                      filename: String)
                      -> Result<(), io::Error> {
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
