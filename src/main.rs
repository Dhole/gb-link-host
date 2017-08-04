#[macro_use]
extern crate enum_primitive;

extern crate num;
extern crate serial;
extern crate clap;
extern crate bufstream;

use std::io;
// use std::io::{BufReader, BufWriter};
use std::time::Duration;
use std::process;

use num::FromPrimitive;

use std::io::prelude::*;
use serial::prelude::*;
use bufstream::BufStream;

use clap::{App, Arg};

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
        println!("until_magic: {:02x}", buf[0]);
        if buf[0] == magic[idx] {
            idx += 1;
        }
        if idx == magic.len() {
            return Ok(());
        }
    }
}

fn mode_printer<T: SerialPort>(mut port: &mut BufStream<T>) -> Result<(), io::Error> {
    loop {
        try!(port.write_all(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x00]));
        try!(port.flush());
        // Wait for the magic bytes.
        try!(read_until_magic(&mut port, &PRINT_MAGIC));
        let mut buf = vec![0; 4];
        // Read cmd, arg1, arg2, arg3
        try!(port.read_exact(&mut buf));
        let cmd = buf[0];
        let args = &buf[1..4];
        println!("cmd, args: {:02x} {:02x} {:02x} {:02x}",
                 cmd,
                 args[0],
                 args[1],
                 args[2]);
        let len = (args[1] as u16) + ((args[2] as u16) << 8);
        let mut payload = vec![0; len as usize];
        let mut checksum = vec![0; 2];
        try!(port.read_exact(&mut payload));
        try!(port.read_exact(&mut checksum));
        println!("payload: {:?}", payload);
        println!("checksum: {:02x} {:02x}", checksum[0], checksum[1]);
        match PrintCommand::from_u8(cmd) {
            Some(PrintCommand::Init) => {}
            Some(PrintCommand::Print) => {}
            Some(PrintCommand::Data) => {}
            Some(PrintCommand::Status) => {}
            None => {}
        }
        let mut ack_status = vec![0; 2];
        try!(port.read_exact(&mut ack_status));
        println!("ack, status = {:02x} {:02x}", ack_status[0], ack_status[1]);
        // try!(port.write_all(&[0x80, 0x00]));
        // try!(port.flush());
    }
    // print!("{}", PrintCommand::Status as u8);
    // print!("{:?}", PrintCommand::from_u8(0x0f));
    // return Ok(());
}
