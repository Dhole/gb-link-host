# Game Boy Link communication, host side

This is the host side of a project that allows communication between a physical Game Boy Game Link Cable and a computer.

Three functionalities are implemented: 

- Sniffing the serial communication
- Virtual Game Boy Printer that stores the images as PNG files
- Printing directly to the Game Boy Printer (no Game Boy needed)

The code is written in Rust.

## Details

You can read the complete details of this project in my blog posts:

- [Sniffing Game Boy serial traffic with an STM32F4](https://dhole.github.io/post/gameboy_serial_1/)
- [Virtual Game Boy Printer with an STM32F4](https://dhole.github.io/post/gameboy_serial_2/)
- [Printing on the Game Boy Printer using an STM32F4](https://dhole.github.io/post/gameboy_serial_3/)

## Usage

```
USAGE:
    gb-link [OPTIONS]

FLAGS:
    -h, --help       Prints help information
    -V, --version    Prints version information

OPTIONS:
    -b, --baud <RATE>        Set the baud rate [default: 1000000]
    -d, --board <BOARD>      Set the development board: generic, st [default: st]
    -f, --file <FILE>        Image file to print
    -m, --mode <MODE>        Set the device mode: sniff, printer, print [default: sniff]
    -s, --serial <DEVICE>    Set the serial device [default: /dev/ttyACM0]
```

## License

The code is released under the 3-clause BSD License.
