# ModbusBuffer

`ModbusBuffer` is a Rust crate providing a circular buffer implementation tailored for Modbus communication. 
It's designed to handle the buffering of Modbus frames efficiently, with features such as configurable frame length 
and optional data overwriting when the buffer is full. 
This crate is especially useful in systems where no standard library is available (i.e., `no_std` environments).
The initial purpose of this crate was to allow decoding Modbus stream over RS485 without care out the `silences`.

## Features

- **Circular Buffer**: Efficiently manages a circular buffer for modbus storage.
- **Configurable Frame Lengths**: Set minimum and maximum frame sizes.
- **Overwrite Option**: Choose between overwriting old data or panicking when the buffer is full.
- **CRC Check**: Includes functionality to compute and verify CRC16 for Modbus frames.
- **Frame Decoding**: Attempts to decode frames continuously as each byte is received, which is ideal 
for scenarios where silence (indicating the end of a frame) cannot be reliably detected.

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
modbus_buffer = "0.0.2"
```

## Example usage:

```rust
use modbus_buffer::ModbusBuffer;

fn main() {
    let mut buffer = ModbusBuffer::<16>::new()
        .min_frame_len(3)
        .max_frame_len(8)
        .overwrite(true);
    
    // start receiving data in the middle of a request
    buffer.push(0xCD);  // value
    buffer.push(0x9F);  // crc
    buffer.push(0xBE);  // crc

    // Receive a second request
    buffer.push(0x12);  // unit address
    buffer.push(0x06);  // fn code
    buffer.push(0x22);  // addr
    buffer.push(0x22);  // addr
    buffer.push(0xAB);  // value
    buffer.push(0xCD);  // value
    buffer.push(0x9F);  // crc
    buffer.push(0xBE);  // crc
    
    let mut output = [0u8; 16];
    // Here, the 'noise' from previous request will be dropped and the second request detected
    let len = buff.try_decode_frame(&mut output);

    
}
```

## Contributing

Contributions are welcome! Please feel free to submit pull requests or create issues if you find bugs 
or have suggestions for improvements.