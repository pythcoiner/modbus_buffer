#![no_std]

#[derive(Debug)]
pub struct ModbusBuffer<const CAPACITY: usize> {
    ring_buffer: [Option<u8>;CAPACITY],
    /// Head of data (Oldest Byte)
    head: usize,
    /// Tail of data (Newest Byte)
    tail: usize,
    size: usize,
    /// Min frame length to be detected (CRC not included) (Default: 3)
    min_frame_len: usize,
    /// Max frame length to be detected (CRC not included) Default: CAPACITY
    max_frame_len: usize,
    /// Whether the buffer should be overwritten if overflowed, or panic (Default: `true`).
    overwrite: bool,
}
impl<const CAPACITY: usize> ModbusBuffer<CAPACITY> {

    /// Creates a new `ModbusBuffer` with specified constants.
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        
        assert!(CAPACITY > 4);
        ModbusBuffer {
            ring_buffer: [None; CAPACITY],
            head: 0,
            tail: 0,
            size: 0,
            min_frame_len: 3,
            max_frame_len: CAPACITY,
            overwrite: true,
        }
    }

    /// Sets the minimum frame length required to detect a Modbus frame, excluding CRC.
    pub fn min_frame_len(mut self, min_frame_len: usize) -> Self {
        self.min_frame_len = min_frame_len;
        self
    }

    /// Sets the maximum frame length that can be detected, excluding CRC.
    pub fn max_frame_len(mut self, max_frame_len: usize) -> Self {
        self.max_frame_len = max_frame_len;
        self
    }

    /// Configures whether to overwrite old data if the buffer is full or to panic.
    pub fn overwrite(mut self, overwrite: bool) -> Self {
            self.overwrite = overwrite;
            self
        }

    /// Adds an item to the buffer, handling overflow based on the `overwrite` flag.

    pub fn push(&mut self, item: u8) {
        if self.size == CAPACITY {
            // Buffer is full
            if self.overwrite {
                self.ring_buffer[self.head] = Some(item);
                self.head = (self.head + 1) % CAPACITY;
                self.tail = (self.tail + 1) % CAPACITY;
            } else {
                panic!("ModbusBuffer exceed its capacity!");
            }

        } else {
            // Buffer has space
            self.ring_buffer[self.tail] = Some(item);
            self.tail = (self.tail + 1) % CAPACITY;
            self.size += 1;
        }
    }

    /// Removes and returns the oldest element from the buffer if available.
    pub fn pop(&mut self) -> Option<u8> {
        if self.size == 0 {
            None
        } else {
            let item = self.ring_buffer[self.head];
            self.ring_buffer[self.head] = None;
            self.head = (self.head + 1) % CAPACITY;
            self.size -= 1;
            item
        }
    }

    /// Returns the current number of elements in the buffer.
    pub fn len(&self) -> usize {
        self.size
    }

    /// Returns true if the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// Returns true if the buffer is full.
    pub fn is_full(&self) -> bool {
        self.size == CAPACITY
    }

    /// Copies the current data from the buffer into the provided output buffer and returns the size.
    fn frame(&self, output_buffer: &mut [u8;CAPACITY]) -> Option<usize> {
        let mut index = 0;

        if self.size > 0 {
            if self.head < self.tail {
                // No wrap-around, direct slice
                self.ring_buffer[self.head..self.tail]
                    .iter()
                    .for_each(|d| {
                        output_buffer[index] = d.unwrap();
                        index += 1;
                    });
            } else {
                // Wrap-around, handle two parts
                // First part from head to end of buffer
                self.ring_buffer[self.head..CAPACITY]
                    .iter()
                    .for_each(|d| {
                        output_buffer[index] = d.unwrap();
                        index += 1;
                    });

                // Second part from start of buffer to tail
                self.ring_buffer[0..self.tail]
                    .iter()
                    .for_each(|d| {
                        output_buffer[index] = d.unwrap();
                        index += 1;
                    });
            }
        }
        Some(self.size)
    }

    /// Computes the CRC16 for the provided data array.
    fn crc16(data: &[u8]) -> u16 {
        let mut crc = 0xFFFF;
        for x in data {
            crc ^= u16::from(*x);
            for _ in 0..8 {
                // if we followed clippy's suggestion to move out the crc >>= 1, the condition may not be met any more
                // the recommended action therefore makes no sense and it is better to allow this lint
                #[allow(clippy::branches_sharing_code)]
                if (crc & 0x0001) != 0 {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        crc << 8 | crc >> 8
    }

    /// Verifies the CRC of the provided frame.
    fn check_crc(frame: &[u8]) -> bool {
        if frame.len() > 4 {
            let crc = Self::crc16(&frame[..frame.len()-2]);
            let expected_crc = [((crc & 0xff00) >> 8) as u8, (crc & 0x00ff) as u8];
            expected_crc == frame[frame.len()-2..]
        } else {
            false
        }

    }

    /// Tries to find a valid Modbus frame in the buffer.
    fn try_decode_buffer(&self, buffer: &[u8]) -> Option<(usize, usize)> {
        let mut window_size = self.min_frame_len + 2;
        if buffer.len() < window_size {
            return None
        }

        while window_size <= buffer.len() {
            for i in 0..=buffer.len() - window_size {
                // Forward direction
                if Self::check_crc(&buffer[i..i + window_size]) {
                    return Some((i, i + window_size - 2));
                }

                if buffer.len() == window_size {
                    return None;
                }
                // Reverse direction
                let j = buffer.len() - i - window_size;
                if Self::check_crc(&buffer[j..j + window_size]) {
                    return Some((j, j + window_size - 2));
                }
            }
            window_size += 1;
        }
        None
    }

    /// Attempts to decode a Modbus frame from the internal buffer and copies it into the provided buffer if successful.
    pub fn try_decode_frame(&mut self, buffer: &mut [u8;CAPACITY]) -> Option<usize> {
        if self.size == 0 || self.size < self.min_frame_len {
            return None
        }

        let mut frame = [0u8;CAPACITY];
        // copy data into `frame`
        let len = self.frame(&mut frame).expect("Should have a frame");

        if let Some((head, tail)) = self.try_decode_buffer(&frame[..len]) {
            // if CRC match
            
            ///  println! is std, not available w/ `#![no_std]` flag
            // println!(" ");
            // println!("---------------- CRC Match!! ------------------------");
            // println!("len={}, tail={}, head={},", len, tail, head);
            // println!("frame={:?}", frame);
            // println!("self={:?}", self);
            // println!("---------------- CRC Match end ------------------------");

            // remove decoded data
            // let len_to_remove = len - tail + 2 ;
            let len_to_remove = tail + 2 ;
            for _ in 0..len_to_remove {
                self.pop();
            }

            // output frame length
            let frame_length = tail - head;

            // copy data
            buffer[..frame_length].copy_from_slice(&frame[head..tail]);

            // return frame length
            Some(frame_length)
        } else {
            None
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn test_overflow_should_panic() {
        let mut buff = ModbusBuffer::<8>::new()
            .overwrite(false);

        buff.push(0x00);
        buff.push(0x01);
        buff.push(0x02);
        buff.push(0x03);
        buff.push(0x04);
        buff.push(0x05);
        buff.push(0x06);
        buff.push(0x07);

        // Should panic here
        buff.push(0x08);

    }

    #[test]
    fn test_overflow_should_not_panic() {
        let mut buff = ModbusBuffer::<8>::new();

        buff.push(0x00);
        buff.push(0x01);
        buff.push(0x02);

        let mut buffer = [0u8;8];
        let len = buff.frame(&mut buffer);
        assert_eq!(len, Some(3usize));
        assert_eq!(buffer, [0u8, 1, 2, 0, 0, 0, 0, 0]);

        buff.push(0x03);
        buff.push(0x04);
        buff.push(0x05);
        buff.push(0x06);
        buff.push(0x07);

        let len = buff.frame(&mut buffer);
        assert_eq!(len, Some(8usize));
        assert_eq!(buffer, [0u8, 1, 2, 3, 4, 5, 6, 7]);

        // overflow + overwrite here
        buff.push(0x08);

        let len = buff.frame(&mut buffer);
        assert_eq!(len, Some(8usize));
        assert_eq!(buffer, [1u8, 2, 3, 4, 5, 6, 7, 8]);

    }

    #[test]
    fn test_receive_request() {
        let mut buff = ModbusBuffer::<10>::new();

        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        let mut output = [0u8;10];
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(output, [0x12u8, 0x06, 0x22, 0x22, 0xAB, 0xCD, 0, 0, 0, 0]);

    }

    #[test]
    fn test_receive_multiple_request() {
        let mut buff = ModbusBuffer::<10>::new();
        let mut output = [0u8;10];

        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code

        // request is not yet complete
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, None);

        buff.push(0x22);    // addr
        buff.push(0x22);    // addr

        // request is not yet complete
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, None);

        buff.push(0xAB);    // value
        buff.push(0xCD);    // value

        // request is not yet complete
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, None);

        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        // now request is complete
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));

        assert_eq!(output, [0x12u8, 0x06, 0x22, 0x22, 0xAB, 0xCD, 0, 0, 0, 0]);

        // buffer should be empty
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, None);

        // new request
        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        let mut output = [0u8;10];
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(output, [0x12u8, 0x06, 0x22, 0x22, 0xAB, 0xCD, 0, 0, 0, 0]);

        // new request
        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        let mut output = [0u8;10];
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(output, [0x12u8, 0x06, 0x22, 0x22, 0xAB, 0xCD, 0, 0, 0, 0]);

        // new request
        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        let mut output = [0u8;10];
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(output, [0x12u8, 0x06, 0x22, 0x22, 0xAB, 0xCD, 0, 0, 0, 0]);

        // new request
        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        let mut output = [0u8;10];
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(output, [0x12u8, 0x06, 0x22, 0x22, 0xAB, 0xCD, 0, 0, 0, 0]);

    }

    #[test]
    fn test_noise_on_head() {
        let mut buff = ModbusBuffer::<10>::new();

        buff.push(0x01);    // noise
        assert_eq!(buff.len(), 1usize);
        buff.push(0x02);    // noise
        assert_eq!(buff.len(), 2usize);
        buff.push(0x03);    // noise
        assert_eq!(buff.len(), 3usize);
        buff.push(0x01);    // noise
        assert_eq!(buff.len(), 4usize);
        buff.push(0x02);    // noise
        assert_eq!(buff.len(), 5usize);
        buff.push(0x03);    // noise
        assert_eq!(buff.len(), 6usize);

        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        // buff.len() should be clamp to max capacity
        assert_eq!(buff.len(), 10usize);

        let mut output = [0u8;10];
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(output, [0x12u8, 0x06, 0x22, 0x22, 0xAB, 0xCD, 0, 0, 0, 0]);
        
        assert_eq!(buff.len(), 0usize);

    }

    #[test]
    fn test_data_remain_on_tail() {
        let mut buff = ModbusBuffer::<20>::new();
        
        // first request
        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        // second request
        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc
        
        let mut output = [0u8;20];
        
        // decode the first request but do not remove the second request on tail
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(buff.len(), 8usize);

        // decode second request
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(buff.len(), 0usize);

    }

    #[test]
    fn test_data_remain_on_tail_with_overlap() {
        // same than previous test but buffer cannot contain 2 request
        
        let mut buff = ModbusBuffer::<15>::new();

        // first request
        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        // second request
        buff.push(0x12);    // slave addr
        buff.push(0x06);    // function code
        buff.push(0x22);    // addr
        buff.push(0x22);    // addr
        buff.push(0xAB);    // value
        buff.push(0xCD);    // value
        buff.push(0x9F);    // crc
        buff.push(0xBE);    // crc

        let mut output = [0u8;15];

        // decode the second request, first one have been partially overwritten
        let len = buff.try_decode_frame(&mut output);
        assert_eq!(len, Some(6));
        assert_eq!(buff.len(), 0usize);
    }
    
    #[test]
    fn buffer_empty() {
        let mut buff = ModbusBuffer::<10>::new();
        
        assert_eq!(buff.len(), 0);
        let p = buff.pop();
        assert_eq!(p, None);
        let mut temp = [0u8;10];
        let q = buff.try_decode_frame(&mut temp);
        assert_eq!(q, None);
    }
    
}
