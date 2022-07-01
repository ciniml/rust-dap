#[cfg(test)]
mod tests {
    use core::time::Duration;
    use rand::Rng;
    use serialport;
    use std::sync::mpsc::channel;
    use std::thread::spawn;

    #[test]
    fn loopback() {
        let port = serialport::new("/dev/ttyACM1", 921600)
            .stop_bits(serialport::StopBits::One)
            .data_bits(serialport::DataBits::Eight)
            .timeout(Duration::from_secs(5))
            .open()
            .unwrap_or_else(|e| {
                eprintln!("Failed to open \"{}\"", e);
                ::std::process::exit(1);
            });

        const NUMBER_OF_TRANSFERS: usize = 50;
        const BUFFER_SIZE: usize = 32;

        let (tx_send, tx_recv) = channel();
        let (rx_send, rx_recv) = channel();
        // Fill write buffer
        for _ in 0..10 {
            let buffer = vec![0u8; BUFFER_SIZE];
            tx_send.send(buffer).unwrap();
        }
        // Fill read buffer
        {
            let buffer = vec![0u8; BUFFER_SIZE];
            rx_send.send(buffer).unwrap();
        }

        let mut read_port = port.try_clone().expect("Failed to clone port.");
        let mut write_port = port;

        let write_thread = spawn(move || {
            let mut rng = rand::thread_rng();
            for _ in 0..NUMBER_OF_TRANSFERS {
                let mut buffer = tx_recv.recv().unwrap();
                //rng.fill(buffer.as_mut_slice());
                for i in 0..buffer.len() {
                    buffer[i] = i as u8;
                }
                write_port.write_all(&buffer).unwrap();
                rx_send.send(buffer).unwrap();
            }
        });
        let read_thread = spawn(move || {
            let mut buffer = rx_recv.recv().unwrap();
            for i in 0..NUMBER_OF_TRANSFERS {
                let expected = rx_recv.recv().unwrap();
                read_port.read_exact(buffer.as_mut_slice()).unwrap();
                assert_eq!(buffer, expected, "Failed at transfer {}", i);
                tx_send.send(buffer).ok();
                buffer = expected;
            }
        });

        read_thread.join().unwrap();
        write_thread.join().unwrap();
    }
}
