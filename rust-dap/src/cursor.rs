// Copyright 2021 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum CursorError {
    InsufficientBuffer,
    NotEnoughData,
}
pub trait CursorRead {
    fn read<'a>(&mut self, buffer: &'a mut [u8]) -> Result<&'a [u8], CursorError>;
}
pub trait CursorWrite {
    fn write(&mut self, data: &[u8]) -> Result<(), CursorError>;
}

pub struct BufferCursor<Buffer> {
    buffer: Buffer,
    position: usize,
}

impl<Buffer> BufferCursor<Buffer> {
    pub fn new(buffer: Buffer) -> Self {
        Self {
            buffer,
            position: 0,
        }
    }
    pub fn new_with_position(buffer: Buffer, position: usize) -> Self {
        Self {
            buffer,
            position: position,
        }
    }
    #[allow(dead_code)]
    pub fn release(self) -> Buffer {
        self.buffer
    }
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.position = 0;
    }
    pub fn get_position(&self) -> usize {
        self.position
    }
}

impl<Buffer: AsRef<[u8]>> CursorRead for BufferCursor<Buffer> {
    fn read<'a>(&mut self, buffer: &'a mut [u8]) -> Result<&'a [u8], CursorError> {
        let data = self.buffer.as_ref();
        let remaining = data.len() - self.position;
        let bytes_to_read = core::cmp::min(remaining, buffer.len());
        buffer.copy_from_slice(&data[self.position..self.position + bytes_to_read]);
        self.position += bytes_to_read;
        Ok(&buffer[0..bytes_to_read])
    }
}

impl<Buffer: AsMut<[u8]>> CursorWrite for BufferCursor<Buffer> {
    fn write(&mut self, data: &[u8]) -> Result<(), CursorError> {
        let bytes_to_write = data.len();
        let remaining = self.buffer.as_mut().len() - self.position;
        if remaining < bytes_to_write {
            Err(CursorError::InsufficientBuffer)
        } else {
            let buffer: &mut [u8] = self.buffer.as_mut();
            buffer[self.position..self.position + bytes_to_write].copy_from_slice(data);
            self.position += bytes_to_write;
            Ok(())
        }
    }
}
