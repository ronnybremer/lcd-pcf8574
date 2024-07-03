#![doc = include_str!("../README.md")]
#![deny(missing_docs)]

use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use lcd::{Delay, Hardware};
use std::thread;
use std::time::Duration;

/// Represents an LCD display attached via PCF8574 I2C expander. Use the traits in the [`lcd`]
/// crate to interact with it.
pub struct Pcf8574 {
    dev: LinuxI2CDevice,
    data: u8,
    on_err: ErrorHandling,
    hardware_type: HardwareType,
}

/// What to do on I/O errors.
pub enum ErrorHandling {
    /// Ignore write errors.
    None,

    /// Panic on write errors.
    Panic,

    /// Run a custom handler function.
    Custom(Box<dyn FnMut(LinuxI2CError) + 'static>),
}

/// Hardware type of the LCD display
///
/// Some vendors have a different GPIO pin mapping.
pub enum HardwareType {
    /// Default
    ///
    /// GPIO pin / Usage
    /// 0: RS, 1: RW, 2: EN, 3: BL, 4: D1, 5: D2, 6: D3, 7: D4 
    Default,

    /// Joy-IT RB-LCD-16x2 and RB-LCD-20x4
    ///
    /// GPIO pin / Usage
    /// 0: D1, 1: D2, 2: D3, 3: D4, 4: RS, 5: RW, 6: unused, 7: RS
    ///
    /// backlight is always on and can't be turned off
    JoyIT,
}

impl Pcf8574 {
    /// Create a new instance, using the Linux I2C interface for communication. `bus` is the number
    /// of `/dev/i2c-<bus>` to use, and `address` is the I2C address of the device.
    ///
    /// After opening the device, defaults to ignoring all I/O errors; see [`Self::on_error`] and
    /// [`ErrorHandling`] for how to change this behavior.
    pub fn new(bus: u8, address: u16) -> Result<Self, LinuxI2CError> {
        Ok(Self {
            dev: LinuxI2CDevice::new(format!("/dev/i2c-{}", bus), address)?,
            data: 0b0000_1000, // backlight on by default
            on_err: ErrorHandling::None,
            hardware_type: HardwareType::Default,
        })
    }

    /// Change the I/O error handling strategy.
    ///
    /// [`lcd::Hardware`] doesn't have any way to return errors to the caller, so error handling
    /// has to be done here, internally.
    pub fn on_error(&mut self, on_err: ErrorHandling) {
        self.on_err = on_err;
    }

    /// Set the hardware type.
    pub fn set_hardware_type(&mut self, hardware_type: HardwareType) {
        self.hardware_type = hardware_type;
    }

    /// Set the display's backlight on or off.
    pub fn backlight(&mut self, on: bool) {
        match self.hardware_type {
            HardwareType::Default => {
                self.set_bit(3, on);
                self.apply();
            },
            HardwareType::JoyIT => {},
        }
    }

    fn set_bit(&mut self, offset: u8, bit: bool) {
        if bit {
            self.data |= 1 << offset;
        } else {
            self.data &= !(1 << offset);
        }
    }
}

impl Hardware for Pcf8574 {
    fn rs(&mut self, bit: bool) {
        match self.hardware_type {
            HardwareType::Default => {
                self.set_bit(0, bit);
            },
            HardwareType::JoyIT => {
                self.set_bit(4, bit);
            },
        }
    }

    fn rw(&mut self, bit: bool) {
        match self.hardware_type {
            HardwareType::Default => {},
            HardwareType::JoyIT => {
                self.set_bit(5, bit);
            },
        }
    }

    fn enable(&mut self, bit: bool) {
        match self.hardware_type {
            HardwareType::Default => {
                self.set_bit(2, bit);
            },
            HardwareType::JoyIT => {
                self.set_bit(7, bit);
            },
        }
    }

    fn data(&mut self, bits: u8) {
        assert!(bits & 0xF0 == 0, "4-bit mode is required");
        match self.hardware_type {
            HardwareType::Default => {
                self.data = (self.data & 0x0F) | (bits << 4);
            },
            HardwareType::JoyIT => {
                self.data = (self.data & 0xF0) | bits;
            },
        }
    }

    fn apply(&mut self) {
        if let Err(e) = self.dev.smbus_write_byte(self.data) {
            match &mut self.on_err {
                ErrorHandling::None => (),
                ErrorHandling::Panic => panic!("smbus_write_byte failed: {}", e),
                ErrorHandling::Custom(f) => f(e),
            }
        }
    }
}

impl Delay for Pcf8574 {
    fn delay_us(&mut self, delay_usec: u32) {
        thread::sleep(Duration::from_micros(u64::from(delay_usec)));
    }
}
