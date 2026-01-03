use embedded_hal::spi::MODE_3;
use embedded_hal::delay::DelayNs;
use std::sync::Mutex;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{
    Gpio1, Gpio10, Gpio11, Gpio12, Gpio15, Gpio16, Gpio2, Gpio40, Gpio41, Gpio42, Gpio6, Gpio7, Gpio8, Gpio9, Output,
    PinDriver,
};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{self, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;

struct ImuHardware {
    spi: SpiDeviceDriver<'static, SpiDriver<'static>>,
    chip_select: PinDriver<'static, Gpio12, Output>,
    cs_bt: PinDriver<'static, Gpio1, Output>,
    cs_lf: PinDriver<'static, Gpio16, Output>,
    cs_ls: PinDriver<'static, Gpio15, Output>,
    cs_rs: PinDriver<'static, Gpio41, Output>,
    cs_rf: PinDriver<'static, Gpio40, Output>,

    en_lf: PinDriver<'static, Gpio6, Output>,
    en_ls: PinDriver<'static, Gpio7, Output>,
    en_rs: PinDriver<'static, Gpio42, Output>,
    en_rf: PinDriver<'static, Gpio2, Output>,
}

static INSTANCE: Mutex<Option<Imu>> = Mutex::new(None);

pub struct Imu {
    hardware: ImuHardware,
}

impl Imu {
    /// Initialize the IMU hardware and create a singleton instance
    pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
        let mut instance = INSTANCE.lock().unwrap();
        if instance.is_some() {
            return Err(anyhow::anyhow!("IMU already initialized"));
        }

        let hardware = unsafe {
            let spi = peripherals.spi2.clone_unchecked();
            let sclk = peripherals.pins.gpio11.clone_unchecked();
            let sdo = peripherals.pins.gpio10.clone_unchecked();
            let sdi = peripherals.pins.gpio9.clone_unchecked();
            let cs = peripherals.pins.gpio12.clone_unchecked();
            let cs_batt = peripherals.pins.gpio1.clone_unchecked();
            let cs_lf = peripherals.pins.gpio16.clone_unchecked();
            let cs_ls = peripherals.pins.gpio15.clone_unchecked();
            let cs_rs = peripherals.pins.gpio41.clone_unchecked();
            let cs_rf = peripherals.pins.gpio40.clone_unchecked();
            let en_lf = peripherals.pins.gpio6.clone_unchecked();
            let en_ls = peripherals.pins.gpio7.clone_unchecked();
            let en_rs = peripherals.pins.gpio42.clone_unchecked();
            let en_rf = peripherals.pins.gpio2.clone_unchecked();

            let config = spi::config::Config::new()
                .baudrate(10.MHz().into())
                .data_mode(MODE_3);
            let spi = SpiDeviceDriver::new_single(
                spi,
                sclk,
                sdo,
                Some(sdi),
                None as Option<Gpio12>,
                &SpiDriverConfig::new(),
                &config,
            )?;

            ImuHardware {
                spi,
                chip_select: PinDriver::output(cs)?,
                cs_bt: PinDriver::output(cs_batt)?,
                cs_lf: PinDriver::output(cs_lf)?,
                cs_ls: PinDriver::output(cs_ls)?,
                cs_rs: PinDriver::output(cs_rs)?,
                cs_rf: PinDriver::output(cs_rf)?,
                en_lf: PinDriver::output(en_lf)?,
                en_ls: PinDriver::output(en_ls)?,
                en_rs: PinDriver::output(en_rs)?,
                en_rf: PinDriver::output(en_rf)?,
            }
        };

        let mut imu = Imu { hardware };
        
        // Initialize sensors to off state
        imu.off_lf()?;
        imu.off_ls()?;
        imu.off_rs()?;
        imu.off_rf()?;

        let mut r_buffer = [0x00, 0x00];

        // Set 1 to FUNC_CFG_ACCES bit in FUNC_CFG_ACCESS (01h) register,
        // to enable writing to configuration registers
        let w_buffer = [0x01, 0x80];
        imu.transfer(&mut r_buffer, &w_buffer)?;

        // Configure CTRL2_G (11h) register
        //   Data rate : 6.66kHz (high performance)
        //   Full scale : +/-2000[dps]
        let w_buffer = [0x11, 0xac];
        imu.transfer(&mut r_buffer, &w_buffer)?;

        *instance = Some(imu);
        Ok(())
    }

    /// Get a mutable reference to the singleton instance
    fn with_instance_mut<F, R>(f: F) -> anyhow::Result<R>
    where
        F: FnOnce(&mut Imu) -> anyhow::Result<R>,
    {
        let mut instance = INSTANCE.lock().unwrap();
        match instance.as_mut() {
            Some(imu) => f(imu),
            None => Err(anyhow::anyhow!("IMU not initialized")),
        }
    }

    /// Transfer data to and from the IMU
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> anyhow::Result<()> {
        self.hardware.chip_select.set_low()?;
        FreeRtos.delay_us(1);
        self.hardware.spi.transfer(read, write)?;
        self.hardware.chip_select.set_high()?;
        Ok(())
    }

    /// Read gyroscope Z-axis data
    pub fn read() -> anyhow::Result<i16> {
        Self::with_instance_mut(|imu| {
            // Read OUTZ_L_G (26h) and OUTZ_H_G (27h)
            let w_buffer: [u8; 3] = [0xa6, 0xff, 0xff]; // Read 2 bytes from 0x26
            let mut r_buffer: [u8; 3] = [0, 0, 0];
            imu.transfer(&mut r_buffer, &w_buffer)?;

            let result = ((r_buffer[2] as i16) << 8) | (r_buffer[1] as i16);
            Ok(result)
        })
    }

    /// Read battery voltage
    pub fn read_batt() -> anyhow::Result<u16> {
        Self::with_instance_mut(|imu| {
            let write = [0xff, 0xff];
            let mut read = [0, 0];
            
            imu.hardware.cs_bt.set_low()?;
            imu.hardware.spi.transfer(&mut read, &write)?;
            imu.hardware.cs_bt.set_high()?;

            let result = ((read[0] as u16) << 8) | read[1] as u16;
            let result = result & 0x1fff;
            let result = result / 2;
            Ok(result)
        })
    }

    /// Read left-front distance sensor
    pub fn read_lf() -> anyhow::Result<u16> {
        Self::with_instance_mut(|imu| {
            let write = [0xff, 0xff];
            let mut read = [0, 0];
            
            imu.hardware.cs_lf.set_low()?;
            imu.hardware.spi.transfer(&mut read, &write)?;
            imu.hardware.cs_lf.set_high()?;

            let result = ((read[0] as u16) << 8) | read[1] as u16;
            let result = result & 0x1fff;
            let result = result / 2;
            Ok(result)
        })
    }

    /// Read left-side distance sensor
    pub fn read_ls() -> anyhow::Result<u16> {
        Self::with_instance_mut(|imu| {
            let write = [0xff, 0xff];
            let mut read = [0, 0];
            
            imu.hardware.cs_ls.set_low()?;
            imu.hardware.spi.transfer(&mut read, &write)?;
            imu.hardware.cs_ls.set_high()?;

            let result = ((read[0] as u16) << 8) | read[1] as u16;
            let result = result & 0x1fff;
            let result = result / 2;
            Ok(result)
        })
    }

    /// Read right-front distance sensor
    pub fn read_rf() -> anyhow::Result<u16> {
        Self::with_instance_mut(|imu| {
            let write = [0xff, 0xff];
            let mut read = [0, 0];
            
            imu.hardware.cs_rf.set_low()?;
            imu.hardware.spi.transfer(&mut read, &write)?;
            imu.hardware.cs_rf.set_high()?;

            let result = ((read[0] as u16) << 8) | read[1] as u16;
            let result = result & 0x1fff;
            let result = result / 2;
            Ok(result)
        })
    }

    /// Read right-side distance sensor
    pub fn read_rs() -> anyhow::Result<u16> {
        Self::with_instance_mut(|imu| {
            let write = [0xff, 0xff];
            let mut read = [0, 0];
            
            imu.hardware.cs_rs.set_low()?;
            imu.hardware.spi.transfer(&mut read, &write)?;
            imu.hardware.cs_rs.set_high()?;

            let result = ((read[0] as u16) << 8) | read[1] as u16;
            let result = result & 0x1fff;
            let result = result / 2;
            Ok(result)
        })
    }

    /// Turn on left-front distance sensor
    pub fn on_lf(&mut self) -> anyhow::Result<()> {
        self.hardware.en_lf.set_high()?;
        Ok(())
    }

    /// Turn off left-front distance sensor
    pub fn off_lf(&mut self) -> anyhow::Result<()> {
        self.hardware.en_lf.set_low()?;
        Ok(())
    }

    /// Turn on left-side distance sensor
    pub fn on_ls(&mut self) -> anyhow::Result<()> {
        self.hardware.en_ls.set_high()?;
        Ok(())
    }

    /// Turn off left-side distance sensor
    pub fn off_ls(&mut self) -> anyhow::Result<()> {
        self.hardware.en_ls.set_low()?;
        Ok(())
    }

    /// Turn on right-front distance sensor
    pub fn on_rf(&mut self) -> anyhow::Result<()> {
        self.hardware.en_rf.set_high()?;
        Ok(())
    }

    /// Turn off right-front distance sensor
    pub fn off_rf(&mut self) -> anyhow::Result<()> {
        self.hardware.en_rf.set_low()?;
        Ok(())
    }

    /// Turn on right-side distance sensor
    pub fn on_rs(&mut self) -> anyhow::Result<()> {
        self.hardware.en_rs.set_high()?;
        Ok(())
    }

    /// Turn off right-side distance sensor
    pub fn off_rs(&mut self) -> anyhow::Result<()> {
        self.hardware.en_rs.set_low()?;
        Ok(())
    }
}
