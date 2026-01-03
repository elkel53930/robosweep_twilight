use esp_idf_hal::{delay::FreeRtos, peripherals::Peripherals};
pub mod imu;

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, TWILIGHT!");

    let mut peripherals = Peripherals::take().unwrap();

    imu::Imu::init(&mut peripherals)?;

    loop {
        let battery_voltage = imu::Imu::read_batt()?;
        log::info!("Battery voltage: {}", battery_voltage);
        FreeRtos::delay_ms(1000);
    }
}
