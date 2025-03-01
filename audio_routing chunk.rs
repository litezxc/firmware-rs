use audio::audio_filter::{sample_to_f32, sample_to_u32};
use defmt::info;
use embassy_stm32::time::Hertz;
use embassy_stm32::{i2s, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_time::{Duration, WithTimeout as _};
use embassy_futures::join;

use crate::*;

#[allow(unused)]
pub struct I2sResources1<'d> {
    pub i2s: peripherals::SPI2,
    pub ck: peripherals::PB10,
    pub sd: peripherals::PB15,
    pub ws: peripherals::PB12,
    pub dma: peripherals::DMA1_CH4,
    pub dma_buf: &'d mut [u16],
}

#[allow(unused)]
pub struct I2sResources2<'d> {
    pub i2s: peripherals::SPI3,
    pub ck: peripherals::PB3,
    pub sd: peripherals::PB5,
    pub ws: peripherals::PA4, // Используем правильный пин для SPI3
    pub dma: peripherals::DMA1_CH5,
    pub dma_buf: &'d mut [u16],
}

fn new_i2s<'d>(resources: &'d mut I2sResources1) -> i2s::I2S<'d, u16> {
    let mut config = i2s::Config::default();
    config.standard = i2s::Standard::Philips;
    config.format = i2s::Format::Data32Channel32;
    config.master_clock = false;

    i2s::I2S::new_txonly_nomck(
        &mut resources.i2s,
        &mut resources.sd,
        &mut resources.ws,
        &mut resources.ck,
        &mut resources.dma,
        resources.dma_buf,
        Hertz(SAMPLE_RATE_HZ),
        config,
    )
}

fn new_i2s2<'d>(resources: &'d mut I2sResources2) -> i2s::I2S<'d, u16> {
    let mut config = i2s::Config::default();
    config.standard = i2s::Standard::Philips;
    config.format = i2s::Format::Data32Channel32;
    config.master_clock = false;

    i2s::I2S::new_txonly_nomck(
        &mut resources.i2s,
        &mut resources.sd,
        &mut resources.ws,
        &mut resources.ck,
        &mut resources.dma,
        resources.dma_buf,
        Hertz(SAMPLE_RATE_HZ),
        config,
    )
}

#[embassy_executor::task]
pub async fn audio_routing_task(
    mut i2s_resources1: I2sResources1<'static>,
    mut i2s_resources2: I2sResources2<'static>,
    mut usb_audio_receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, UsbSampleBlock>,
) {
    let mut volume = (0.0, 0.0);
    let mut i2s_dac1 = new_i2s(&mut i2s_resources1);
    let mut i2s_dac2 = new_i2s2(&mut i2s_resources2);
    let mut running = false;

    // Статические буферы для обработки данных
    let mut processed_samples_left = [0u16; USB_MAX_SAMPLE_COUNT * 2];
    let mut processed_samples_right = [0u16; USB_MAX_SAMPLE_COUNT * 2];

    loop {
        // Data should arrive at least once every millisecond.
        let result = usb_audio_receiver
            .receive()
            .with_timeout(Duration::from_millis(1)) // Оптимальное время ожидания
            .await;

        if let Some(new_volume) = VOLUME_SIGNAL.try_take() {
            volume = new_volume;
        }

        let error = if let Ok(samples) = result {
            let mut index = 0;

            // Обработка сэмплов блоками по 2 (левый и правый канал)
            for chunk in samples.chunks_exact(2) {
                let left_sample_f32 = sample_to_f32(chunk[0]);
                let right_sample_f32 = sample_to_f32(chunk[1]);

                let scaled_left = left_sample_f32 * volume.0;
                let scaled_right = right_sample_f32 * volume.1;

                let left_sample = sample_to_u32(scaled_left);
                let right_sample = sample_to_u32(scaled_right);

                // Записываем сэмплы в буферы для левого и правого каналов
                // Левый канал: данные в первом сэмпле, второй сэмпл = 0
                processed_samples_left[index] = (left_sample >> 16) as u16;
                processed_samples_left[index + 1] = left_sample as u16;
                processed_samples_left[index + 2] = 0; // Второй сэмпл = 0
                processed_samples_left[index + 3] = 0;

                // Правый канал: данные в первом сэмпле, второй сэмпл = 0
                processed_samples_right[index] = (right_sample >> 16) as u16;
                processed_samples_right[index + 1] = right_sample as u16;
                processed_samples_right[index + 2] = 0; // Второй сэмпл = 0
                processed_samples_right[index + 3] = 0;

                index += 4;
            }

            let error = if !running {
                info!("Start I2S");

                i2s_dac1.clear();
                i2s_dac2.clear();
                let (result1, result2) = join::join(
                    i2s_dac1.write_immediate(&processed_samples_left[..index]),
                    i2s_dac2.write_immediate(&processed_samples_right[..index]),
                ).await;
                i2s_dac1.start();
                i2s_dac2.start();
                running = true;
                result1.is_err() || result2.is_err()
            } else {
                let (result1, result2) = join::join(
                    i2s_dac1.write(&processed_samples_left[..index]),
                    i2s_dac2.write(&processed_samples_right[..index]),
                ).await;
                result1.is_err() || result2.is_err()
            };

            // Notify the channel that the buffer is now ready to be reused
            usb_audio_receiver.receive_done();

            error
        } else {
            true
        };

        // Stop I2S in case of errors or stopped streaming.
        if error && running {
            info!("Stop I2S");

            i2s_dac1.stop().await;
            i2s_dac2.stop().await;
            drop(i2s_dac1);
            drop(i2s_dac2);
            i2s_dac1 = new_i2s(&mut i2s_resources1);
            i2s_dac2 = new_i2s2(&mut i2s_resources2);
            running = false;
        }
    }
}