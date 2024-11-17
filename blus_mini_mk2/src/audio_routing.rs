use audio::{audio_filter, AudioFilter};
use core::sync::atomic::Ordering::Relaxed;
use defmt::{debug, info, panic};
use embassy_futures::select::{select, Either, Select};
use embassy_stm32::gpio::Output;
use embassy_stm32::sai::word;
use embassy_stm32::{peripherals, sai};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel;
use embassy_time::{Duration, Instant, WithTimeout};
use grounded::uninit::GroundedArrayCell;
use static_assertions;

use crate::*;

// Generous sample buffer
const SAI_AMP_SAMPLE_COUNT: usize = 2 * (OUTPUT_CHANNEL_COUNT / INPUT_CHANNEL_COUNT) * MAX_SAMPLE_COUNT;

// Buffer of around 1 ms size
const SAI_RPI_SAMPLE_COUNT: usize = SAMPLE_SIZE_PER_S.div_ceil(1000) / SAMPLE_SIZE;

// Do not exceed the write buffer (after processing) with samples from the Raspberry Pi.
static_assertions::const_assert!(
    (OUTPUT_CHANNEL_COUNT / INPUT_CHANNEL_COUNT) * SAI_RPI_SAMPLE_COUNT <= SAI_AMP_SAMPLE_COUNT
);

#[allow(unused)]
pub struct Sai1Resources {
    pub sai: peripherals::SAI1,

    pub mclk_a: peripherals::PE2,
    pub sck_a: peripherals::PE5,
    pub sd_a: peripherals::PE6,
    pub fs_a: peripherals::PE4,
    pub dma_a: peripherals::DMA1_CH3,

    pub sd_b: peripherals::PE11,
    pub dma_b: peripherals::DMA1_CH4,
}

#[allow(unused)]
pub struct Sai4Resources {
    pub sai: peripherals::SAI4,

    pub mclk_a: peripherals::PE0,
    pub sck_a: peripherals::PD13,
    pub sd_a: peripherals::PC1,
    pub fs_a: peripherals::PD12,
    pub dma_a: peripherals::BDMA_CH0,

    pub sck_b: peripherals::PE12,
    pub sd_b: peripherals::PE11,
    pub fs_b: peripherals::PE13,
    pub dma_b: peripherals::BDMA_CH1,
}

// Accessible by BDMA (Zone D3)
#[link_section = ".sram4"]
static mut SAI_AMP_WRITE_BUFFER: GroundedArrayCell<u32, SAI_AMP_SAMPLE_COUNT> = GroundedArrayCell::uninit();

#[link_section = ".sram4"]
static mut SAI_RPI_READ_BUFFER: GroundedArrayCell<u32, SAI_RPI_SAMPLE_COUNT> = GroundedArrayCell::uninit();

fn new_sai_amp_rpi<'d>(
    resources: &'d mut Sai4Resources,
    sai_amp_write_buffer: &'d mut [u32],
    sai_rpi_read_buffer: &'d mut [u32],
    sample_rate_hz: u32,
    audio_source: AudioSource,
) -> (
    sai::Sai<'d, peripherals::SAI4, u32>,
    sai::Sai<'d, peripherals::SAI4, u32>,
) {
    let clk_source = match audio_source {
        AudioSource::Spdif => embassy_stm32::pac::rcc::vals::Saiasel::_RESERVED_5,
        _ => embassy_stm32::pac::rcc::vals::Saiasel::PLL1_Q,
    };

    embassy_stm32::pac::RCC.d3ccipr().modify(|w| {
        w.set_sai4asel(clk_source);
    });

    let (sai_amp, sai_rpi) = sai::split_subblocks(&mut resources.sai);

    let sai_amp_driver = {
        let mut config = sai::Config::default();

        config.slot_count = sai::word::U4(OUTPUT_CHANNEL_COUNT as u8);
        config.slot_enable = 0xFFFF; // All slots
        config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
        config.frame_sync_active_level_length = word::U7(1);
        config.bit_order = sai::BitOrder::MsbFirst;
        config.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;

        match audio_source {
            AudioSource::Spdif => {
                config.data_size = sai::DataSize::Data16;
                config.frame_length = (OUTPUT_CHANNEL_COUNT * 16) as u8;

                config.master_clock_divider = sai::MasterClockDivider::MasterClockDisabled;
            }
            _ => {
                assert_eq!(SAMPLE_WIDTH_BIT, 32);
                config.data_size = sai::DataSize::Data32;
                config.frame_length = (OUTPUT_CHANNEL_COUNT * 32) as u8;

                match sample_rate_hz {
                    SAMPLE_RATE_HZ => config.master_clock_divider = sai::MasterClockDivider::Div2,
                    _ => panic!("Unsupported SAI sample rate."),
                }
            }
        };

        sai::Sai::new_asynchronous(
            sai_amp,
            &mut resources.sck_a,
            &mut resources.sd_a,
            &mut resources.fs_a,
            &mut resources.dma_a,
            sai_amp_write_buffer,
            config,
        )
    };

    let sai_rpi_driver = {
        let mut config = sai::Config::default();
        const CHANNEL_COUNT: usize = INPUT_CHANNEL_COUNT;

        config.tx_rx = sai::TxRx::Receiver;
        config.slot_count = sai::word::U4(CHANNEL_COUNT as u8);
        config.slot_enable = 0xFFFF; // All slots
        config.data_size = sai::DataSize::Data32;
        config.frame_length = (CHANNEL_COUNT * SAMPLE_WIDTH_BIT) as u8;
        config.frame_sync_active_level_length = sai::word::U7(SAMPLE_WIDTH_BIT as u8);
        config.bit_order = sai::BitOrder::MsbFirst;
        config.mute_value = sai::MuteValue::LastValue;

        match sample_rate_hz {
            SAMPLE_RATE_HZ => config.master_clock_divider = sai::MasterClockDivider::Div2,
            _ => panic!("Unsupported SAI sample rate."),
        }

        sai::Sai::new_asynchronous(
            sai_rpi,
            &mut resources.sck_b,
            &mut resources.sd_b,
            &mut resources.fs_b,
            &mut resources.dma_b,
            sai_rpi_read_buffer,
            config,
        )
    };

    (sai_amp_driver, sai_rpi_driver)
}

fn process(
    samples: &[u32],
    processed_samples: &mut Vec<u32, { 2 * MAX_SAMPLE_COUNT }>,
    filters: &mut [AudioFilter; OUTPUT_CHANNEL_COUNT],
    gain_left: f32,
    gain_right: f32,
) {
    for index in 0..samples.len() {
        let sample = audio_filter::sample_to_f32(samples[index]);

        if index % 2 == 0 {
            // Left channel
            processed_samples
                .push(audio_filter::sample_to_u32(filters[0].run(sample) * gain_left))
                .unwrap();
            processed_samples
                .push(audio_filter::sample_to_u32(filters[1].run(sample) * gain_left))
                .unwrap();
        } else {
            // Right channel
            processed_samples
                .push(audio_filter::sample_to_u32(filters[2].run(sample) * gain_right))
                .unwrap();
            processed_samples
                .push(audio_filter::sample_to_u32(filters[3].run(sample) * gain_right))
                .unwrap();
        }
    }
}

#[embassy_executor::task]
pub async fn audio_routing_task(
    mut filters: [AudioFilter<'static>; OUTPUT_CHANNEL_COUNT],
    mut sai4_resources: Sai4Resources,
    audio_receiver: channel::Receiver<'static, NoopRawMutex, SampleBlock, SAMPLE_BLOCK_COUNT>,
    mut led_status: Output<'static>,
    mut led_usb: Output<'static>,
    mut led_rpi: Output<'static>,
    mut led_spdif: Output<'static>,
) {
    info!("Amplifier SAI write buffer size: {} samples", SAI_AMP_SAMPLE_COUNT);
    info!("Raspberry Pi SAI read buffer size: {} samples", SAI_RPI_SAMPLE_COUNT);

    let sai_amp_write_buffer: &mut [u32] = unsafe {
        SAI_AMP_WRITE_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SAI_AMP_WRITE_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let sai_rpi_read_buffer: &mut [u32] = unsafe {
        SAI_RPI_READ_BUFFER.initialize_all_copied(0);
        let (ptr, len) = SAI_RPI_READ_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let (mut sai_amp, mut sai_rpi) = new_sai_amp_rpi(
        &mut sai4_resources,
        sai_amp_write_buffer,
        sai_rpi_read_buffer,
        SAMPLE_RATE_HZ,
        AudioSource::None,
    );

    let mut source = AudioSource::None;
    let mut last_source = source;

    let mut usb_gain = (0.0, 0.0);
    let mut pot_gain = (0.0, 0.0);

    sai_rpi.start().unwrap();

    loop {
        let sample_block = match select(audio_receiver.receive(), sai_amp.wait_write_error()).await {
            Either::First(x) => {
                // Determine the next active source, if none was previously selected.
                if matches!(source, AudioSource::None) {
                    source = match &x {
                        SampleBlock::Spdif(_) => AudioSource::Spdif,
                        SampleBlock::Usb(_) => AudioSource::Usb,
                    };
                }
                Some(x)
            }
            Either::Second(_) => {
                source = AudioSource::None;
                None
            }
        };

        // Reset SAI if the source changes. The source is reset to `None` in case of errors,
        // thus also resetting the SAI.
        if last_source != source {
            info!("New source: {}", source);
            last_source = source;

            for led in [&mut led_usb, &mut led_rpi, &mut led_spdif] {
                led.set_low();
            }

            match source {
                AudioSource::Spdif => led_spdif.set_high(),
                AudioSource::Usb => led_usb.set_high(),
                AudioSource::RaspberryPi => led_rpi.set_high(),
                _ => (),
            }

            drop(sai_amp);
            drop(sai_rpi);

            (sai_amp, sai_rpi) = new_sai_amp_rpi(
                &mut sai4_resources,
                sai_amp_write_buffer,
                sai_rpi_read_buffer,
                SAMPLE_RATE_HZ,
                source,
            );

            SAI_ACTIVE_SIGNAL.signal(source);
            AMP_SETUP_SIGNAL.wait().await;

            audio_receiver.clear();
            sai_rpi.start().unwrap();
        }

        let Some(sample_block) = sample_block else { continue };

        let mut processed_samples: Vec<u32, { 2 * MAX_SAMPLE_COUNT }> = Vec::new();
        match (sample_block, source) {
            (SampleBlock::Spdif(samples), AudioSource::Spdif) => {
                if let Some(gain) = POT_GAIN_SIGNAL.try_take() {
                    pot_gain = (gain, gain);
                }

                process(
                    samples.as_slice(),
                    &mut processed_samples,
                    &mut filters,
                    pot_gain.0,
                    pot_gain.1,
                );

                // 16 bit playback in 32 bit DMA mode.
                for sample in processed_samples.iter_mut() {
                    *sample >>= 16;
                }
            }
            (SampleBlock::Usb(samples), AudioSource::Usb) => {
                if let Some(gain) = USB_GAIN_SIGNAL.try_take() {
                    usb_gain = gain;
                }

                process(
                    samples.as_slice(),
                    &mut processed_samples,
                    &mut filters,
                    usb_gain.0,
                    usb_gain.1,
                );
            }
            _ => {
                debug!("Drop sample block with source {}", source);
                continue;
            }
        };

        sai_amp.write(&processed_samples).await.unwrap();
    }
}
