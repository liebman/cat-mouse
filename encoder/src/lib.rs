use std::cmp::min;
use std::sync::atomic::AtomicI64;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::pcnt::*;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_sys::EspError;

#[derive(Debug, Clone)]
pub struct Encoder<'d> {
    pcnt: Arc<PcntDriver<'d>>,
    position: Arc<AtomicI64>,
    ticks_per_degree: f64,
    invert: bool,
}

impl<'d> Encoder<'d> {
    pub fn new<'a, PCNT: Pcnt>(
        pcnt: impl Peripheral<P = PCNT> + 'd,
        mut a_pin: impl Peripheral<P = impl InputPin>,
        mut b_pin: impl Peripheral<P = impl InputPin>,
        ticks_per_revolution: u32,
        invert: bool,
    ) -> Result<Self, EspError> {
        let mut pcnt = PcntDriver::new(pcnt)?;
        const POS_LIMIT: i16 = 100;
        const NEG_LIMIT: i16 = -100;
        pcnt.channel_config(
            PcntChannel::Channel0,
            Some(&mut a_pin),
            Some(&mut b_pin),
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: POS_LIMIT,
                counter_l_lim: NEG_LIMIT,
            },
        )?;
        pcnt.channel_config(
            PcntChannel::Channel1,
            Some(&mut b_pin),
            Some(&mut a_pin),
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Increment,
                neg_mode: PcntCountMode::Decrement,
                counter_h_lim: POS_LIMIT,
                counter_l_lim: NEG_LIMIT,
            },
        )?;
        pcnt.set_filter_value(min(10 * 80, 1023))?;
        pcnt.filter_enable()?;
        pcnt.event_enable(PcntEvent::HighLimit)?;
        pcnt.event_enable(PcntEvent::LowLimit)?;

        let enc = Encoder {
            pcnt: Arc::new(pcnt),
            position: Arc::new(AtomicI64::new(0)),
            ticks_per_degree: ticks_per_revolution as f64 / 360.0,
            invert,
        };

        unsafe {
            let position = enc.position.clone();
            enc.pcnt.subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    position.fetch_add(POS_LIMIT as i64, Ordering::Relaxed);
                }
                if status.contains(PcntEvent::LowLimit) {
                    position.fetch_add(NEG_LIMIT as i64, Ordering::Relaxed);
                }
            })?;
        }
        enc.pcnt.counter_pause()?;
        enc.pcnt.counter_clear()?;
        enc.pcnt.counter_resume()?;
        Ok(enc)
    }

    pub fn get_raw_position(&self) -> i64 {
        let mut raw =
            self.position.load(Ordering::Relaxed) + self.pcnt.get_counter_value().unwrap() as i64;
        if self.invert {
            raw = -raw;
        }
        raw
    }

    pub fn get_position(&self) -> i64 {
        // using double floating point this should be valid for a very large position - should be good enough for
        // a small rover. (precision should not cause us any issues)
        (self.get_raw_position() as f64 / self.ticks_per_degree) as i64
    }

    pub fn set_position(&self, position: i64) {
        let raw = match self.invert {
            true => -position,
            false => position,
        };
        // small race condition here but clearing the counter first means that we have *_LIMIT pcnt counts before we'd
        // update the position
        self.pcnt.counter_clear().unwrap();
        self.position.store(
            (raw as f64 * self.ticks_per_degree) as i64,
            Ordering::Relaxed,
        );
    }
}
