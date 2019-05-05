//! Timers

// TODO: on the f303 at least, only TIM2, TIM3, TIM4 can support 32 bits.
// The other times are 16 bit.

use crate::hal::timer::{CountDown, Periodic};
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303"
))]
use crate::stm32::TIM1;
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
use crate::stm32::TIM4;
#[cfg(any(
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
use crate::stm32::{TIM12, TIM13, TIM14, TIM18, TIM5};
use crate::stm32::{TIM2, TIM6};
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
use crate::stm32::{TIM3, TIM7};

use cast::{u16, u32};
use nb;
use void::Void;

use crate::rcc::{Clocks, APB1, APB2};
use crate::time::Hertz;

/// Hardware timers
pub struct Timer<TIM> {
    clocks: Clocks,
    tim: TIM,
    timeout: Hertz,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $APB:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl Periodic for Timer<$TIM> {}

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                #[allow(unused_unsafe)]
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.pause();
                    // restart counter
                    self.tim.cnt.reset();

                    self.set_freq(timeout);

                    // start counter
                    self.resume()
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_uif_bit();
                        Ok(())
                    }
                }
            }

            impl Timer<$TIM> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIM, timeout: T, clocks: Clocks, apb: &mut $APB) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer {
                        clocks,
                        tim,
                        timeout: Hertz(0),
                    };
                    timer.start(timeout);

                    timer
                }

                // NOTE(allow) `w.psc().bits()` is safe for TIM{6,7} but not for TIM{2,3,4} due to
                // some SVD omission
                #[allow(unused_unsafe)]
                pub fn set_freq<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.timeout = timeout.into();

                    let frequency = self.timeout.0;
                    let ticks = self.clocks.pclk1().0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                        / frequency;

                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
                }

                /// Clear uif bit
                pub fn clear_uif_bit(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }

                /// Pauses the TIM peripheral
                pub fn pause(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                }

                /// Resume (unpause) the TIM peripheral
                pub fn resume(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                /// Reset the counter of the TIM peripheral
                pub fn reset_counter(&mut self) {
                    self.tim.cnt.reset();
                }

                /// Read the counter of the TIM peripheral
                pub fn counter(&self) -> u32 {
                    self.tim.cnt.read().bits()
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn free(mut self) -> $TIM {
                    // pause counter
                    self.pause();
                    self.tim
                }
            }
        )+
    }
}

#[cfg(any(feature = "stm32f301", feature = "stm32f318"))]
hal! {
    TIM2: (tim2, APB1, tim2en, tim2rst),
    TIM6: (tim6, APB1, tim6en, tim6rst),
}

#[cfg(any(feature = "stm32f302", feature = "stm32f303"))]
hal! {
    TIM1: (tim1, APB2, tim1en, tim1rst),
    TIM2: (tim2, APB1, tim2en, tim2rst),
    TIM3: (tim3, APB1, tim3en, tim3rst),
    TIM4: (tim4, APB1, tim4en, tim4rst),
    TIM6: (tim6, APB1, tim6en, tim6rst),
    TIM7: (tim7, APB1, tim7en, tim7rst),
}

#[cfg(feature = "stm32f334")]
hal! {
    TIM2: (tim2, APB1, tim2en, tim2rst),
    TIM3: (tim3, APB1, tim3en, tim3rst),
    TIM6: (tim6, APB1, tim6en, tim6rst),
    TIM7: (tim7, APB1, tim7en, tim7rst),
}

#[cfg(any(
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
hal! {
    TIM2: (tim2, APB1, tim2en, tim2rst),
    TIM3: (tim3, APB1, tim3en, tim3rst),
    TIM4: (tim4, APB1, tim4en, tim4rst),
    TIM5: (tim5, APB1, tim5en, tim5rst),
    TIM6: (tim6, APB1, tim6en, tim6rst),
    TIM7: (tim7, APB1, tim7en, tim7rst),
    TIM12: (tim12, APB1, tim12en, tim12rst),
    TIM13: (tim13, APB1, tim13en, tim13rst),
    TIM14: (tim14, APB1, tim14en, tim14rst),
    TIM18: (tim18, APB1, tim18en, tim18rst),
}
