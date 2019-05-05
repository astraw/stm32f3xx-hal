//! PWM

use cast::{u16, u32};
use embedded_hal::timer::Periodic;
use crate::stm32::{TIM1, TIM2, TIM3};

use crate::rcc::{APB1, APB2, Clocks};
use crate::time::Hertz;

use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA5, PA6,
    PA7};
use crate::gpio::gpiob::{PB0, PB1, PB10, PB11};

use crate::gpio::{AF1, AF2};

/// Hardware timers used as PWM
pub struct PWM<TIM, PINS> {
    clocks: Clocks,
    tim: TIM,
    period: Hertz,
    pins: PINS,
}

/// Interrupt events
pub enum Event {
    /// PWM period ended
    RollOver,
}

// FIXME these should be "closed" traits
/// CH1 pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait Ch1Pin<TIM> {}
/// CH2 pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait Ch2Pin<TIM> {}
/// CH3 pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait Ch3Pin<TIM> {}
/// CH4 pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait Ch4Pin<TIM> {}

unsafe impl Ch1Pin<TIM2> for PA0<AF1> {}
unsafe impl Ch1Pin<TIM2> for PA5<AF1> {}
// unsafe impl Ch1Pin<TIM2> for PA15<AF1> {}
unsafe impl Ch2Pin<TIM2> for PA1<AF1> {}
// unsafe impl Ch2Pin<TIM2> for PB3<AF1> {}
unsafe impl Ch3Pin<TIM2> for PA2<AF1> {}
unsafe impl Ch3Pin<TIM2> for PB10<AF1> {}
unsafe impl Ch4Pin<TIM2> for PA3<AF1> {}
unsafe impl Ch4Pin<TIM2> for PB11<AF1> {}

unsafe impl Ch1Pin<TIM3> for PA6<AF2> {}
unsafe impl Ch2Pin<TIM3> for PA7<AF2> {}
unsafe impl Ch3Pin<TIM3> for PB0<AF2> {}
unsafe impl Ch4Pin<TIM3> for PB1<AF2> {}

/// PWM channels on the timer/counter
#[derive(Copy,Clone,Debug,PartialEq)]
pub enum Channel {
    /// Channel 1
    Ch1,
    /// Channel 2
    Ch2,
    /// Channel 3
    Ch3,
    /// Channel 4
    Ch4,
}

#[cfg(feature = "unproven")]
use crate::hal::Pwm;

macro_rules! hal {
    ($($TIM:ident: ($pwm:ident, $APB:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl<P1, P2, P3, P4> Periodic for PWM<$TIM, (P1, P2, P3, P4)> {}

            #[cfg(feature = "unproven")]
            impl<P1, P2, P3, P4> Pwm for PWM<$TIM, (P1, P2, P3, P4)> {
                type Channel = Channel;
                type Duty = u32;
                type Time = Hertz;

                /// Returns the current duty cycle
                fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
                    match channel {
                        Channel::Ch1 => self.tim.ccr1.read().bits(),
                        Channel::Ch2 => self.tim.ccr2.read().bits(),
                        Channel::Ch3 => self.tim.ccr3.read().bits(),
                        Channel::Ch4 => self.tim.ccr4.read().bits(),
                    }
                }

                /// Disables a PWM `channel`
                fn disable(&mut self, channel: Self::Channel) {
                    match channel {
                        Channel::Ch1 => {
                            self.tim.ccmr1_output.modify(|_, w| w.oc1pe().clear_bit() );
                        },
                        Channel::Ch2 => {
                            self.tim.ccmr1_output.modify(|_, w| w.oc2pe().clear_bit() );
                        },
                        Channel::Ch3 => {
                            self.tim.ccmr2_output.modify(|_, w| w.oc3pe().clear_bit() );
                        },
                        Channel::Ch4 => {
                            self.tim.ccmr2_output.modify(|_, w| w.oc4pe().clear_bit() );
                        },
                    }
                }

                /// Enables a PWM `channel`
                fn enable(&mut self, channel: Self::Channel) {
                    const PWM_MODE_1: u8 = 0b0110;
                    match channel {
                        Channel::Ch1 => {
                            self.tim.ccmr1_output.modify(|_, w| unsafe{ w.oc1m().bits(PWM_MODE_1) });
                            self.tim.ccmr1_output.modify(|_, w| w.oc1pe().set_bit() );
                        },
                        Channel::Ch2 => {
                            self.tim.ccmr1_output.modify(|_, w| unsafe{ w.oc2m().bits(PWM_MODE_1) });
                            self.tim.ccmr1_output.modify(|_, w| w.oc2pe().set_bit() );
                        },
                        Channel::Ch3 => {
                            self.tim.ccmr2_output.modify(|_, w| unsafe{ w.oc3m().bits(PWM_MODE_1) });
                            self.tim.ccmr2_output.modify(|_, w| w.oc3pe().set_bit() );
                        },
                        Channel::Ch4 => {
                            self.tim.ccmr2_output.modify(|_, w| unsafe{ w.oc4m().bits(PWM_MODE_1) });
                            self.tim.ccmr2_output.modify(|_, w| w.oc4pe().set_bit() );
                        },
                    }
                }

                /// Returns the current PWM period
                fn get_period(&self) -> Self::Time {
                    self.period
                }

                /// Returns the maximum duty cycle value
                fn get_max_duty(&self) -> Self::Duty {
                    // TODO check this.
                    self.tim.arr.read().bits()
                }

                /// Sets a new duty cycle
                fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
                    self.set_channel(channel, duty)
                }

                /// Sets a new PWM period
                fn set_period<P>(&mut self, period: P)
                    where
                        P: Into<Self::Time>
                {
                    self.period = period.into();
                    let (arr, psc) = self.compute_timing_info(&self.period);
                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
                }

            }

            impl<P1, P2, P3, P4> PWM<$TIM, (P1, P2, P3, P4)> {

                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a PWM device
                pub fn $pwm<T>(tim: $TIM, pins: (P1, P2, P3, P4), period: T, clocks: Clocks, apb: &mut $APB) -> Self
                where
                    T: Into<Hertz>,
                    P1: Ch1Pin<$TIM>,
                    P2: Ch2Pin<$TIM>,
                    P3: Ch3Pin<$TIM>,
                    P4: Ch4Pin<$TIM>,
                {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut pwm = PWM {
                        clocks,
                        tim,
                        period: Hertz(0),
                        pins,
                    };
                    pwm.start(period);

                    pwm
                }

                // NOTE(allow) `w.psc().bits()` is safe for TIM{6,7} but not for TIM{2,3,4} due to
                // some SVD omission
                #[allow(unused_unsafe)]
                fn start<T>(&mut self, period: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // restart counter
                    self.tim.cnt.reset();

                    self.enable(Channel::Ch1);
                    self.enable(Channel::Ch2);
                    self.enable(Channel::Ch3);
                    self.enable(Channel::Ch3);

                    // Capture/compare enable on all channels.
                    self.tim.ccer.modify(|_, w| w
                        .cc1e().set_bit()
                        .cc2e().set_bit()
                        .cc3e().set_bit()
                        .cc4e().set_bit()
                        );

                    self.set_period(period);

                    {
                        let _x = &self.pins; // dummy usage
                    }

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn compute_timing_info(&self, period: &Hertz) -> (u16, u16) {
                    let frequency = period.0;
                    let ticks = self.clocks.pclk1().0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                        / frequency;

                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                    let arr = u16(ticks / u32(psc + 1)).unwrap();
                    (arr, psc)
                }

                /// Set duration for channel
                fn set_channel(&mut self, channel: Channel, pwm_period: u32) {
                    match channel {
                        Channel::Ch1 => self.tim.ccr1.write(|w| unsafe {w.bits(pwm_period)}),
                        Channel::Ch2 => self.tim.ccr2.write(|w| unsafe {w.bits(pwm_period)}),
                        Channel::Ch3 => self.tim.ccr3.write(|w| unsafe {w.bits(pwm_period)}),
                        Channel::Ch4 => self.tim.ccr4.write(|w| unsafe {w.bits(pwm_period)}),
                    };
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::RollOver => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::RollOver => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn free(self) -> $TIM {
                    // // pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    self.tim
                }
            }
        )+
    }
}

hal! {
    TIM1: (pwm1, APB2, tim1en, tim1rst),
    TIM2: (pwm2, APB1, tim2en, tim2rst),
    TIM3: (pwm3, APB1, tim3en, tim3rst),
}
