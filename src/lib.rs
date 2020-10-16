#![no_std]

use cortex_m::asm;
use embedded_hal::{blocking::spi::Transfer, digital::v2::OutputPin};

enum Command {
    Reset = 0b0000_0110,
    Start = 0b0000_1000,
    Pwdn = 0b0000_0010,
    Rdata = 0b0001_0000,
    Rreg = 0b0010_0000,
    Wreg = 0b0100_0000,
}

/// A macro to create enums for bitfield fields easily.
macro_rules! benum {
    {
        $(#[$outer:meta])*
        pub enum $name:ident {
            $(
                $(#[$inner:meta])*
                $variant:ident => $value:literal
            ),+ $(,)?
        }
    } => {
        #[derive(Debug)]
        #[derive(Clone)]
        $(#[$outer])*
        pub enum $name {
            $(
                $(#[$inner])*
                $variant = $value
            ),+
        }

        impl From<u8> for $name {
            fn from(value: u8) -> Self {
                match value {
                    $($value => $name::$variant,)+
                    _ => unreachable!(),
                }
            }
        }

        impl From<$name> for u8 {
            fn from(value: $name) -> Self {
                value as u8
            }
        }
    };
}

benum!(
    /// Configures the mux of the ADS1220.
    pub enum Mux {
        /// AINP = AIN0, AINN = AIN1 (default)
        Ain0Ain1 => 0b0000,
        /// AINP = AIN0, AINN = AIN2
        Ain0Ain2 => 0b0001,
        /// AINP = AIN0, AINN = AIN3
        Ain0Ain3 => 0b0010,
        /// AINP = AIN1, AINN = AIN2
        Ain1Ain2 => 0b0011,
        /// AINP = AIN1, AINN = AIN3
        Ain1Ain3 => 0b0100,
        /// AINP = AIN2, AINN = AIN3
        Ain2Ain3 => 0b0101,
        /// AINP = AIN1, AINN = AIN0
        Ain1Ain0 => 0b0110,
        /// AINP = AIN3, AINN = AIN2
        Ain3Ain2 => 0b0111,
        /// AINP = AIN0, AINN = AVSS
        Ain0Avss => 0b1000,
        /// AINP = AIN1, AINN = AVSS
        Ain1Avss => 0b1001,
        /// AINP = AIN2, AINN = AVSS
        Ain2Avss => 0b1010,
        /// AINP = AIN3, AINN = AVSS
        Ain3Avss => 0b1011,
        /// (V(REFPx) – V(REFNx)) / 4 monitor (PGA bypassed)
        Vref4 => 0b1100,
        /// (AVDD – AVSS)/ 4 monitor (PGA bypassed)
        AvddAvss4 => 0b1101,
        /// AINP and AINN shorted to (AVDD+ AVSS) / 2
        AvddAvss2 => 0b1110,
        /// Do not use.
        _Reserved => 0b1111,
    }
);

benum!(
    /// Configures the gain of the ADS1220.
    pub enum Gain {
        /// Gain = 1 (default)
        X1 => 0b000,
        /// Gain = 2
        X2 => 0b001,
        /// Gain = 4
        X4 => 0b010,
        /// Gain = 8
        X8 => 0b011,
        /// Gain = 16
        X16 => 0b100,
        /// Gain = 32
        X32 => 0b101,
        /// Gain = 64
        X64 => 0b110,
        /// Gain = 128
        X128 => 0b111,
    }
);

bitfield::bitfield! {
    /// Configuration Register 0 of the ADS1220.
    pub struct Config0(u8);
    impl Debug;
    /// Configures the mux of the ADS1220.
    pub from into Mux, mux, set_mux: 7, 4;
    /// Configures the gain of the ADS1220.
    pub from into Gain, gain, set_gain: 3, 1;
    /// If set to `true` the PGA of the ADS1220 is bypassed.
    pub pga_bypass, set_pga_bypass: 0;
}

benum!(
    /// Configures the data rate of the ADS1220.
    ///
    /// The DR enum varian is different depending on what the operating mode is.
    pub enum DataRate {
        /// Normal mode 20 SPS, DC mode 5 SPS, Turbo mode 40 SPS.
        Sps20_5_40 => 0b000,
        /// Normal mode 45 SPS, DC mode 11.25 SPS, Turbo mode 90 SPS.
        Sps45_11_90 => 0b001,
        /// Normal mode 90 SPS, DC mode 22.5 SPS, Turbo mode 180 SPS.
        Sps90_22_180 => 0b010,
        /// Normal mode 175 SPS, DC mode 44 SPS, Turbo mode 350 SPS.
        Sps175_44_350 => 0b011,
        /// Normal mode 330 SPS, DC mode 82.5 SPS, Turbo mode 660 SPS.
        Sps330_82_660 => 0b100,
        /// Normal mode 600 SPS, DC mode 150 SPS, Turbo mode 1200 SPS.
        Sps600_150_1200 => 0b101,
        /// Normal mode 1000 SPS, DC mode 250 SPS, Turbo mode 2000 SPS.
        Sps1000_250_2000 => 0b110,
        /// Do not use.
        _Reserved => 0b111,
    }
);

benum!(
    /// Configures the data rate of the ADS1220.
    pub enum OperatingMode {
        ///  Normal mode (256-kHz modulator clock, default)
        Normal => 0b00,
        /// Duty-cycle mode (internal duty cycle of 1:4)
        DutyCycle => 0b01,
        /// Turbo mode (512-kHz modulator clock)
        Turbo => 0b10,
        /// Do not use.
        _Reserved => 0b11,
    }
);

benum!(
    /// Configures the conversion mode of the ADS1220.
    pub enum ConversionMode {
        /// Does a single conversion and stops after it is done.
        /// A new conversion has to be started manually afterwards.
        SingleShot => 0b00,
        /// Does a conversion and starts a new one after it is done.
        Continuous => 0b01,
    }
);

bitfield::bitfield! {
    /// Configuration Register 1 of the ADS1220.
    pub struct Config1(u8);
    impl Debug;
    /// Configures the data rate of the AD1220.
    ///
    /// The data rate variants have different meanings depending on the operationg mode.
    pub from into DataRate, dr, set_dr: 7,5;
    /// COnfigures the operating mode of the AD1220.
    pub from into OperatingMode, operating_mode, set_operating_mode: 4,3;
    /// Configures the conversion mode of the AD1220.
    pub from into ConversionMode, conversion_mode, set_conversion_mode: 2,2;
    /// This bit enables the internal temperature sensor and puts the device in temperature sensor mode.
    /// The settings of configuration register 0 have no effect and the device uses the internal reference for measurement when temperature sensor mode is enabled.
    pub temperature_sensor_enabled, set_enable_temperature_sensor_enabled: 1;
    /// This bit controls the 10-µA, burn-out current sources.
    /// The burn-out current sources can be used to detect sensor faults such as wire breaks and shorted sensors.
    pub burnot_current_sources_enabled, set_burnot_current_sources_enabled: 0;
}

benum!(
    /// Configures the data rate of the ADS1220.
    pub enum VoltageReference {
        /// Internal 2.048-V reference selected (default).
        Internal => 0b00,
        /// External reference selected using dedicated REFP0 and REFN0 inputs.
        ExternalRef0 => 0b01,
        /// External reference selected using AIN0/REFP1 and AIN3/REFN1 inputs.
        ExternalRef1 => 0b10,
        /// Analog supply (AVDD – AVSS) used as reference.
        Analog => 0b11,
    }
);

benum!(
    /// Configures the 50/60Hz rejection FIR filter of the ADS1220.
    pub enum FirConfiguration {
        /// No 50-Hz or 60-Hz rejection (default)
        Off => 0b00,
        /// Simultaneous 50-Hz and 60-Hz rejection
        Both => 0b01,
        /// 50-Hz rejection only
        Hz50 => 0b10,
        /// 50-Hz rejection only
        Hz60 => 0b11,
    }
);

benum!(
    /// Configures the low side power switch of the ADS1220.
    pub enum PowerSwitchConfig {
        /// Switch is always open.
        Open => 0b00,
        /// Switch automatically closes when the START/SYNC command is sent and opens when the POWERDOWN command is issued.
        Auto => 0b01,
    }
);

benum!(
    /// Configures the current for both IDAC1 and IDAC2 excitation current sources of the ADS1220.
    pub enum CurrentSetting {
        /// Off (default)
        Off => 0b000,
        /// 10 uA
        A10 => 0b001,
        /// 50 uA
        A50 => 0b010,
        /// 100 uA
        A100 => 0b011,
        /// 250 uA
        A250 => 0b100,
        /// 500 uA
        A500 => 0b101,
        /// 10000 uA
        A1000 => 0b110,
        /// 1500 uA
        A1500 => 0b111,
    }
);

bitfield::bitfield! {
    /// Configuration Register 2 of the ADS1220.
    pub struct Config2(u8);
    impl Debug;
    pub from into VoltageReference, vref, set_vref: 7,6;
    pub from into FirConfiguration, fir, set_fir: 5,4;
    pub from into PowerSwitchConfig, psw, set_psw: 3,3;
    pub from into CurrentSetting, idac, set_idac: 2,0;
}

benum!(
    /// Configures the channel for IDACn, where n is 1 or 2, of the ADS1220.
    pub enum Imux {
        /// IDACn disabled (default)
        Disabled => 0b000,
        /// IDACn connected to AIN0/REFP1
        Ain0 => 0b001,
        /// IDACn connected to AIN1
        Ain1 => 0b010,
        /// IDACn connected to AIN2
        Ain2 => 0b011,
        /// IDACn connected to AIN3/REFN1
        Ain3 => 0b100,
        /// IDACn connected to REFP0
        RefP0 => 0b101,
        /// IDACn connected to REFN1
        RefN1 => 0b110,
        _Reserved => 0b111,
    }
);

benum!(
    /// Configures the channel for IDACn, where n is 1 or 2, of the ADS1220.
    pub enum Drdy {
        ///  Only the dedicated DRDY pin is used to indicate when data are ready (default).
        Only => 0b000,
        /// Data ready is indicated simultaneously on DOUT/DRDY and DRDY.
        Both => 0b001,
    }
);

bitfield::bitfield! {
    pub struct Config3(u8);
    impl Debug;
    pub from into Imux, i1mux, set_i1mux: 7,5;
    pub from into Imux, i2mux, set_i2mux: 4,2;
    pub from into Drdy, drdym, set_drdym: 1,1;
}

/// A trait to mark a register config struct of the ADS1220 with the corresponding address.
pub trait Register {
    const ADDRESS: u8;
}

/// Implements the register trait for a register.
macro_rules! register {
    ( $reg:tt, $addr:expr ) => {
        impl Register for $reg {
            const ADDRESS: u8 = $addr;
        }

        impl From<u8> for $reg {
            fn from(value: u8) -> $reg {
                $reg(value)
            }
        }

        impl From<$reg> for u8 {
            fn from(value: $reg) -> u8 {
                value.0
            }
        }
    };
}

register!(Config0, 0);
register!(Config1, 1);
register!(Config2, 2);
register!(Config3, 3);

/// Implements a shortcut command for the ADS1220.
macro_rules! fast_command {
    ( $name:tt, $cmd:expr ) => {
        #[allow(dead_code)]
        pub fn $name(&mut self) -> Result<(), Ads1220Error<<SPI as Transfer<u8>>::Error>> {
            self.transfer($cmd as u8, &mut [])?;
            Ok(())
        }
    };
}

/// Describes an ADS1220 error. Currently this just wraps an SPI errors.
#[derive(Debug)]
pub enum Ads1220Error<E> {
    Spi(E),
}

impl<E> From<E> for Ads1220Error<E> {
    fn from(error: E) -> Self {
        Ads1220Error::Spi(error)
    }
}

/// An abstraction over the ADC1220 4-Channel 2-kSPS, 24bit ADC.
pub struct Ads1220<SPI, NCS>
where
    SPI: Transfer<u8>,
    <SPI as Transfer<u8>>::Error: core::fmt::Debug,
    NCS: OutputPin,
{
    spi: SPI,
    ncs: NCS,
}

impl<SPI, NCS> Ads1220<SPI, NCS>
where
    SPI: Transfer<u8>,
    <SPI as Transfer<u8>>::Error: core::fmt::Debug,
    NCS: OutputPin,
{
    /// Creates a new handle to control an ADS1220.
    /// Expects a handle to an SPI bus and a chip select pin.
    /// This drives the chip select pin high on creation.
    pub fn new(spi: SPI, mut ncs: NCS) -> Self {
        ncs.set_high().ok();
        Self { spi, ncs }
    }

    /// Transfers a command and a respective payload.
    /// This also manipulates the CS pin correctly, no matter the outcome of the transfer.
    fn transfer(
        &mut self,
        command: u8,
        result: &mut [u8],
    ) -> Result<(), Ads1220Error<<SPI as Transfer<u8>>::Error>> {
        self.select();

        let command = &mut [command];
        let r = self
            .spi
            .transfer(command)
            .and_then(|_| self.spi.transfer(result));

        self.deselect();
        r.map(|_| ()).map_err(From::from)
    }

    /// Sets the CS pin low.
    fn select(&mut self) {
        self.ncs.set_low().ok();
    }

    /// Sets the CS pin high.
    fn deselect(&mut self) {
        self.ncs.set_high().ok();
    }

    fast_command!(start, Command::Start);
    fast_command!(reset, Command::Reset);
    fast_command!(powerdown, Command::Pwdn);

    /// Reads the current aquisition from the ADC as a u32.
    pub fn data_u32(&mut self) -> Result<u32, Ads1220Error<<SPI as Transfer<u8>>::Error>> {
        let mut buf: [u8; 4] = [0; 4];
        self.transfer(Command::Rdata as u8, &mut buf[1..4])?;
        Ok(u32::from_be_bytes(buf))
    }

    /// Reads the current aquisition from the ADC as an i32.
    pub fn data_i32(&mut self) -> Result<i32, Ads1220Error<<SPI as Transfer<u8>>::Error>> {
        let data = self.data_u32()? as i32;
        // Convert signed 24bit to signed 32bit
        let bits = 24;
        let m: i32 = 1 << (bits - 1);
        Ok((data ^ m) - m)
    }

    /// Reads the current aquisition from the ADC as an f32.
    pub fn data_f32(&mut self) -> Result<f32, Ads1220Error<<SPI as Transfer<u8>>::Error>> {
        let d = self.data_i32()? as f32;
        Ok((d / ((2_u32.pow(23)) as f32)))
    }

    /// Writes a config register of the ADS1220.
    pub fn set_config<C: Register + Into<u8>>(
        &mut self,
        config0: C,
    ) -> Result<(), Ads1220Error<<SPI as Transfer<u8>>::Error>> {
        self.transfer(
            (Command::Wreg as u8) | ((C::ADDRESS) << 2) | 0,
            &mut [config0.into()],
        )?;
        Ok(())
    }

    /// Reads a config register of the ADS1220.
    pub fn get_config<C: Register + From<u8>>(
        &mut self,
    ) -> Result<C, Ads1220Error<<SPI as Transfer<u8>>::Error>> {
        let mut buf = [0];
        self.transfer(
            (Command::Rreg as u8) | ((C::ADDRESS as u8) << 2) | 0,
            &mut buf,
        )?;
        Ok(C::from(buf[0]))
    }
}
