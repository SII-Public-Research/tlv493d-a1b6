use embedded_hal::blocking::delay::*;

use core::marker::PhantomData;
use log::{debug};
use bitflags::bitflags;
use embedded_hal::blocking::i2c;

#[cfg(feature = "std")]
extern crate std;






//////////////////////////////////////////////////////////////////////////////////////
//              CRATE "sensor_tlv493d" DE RYANKURTE MODIFIEE POUR RPPAL
/////////////////////////////////////////////////////////////////////////////////////

pub struct Tlv493d<I2c, E> {
    pub i2c: I2c,
    pub addr: u8,
    pub initial: [u8; 10],
    pub last_frm: u8,
    _e: PhantomData<E>,
}

// Base address for Tlv493d, bottom bit set during power up
// based on value of SDA.
pub const ADDRESS_BASE_1: u8 = 0b1011110;
pub const ADDRESS_BASE_2: u8 = 0b0011111;

// Read registers for the Tlv493d
pub enum ReadRegisters {
    Rx      = 0x00, // X direction flux (Bx[11..4])
    By      = 0x01, // Y direction flux (By[11..4])
    Bz      = 0x02, // X direction flux (Bz[11..4])
    Temp    = 0x03, // Temperature high bits, frame counter, channel, (Temp[11..8] | FRM[1..0] | CH[1..0])
    Bx2     = 0x04, // Lower X and Y flux (Bx[3..0] | Bx[3..0])
    Bz2     = 0x05, // Flags + Lower Z flux (Reserved | T | FF | PD | Bz[3..0])
    Temp2   = 0x06, // Temperature low bits (Temp[7..0])
    
    FactSet1 = 0x07,
    FactSet2 = 0x08,
    FactSet3 = 0x09,
}

// Write registers for the Tlv493d
pub enum WriteRegisters {
    Res     = 0x00, // Reserved
    Mode1   = 0x01, // Mode 1 register (P | IICAddr[1..0] | Reserved[1..0] | INT | FAST | LOW)
    Res2    = 0x02, // Reserved
    Mode2   = 0x03, // Mode 2 register (T | LP | PT | Reserved[4..0])
}

// TLV493D Measurement values
#[derive(Debug, PartialEq, Clone)]
pub struct Values {
    x: f32,     // X axis magnetic flux (mT)
    y: f32,     // Y axis magnetic flux (mT)
    z: f32,     // Z axis magnetic flux (mT)
    temp: f32,  // Device temperature (C)
}

// Device operating mode
// Note that in most cases the mode is a combination of mode and IRQ flags
pub enum Mode {
    Disabled,       // Reading disabled
    Master,         // Master initiated mode (reading occurs after readout)
    Fast,           // Fast mode (3.3kHz)
    LowPower,       // Low power mode (100Hz)
    UltraLowPower,  // Ultra low power mode (10Hz)
}

bitflags! {
    /// Device Mode1 register
    pub struct Mode1: u8 {
        const PARITY     = 0b1000_0000;     // Parity of configuration map, must be calculated prior to write command
        const I2C_ADDR_1 = 0b0100_0000;     // Set I2C address top bit in bus configuration
        const I2C_ADDR_0 = 0b0010_0000;     // Set I2C address bottom bit in bus configuration
        const IRQ_EN     = 0b0000_0100;      // Enable read-complete interrupts
        const FAST       = 0b0000_0010;      // Enable fast mode (must be disabled for power-down)
        const LOW        = 0b0000_0001;      // Low power mode
    }
}

bitflags! {
    /// Device Mode2 register
    pub struct Mode2: u8 {
        const TEMP_DISABLE   = 0b1000_0000;     // DISABLE temperature measurement
        const LOW_POW_PERIOD = 0b0100_0000;     // Set low power period ("0": 100ms, "1": 12ms)
        const PARITY_TEST_EN = 0b0010_0000;     // Enable / Disable parity test
    }
}

/// Tlv493d related errors
#[derive(Debug)]
#[cfg_attr(feature = "std", derive(thiserror::Error))] 
pub enum Error<E: core::fmt::Debug> {
    // No device found with specified i2c bus and address
    #[cfg_attr(feature = "std", error("No device found with specified i2c bus and address"))] 
    NoDevice,

    // Device ADC locked up and must be reset
    #[cfg_attr(feature = "std", error("Device ADC lockup, reset required"))] 
    AdcLockup,

    // Uncorrect I2C address
    #[cfg_attr(feature = "std", error("Sensor I2C address must be 0x5E or 0x1F"))] 
    WrongI2CAddress,

    // Underlying I2C device error
    #[cfg_attr(feature = "std", error("I2C device error: {0:?}"))] 
    I2c(E),
}


impl <I2c, E> Tlv493d<I2c, E>
where
    I2c: i2c::Read<Error = E> + i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
    E: core::fmt::Debug,
{
    /// Create a new TLV493D instance
    pub fn new(i2c: I2c, addr: u8, mode: &Mode) -> Result<Self, Error<E>> {
        debug!("New Tlv493d with address: 0x{:02x}", addr);
        
        // Construct object
        let mut s = Self {
            i2c, addr, initial: [0u8; 10], last_frm: 0xff, _e: PhantomData,
        };

        ////////////////////////// STARTUP PER FIG. 5.1 IN TLV493D-A1B6 USER MANUEL ///////////////////////
        // Write recovery value
        let _= s.i2c.write(0xFF, &[]);

        // Send reset command
        match s.addr {
            ADDRESS_BASE_1 => { let _= s.i2c.write(0x00, &[0xFF]); } //The result is not used because it  will be an I2C error, because theses commands reset sensor and set-up
            ADDRESS_BASE_2 => { let _= s.i2c.write(0x00, &[0x00]); } //its address, and the sensor do not respond with Ack-bit, so i2c method will interpret it as an error.
            _other         => { return Err(Error::WrongI2CAddress) }
        }
       
        // Read initial bitmap from device
        let _ = s.i2c.read(s.addr, &mut s.initial[..]).map_err(Error::I2c)?;
        debug!("initial read: {:02x?}", s.initial);

        // Set mode
        s.configure(mode)?;

        // Return object
        Ok(s)
    }

    // Method to configure the mode of foperation of the sensor
    //  INPUTS :    - Mode : mode to use
    //
    //  OUTPUTS :   - Result<(), Error<E>>
    //
    pub fn configure(&mut self, mode: &Mode) -> Result<(), Error<E>> {
        let mut m1 = unsafe { Mode1::from_bits_unchecked(self.initial[7]) };
        let m2 = unsafe { Mode2::from_bits_unchecked(self.initial[9]) };

        // Clear mode flags
        m1.remove(Mode1::PARITY);
        m1.remove(Mode1::FAST | Mode1::LOW);

        match mode {
            Mode::Disabled => (),
            Mode::Master            => m1 |= Mode1::FAST | Mode1::LOW,
            Mode::Fast              => m1 |= Mode1::FAST | Mode1::IRQ_EN,
            Mode::LowPower          => m1 |= Mode1::LOW | Mode1::IRQ_EN,
            Mode::UltraLowPower     => m1 |= Mode1::IRQ_EN,
        }

        let mut cfg = [
            0x00,
            m1.bits(),
            self.initial[8],
            m2.bits(),
        ];

        let mut parity = 0;
        for v in &cfg {
            for i in 0..8 {
                if v & (1 << i) != 0 {
                    parity += 1;
                }
            }
        }
        if parity % 2 == 0 {
            m1 |= Mode1::PARITY;
            cfg[1] = m1.bits();
        }

        self.i2c.write(self.addr, &cfg).map_err(Error::I2c)?;

        Ok(())
    }

    // Method to read raw values from the sensor
    // The sensor need to be already configured
    //  INPUTS :    - 
    //
    //  OUTPUTS :   - Result<[i16; 4], Error<E>>
    //                where [i16; 4] contains the values ==> Bx, By, Bz, Temp (no units)
    //
    pub fn read_raw(&mut self) -> Result<[i16; 4], Error<E>> {
        let mut v = [0i16; 4];

        // Read data from device
        let mut b = [0u8; 7];
        self.i2c.read(self.addr, &mut b[..]).map_err(Error::I2c)?;

        // Detect ADC lockup (stalled FRM field)
        let frm = b[3] & 0b0000_1100;
        if self.last_frm == frm {
            return Err(Error::AdcLockup)
        } else {
            self.last_frm = frm;
        }
        
        // Convert to values (no units)
        // Double-cast here required for sign-extension
        v[0] = (b[0] as i8 as i16) << 4 | ((b[4] & 0xF0) >> 4) as i16;
        v[1] = (b[1] as i8 as i16) << 4 | (b[4] & 0x0F) as i16;
        v[2] = (b[2] as i8 as i16) << 4 | (b[5] & 0x0F) as i16;
        v[3] = (b[3] as i8 as i16 & 0xF0) << 4 | (b[6] as i16 & 0xFF);

        debug!("Read data {:02x?} values: {:04x?}", b, v);

        Ok(v)
    }


    // Read data and convert to correct values in mT and °C
    // The sensor need to be already configured
    //  INPUTS :    - 
    //
    //  OUTPUTS :   - Result<Values, Error<E>>
    //                where Values contains the values ==> Bx, By, Bz, Temp (with good units)
    //
    pub fn read(&mut self) -> Result<Values, Error<E>> {
        let raw = self.read_raw()?;
        Ok(Values {
            x: raw[0] as f32 * 0.098f32,
            y: raw[1] as f32 * 0.098f32,
            z: raw[2] as f32 * 0.098f32,
            temp: (raw[3] - 340) as f32 * 1.1f32 + 24.2f32,
        })
    }


    // Method that read N times the magnetic field values on the sensor and return
    // a Bfield struct containing the mean of the N measurements.
    // The means and standard-deviations are computed iteratively
    //
    // INPUTS  : - n     : u16 (number of measurement on which we want to compute the mean)
    //           - delay : &mut rppal::hal::Delay (struct needed )
    //
    // OUTPUTS : - Result --> Ok(Bfield struct), or Err(Error struct) 
    //
    pub fn get_b_mean(&mut self, n: u16, t: u16, delay: &mut rppal::hal::Delay) -> Result<Bfield, Error<E>>  {
        //initialize b, and set counter to zero
        let mut b = Bfield {bx:0.0, by:0.0, bz:0.0, ux:0.0, uy:0.0, uz:0.0};
        let mut j = 0; //counter of valid measurements
        let mut i = 0; //counter of invalid measurements (read_raw function returning an error )

        //These variables are needed to compte Mean and StandardDeviation iteratively
        let (mut bx_prev, mut by_prev, mut bz_prev): (f32, f32, f32);
        
        //Looping until we get N valid measurements
        while j!= n {
            (*delay).delay_ms(t as u16);
            match self.read()  {
                Ok(b_new) => {
                    j += 1;    //Iterating the number of valid measurements

                    bx_prev = b.bx; //
                    by_prev = b.by; //Storing previous means in 3 variables
                    bz_prev = b.bz; //

                    b.bx = b.bx + (b_new.x - b.bx) / (j as f32) ; //
                    b.by = b.by + (b_new.y - b.by) / (j as f32) ; //Compute new means
                    b.bz = b.bz + (b_new.z - b.bz) / (j as f32) ; //

                    b.ux = b.ux + (b_new.x - bx_prev) * (b_new.x - b.bx) ; //
                    b.uy = b.uy + (b_new.y - by_prev) * (b_new.y - b.by) ; //Compute new variances
                    b.uz = b.uz + (b_new.z - bz_prev) * (b_new.z - b.bz) ; //
                }
                Err(error) => {
                    i += 1;   //Iterating the number of invalid mesurements
                    if i > n {
                        return Err(error);
                    }
                }
            }
        }
        b.ux = (b.ux / (j as f32)).sqrt(); //
        b.uy = (b.uy / (j as f32)).sqrt(); //Compute final standard-deviations
        b.uz = (b.uz / (j as f32)).sqrt(); //

        Ok(b) //Returning Ok(b) if the n measurements are made correctly
    }


    // Function that read N times the magnetic field values on the sensor, average them, and then
    // return a tuple containing the angle between Bxy and X axis, and its standard deviation.
    //
    // INPUTS  : - n     : u16 (number of measurement on which we want to compute the mean)
    //           - delay : &mut rppal::hal::Delay (struct needed )
    //
    // OUTPUTS : - Result --> Ok(Bfield struct), or Err(Error struct) 
    // 
    pub fn get_bxy_angle(&mut self, n: u16, t: u16, delay: &mut rppal::hal::Delay) -> Result<(f32, f32), Error<E>>  {
        let angle_xy:   f32;
        let u_angle_xy: f32;
        match self.get_b_mean(n, t, delay) {
            Ok(b) => {
                (angle_xy, u_angle_xy) = b.b_angle_xy();
                Ok((angle_xy, u_angle_xy))
            }
            Err(error) => { return Err(error)}
        }
    }

}







////////////////////////////////////////////////////////////////////////////////////
//                        AJOUT PAR MOI (ANTOINE DE LAUNAY)
////////////////////////////////////////////////////////////////////////////////////

// Struct that contains the 3 components of the magnetic field, in mT, and their standard-deviations (>0 if several measurements are made)
// "Values" already exists but does not include standard-deviation. A little bit redundant...
pub struct Bfield {
    pub bx : f32, // X axis field compenent
    pub by : f32, // Y axis field component
    pub bz : f32, // Z axis field component
    pub ux : f32, // Standard deviation for X axis compenent
    pub uy : f32, // Standard deviation for X axis compenent
    pub uz : f32, // Standard deviation for Z axis component
}





impl Bfield {

    // Function that compute the angle made by the magnetic field on the XY plan based on
    // a N-measurements mean value of the magnetic field
    //
    // INPUTS  : - b : Bfield struct.
    //
    // OUTPUTS : - angle_xy : f32
    //                --->       1000.0 if magnetic field is too low to get a reliable value
    //                ---> 0.0 to 360.0 if measurement is correctly done
    //           - u_angle_xy : f32 (computed standard-deviation on angle_xy, in degrees °)
    //
    pub fn b_angle_xy(& self) -> (f32,f32) 
    {
        let b = self;
        //Norm of magnetic field. If it is too low, the computed angle will not be reliable (it will mostly be based on noise)
        let norm_b = ( b.bx.powf(2.0) + b.by.powf(2.0) + b.bz.powf(2.0) ).sqrt();
        if norm_b > 1.5 {
            //Compute angle (four quadrant arctangent of By/Bx ), and its standard-deviation.
            let angle_xy = b.by.atan2(b.bx) * 180.0 / 3.1415;
            let u_angle_xy = ( (b.ux * b.by / (b.bx*b.bx + b.by*b.by)).powf(2.0)   +   (b.uy / (b.bx + b.by*b.by/b.bx )).powf(2.0) ).sqrt() * 180.0/3.1415;
            (angle_xy, u_angle_xy)
        } else {
            (1000.0, 0.0)
        }
    }

}