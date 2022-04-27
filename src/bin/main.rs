use rppal::i2c::I2c;
use tlv493d_raspberry::{Tlv493d, *};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create MPU
    let mut i2c   = I2c::new().expect("1");
    let addr      =  ADDRESS_BASE_1;
    let mode      = Mode::Master;
    let mut tlv   = Tlv493d::new(i2c, addr, &mode).unwrap();
    let mut delay = rppal::hal::Delay::new();

    //Initialize the variable which will contain data
    let mut _bfield: Bfield;
    let mut angle_xy: f32;
    let mut _u_angle_xy: f32;

    //Setting up the number of measurement on which we want to compute the mean, and delay to apply between each measurement
    let n = 100;

    loop {
        // get 3d hall data (computing means and standard-deviations over N measurements) : bx, by, bz
        match tlv.get_b_mean(n, &mut delay) 
        {
            Ok(b) => 
            {   (angle_xy, _u_angle_xy) = b.b_angle_xy();
                println!("Bx={:.1}mT By={:.1}mT Bz={:.1}mT AngleXY={:.1}°",b.bx, b.by, b.bz, angle_xy);
                _bfield = b;
            }

            Err(error) => 
            {   println!("Erreur : {0:?}", error);
                match error 
                {   Error::AdcLockup => 
                    {   i2c = I2c::new().expect("1");
                        tlv = Tlv493d::new(i2c, addr, &mode).unwrap();
                    }
                    _other => {}
                }
            }
        } //end match
    } //end loop
} //end main