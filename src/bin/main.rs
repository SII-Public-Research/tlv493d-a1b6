use std;
use rppal::i2c::I2c;
use sensor_tlv493d::*;
use tlv493d_raspberry::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create MPU
    let i2c = I2c::new().expect("1");
    let mut tlv = Tlv493d::new(i2c, ADDRESS_BASE, Mode::LowPower).unwrap();
    let mut delay = rppal::hal::Delay::new();

    //Initialize the variable which will contain data
    let mut _bfield: Bfield;
    let mut angle_xy: f32;
    let mut _u_angle_xy: f32;

    //Setting up the number of measurement on which we want to compute the mean
    let n = 50;


    loop {
        // get 3d hall data (computing means and standard-deviations over N measurements) : bx, by, bz
        match get_b_mean(&mut tlv, n, &mut delay) {
            Ok(b) => {
                (angle_xy, _u_angle_xy) = b_angle_xy(&b);
                println!("Bx={}mT By={}mT Bz={}mT AngleXY={}°",b.bx, b.by, b.bz, angle_xy);
                _bfield = b;
            }
            Err(error) => {
                println!("Erreur : {}", error);
            }
        } //end match
    } //end loop
} //end main