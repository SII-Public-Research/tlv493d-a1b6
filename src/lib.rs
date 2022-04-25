use embedded_hal::blocking::delay::*;


// Struct that contains the 3 components of the magnetic field, in mT, and their standard-deviations (>0 if several measurements are made)
pub struct Bfield {
    pub bx : f32, // X axis field compenent
    pub by : f32, // Y axis field component
    pub bz : f32, // Z axis field component
    pub ux : f32, // Standard deviation for X axis compenent
    pub uy : f32, // Standard deviation for X axis compenent
    pub uz : f32, // Standard deviation for Z axis component
}



// Function that read N times the magnetic field values on the sensor and return
// a Bfield struct containing the mean of the N measurements.
// The means and standard-deviations are computed iteratively
//
// INPUTS  : - tlv : &mut sensor_tlv493d::Tlv493d (struct containing parameters to use the sensor)
//           - n : u16 (number of measurement on which we want to compute the mean)
//           - delay : &mut rppal::hal::Delay (struct needed )
//
// OUTPUTS : - Result --> Ok(Bfield struct), or Err(Error struct) 
//
pub fn get_b_mean<I2c, E> (tlv : &mut sensor_tlv493d::Tlv493d<I2c, E>, n : u16, delay : &mut rppal::hal::Delay) -> Result<Bfield, sensor_tlv493d::Error<E>> 
where 
    I2c: embedded_hal::blocking::i2c::Read<Error = E> + embedded_hal::blocking::i2c::Write<Error = E> + embedded_hal::blocking::i2c::WriteRead<Error = E>,
    E: core::fmt::Debug,
{
    //initialize b, and set counter to zero
    let mut b = Bfield {bx:0.0, by:0.0, bz:0.0, ux:0.0, uy:0.0, uz:0.0};
    let mut j = 0; //counter of valid measurements
    let mut i = 0; //counter of invalid measurements (read_raw function returning an error )
    //These variables are needed to compte Mean and StandardDeviation iteratively
    let (mut bx_new, mut by_new, mut bz_new, mut bx_prev, mut by_prev, mut bz_prev): (f32, f32, f32, f32, f32, f32);
    //Looping until we get N valid measurements
    while j!= n {
        (*delay).delay_ms(5 as u16);
        match tlv.read_raw()  {
            Ok(v) => {
                j += 1;    //Iterating the number of valid measurements

                bx_prev = b.bx; //
                by_prev = b.by; //Storing previous means in 3 variables
                bz_prev = b.bz; //

                bx_new  = (v[0] as f32) * 0.098; //
                by_new  = (v[1] as f32) * 0.098; //Compute new measurements
                bz_new  = (v[2] as f32) * 0.098; //

                b.bx = b.bx + (bx_new - b.bx) / (j as f32) ; //
                b.by = b.by + (by_new - b.by) / (j as f32) ; //Compute new means
                b.bz = b.bz + (bz_new - b.bz) / (j as f32) ; //

                b.ux = b.ux + (bx_new - bx_prev) * (bx_new - b.bx) ; //
                b.uy = b.uy + (by_new - by_prev) * (by_new - b.by) ; //Compute new variances
                b.uz = b.uz + (bz_new - bz_prev) * (bz_new - b.bz) ; //
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
pub fn b_angle_xy(b : &Bfield ) -> (f32, f32) 
{
    //Initializing the two variables (computed angle made by Bfield on the XY plan, and its standard-deviation computed with Bfield values)
    let mut angle_xy   = 1000.0;
    let mut u_angle_xy = 0.0   ;
    //Norm of magnetic field. If it is too low, the computed angle will not be reliable (it will mostly be based on noise)
    let norm_b = ( b.bx.powf(2.0) + b.by.powf(2.0) + b.bz.powf(2.0) ).sqrt();
    if norm_b > 1.5 {
        //Compute angle (four quadrant arctangent of By/Bx ), and its standard-deviation.
        angle_xy = b.by.atan2(b.bx) * 180.0 / 3.1415;
        u_angle_xy = ( (b.ux * b.by / (b.bx*b.bx + b.by*b.by)).powf(2.0)   +   (b.uy / (b.bx + b.by*b.by/b.bx )).powf(2.0) ).sqrt() * 180.0/3.1415;
    }
   return (angle_xy, u_angle_xy);
}