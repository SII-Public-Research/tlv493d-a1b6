## tlv493d_a1b6_raspberry

Rust driver for tlv493d_a1b6 sensor.

The crate is initially made by Ryankurte ( https://crates.io/crates/sensor-tlv493d/0.1.0 ).
Changed some small things because it was not working for me (the initialisation of the sensor, with the address setting up, ...)

I added somes functions for specific things I needed to do like:
     - Compute mean over N measurements, and compute standard-deviation of the measurement
     - Compute the angle (angle between the X axis and the vector Bxy) and its standard-deviation (based on bx, by, bz standard-deviations)
     - Added another struct (because "Values" struct has private fields !)

But these functions used delay type from Rppal crate (we work on a raspberry pi 3). So it may not work on another device
because of this, unless the crate is adapted by someone.

/!\ During tests, Master mode was working well, but not the others ones...