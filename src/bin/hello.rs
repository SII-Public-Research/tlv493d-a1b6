#![no_main]
#![no_std]

use test_tlv493d as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    test_tlv493d::exit()
}
