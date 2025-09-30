use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Break into debugger if attached, then spin.
    loop {
        cortex_m::asm::bkpt();
    }
}
