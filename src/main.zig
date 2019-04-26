// Example blinks LED using busy loop on TM4C123G Launchpad

const std = @import("std");
const hw = @import("mcu_hw");

var loop_mem: u32 = 0;
const looper = @ptrCast(*volatile u32, &loop_mem);

const BIT0 = 1 << 0;
const BIT1 = 1 << 1;
const BIT2 = 1 << 2;
const BIT3 = 1 << 3;
const BIT4 = 1 << 4;
const BIT5 = 1 << 5;
const BIT6 = 1 << 6;
const BIT7 = 1 << 7;

fn initHardware() void {
    // enable peripheral, takes 5 cycles before access available, else bus fault
    hw.SYSCTL_RCGCGPIO_R.* |= hw.SYSCTL.RCGC2.GPIOF;
    while (hw.SYSCTL_PRGPIO_R.* & hw.SYSCTL.RCGC2.GPIOF != 0) {}
    hw.GPIO_PORTF_DIR_R.* |= BIT3;
    hw.GPIO_PORTF_DEN_R.* |= BIT3;
}

pub fn main() noreturn {
    initHardware();

    while (true) {
        hw.GPIO_PORTF_DATA_R.* |= BIT3;
        looper.* = 0xFFFF;
        while (looper.* != 0) {
            looper.* -= 1;
        }
        hw.GPIO_PORTF_DATA_R.* &= ~BIT3;
    }
}
