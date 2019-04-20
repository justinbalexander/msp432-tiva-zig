const std = @import("std");
const builtin = @import("builtin");
use @import("mcu_hw");

const main = @import("main.zig").main;
//TODO see first isrVector
//export var systemStack: [64]u32 = undefined;

///****************************************************************************
/// These are symbols that the linker will provide. They will have the address
/// as indicated in the linker script.
///****************************************************************************
extern var _ldata: u8;
extern var _data: u8;
extern var _edata: u8;
extern var _bss: u8;
extern var _ebss: u8;

const ISRHandler = extern fn () void;

export const isrVectors linksection(".isr_vector") = []ISRHandler{
    //TODO: unreachable, gen_const_val, not hardcoded or another function?
    //    @ptrCast(ISRHandler, &systemStack[systemStack.len - 1]),
    @intToPtr(ISRHandler, 0x20008000), // The initial stack pointer

    @ptrCast(ISRHandler, resetISR), // The reset handler
    nmiSR, // The NMI handler
    hardFaultISR, // The hard fault handler
    intDefaultHandler, // The MPU fault handler
    intDefaultHandler, // The bus fault handler
    intDefaultHandler, // The usage fault handler
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    intDefaultHandler, // SVCall handler
    intDefaultHandler, // Debug monitor handler
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    intDefaultHandler, // The PendSV handler
    intDefaultHandler, // The SysTick handler
    intDefaultHandler, // GPIO Port A
    intDefaultHandler, // GPIO Port B
    intDefaultHandler, // GPIO Port C
    intDefaultHandler, // GPIO Port D
    intDefaultHandler, // GPIO Port E
    intDefaultHandler, // UART0 Rx and Tx
    intDefaultHandler, // UART1 Rx and Tx
    intDefaultHandler, // SSI0 Rx and Tx
    intDefaultHandler, // I2C0 Master and Slave
    intDefaultHandler, // PWM Fault
    intDefaultHandler, // PWM Generator 0
    intDefaultHandler, // PWM Generator 1
    intDefaultHandler, // PWM Generator 2
    intDefaultHandler, // Quadrature Encoder 0
    intDefaultHandler, // ADC Sequence 0
    intDefaultHandler, // ADC Sequence 1
    intDefaultHandler, // ADC Sequence 2
    intDefaultHandler, // ADC Sequence 3
    intDefaultHandler, // Watchdog timer
    intDefaultHandler, // Timer 0 subtimer A
    intDefaultHandler, // Timer 0 subtimer B
    intDefaultHandler, // Timer 1 subtimer A
    intDefaultHandler, // Timer 1 subtimer B
    intDefaultHandler, // Timer 2 subtimer A
    intDefaultHandler, // Timer 2 subtimer B
    intDefaultHandler, // Analog Comparator 0
    intDefaultHandler, // Analog Comparator 1
    intDefaultHandler, // Analog Comparator 2
    intDefaultHandler, // System Control (PLL, OSC, BO)
    intDefaultHandler, // FLASH Control
    intDefaultHandler, // GPIO Port F
    intDefaultHandler, // GPIO Port G
    intDefaultHandler, // GPIO Port H
    intDefaultHandler, // UART2 Rx and Tx
    intDefaultHandler, // SSI1 Rx and Tx
    intDefaultHandler, // Timer 3 subtimer A
    intDefaultHandler, // Timer 3 subtimer B
    intDefaultHandler, // I2C1 Master and Slave
    intDefaultHandler, // Quadrature Encoder 1
    intDefaultHandler, // CAN0
    intDefaultHandler, // CAN1
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    intDefaultHandler, // Hibernate
    intDefaultHandler, // USB0
    intDefaultHandler, // PWM Generator 3
    intDefaultHandler, // uDMA Software Transfer
    intDefaultHandler, // uDMA Error
    intDefaultHandler, // ADC1 Sequence 0
    intDefaultHandler, // ADC1 Sequence 1
    intDefaultHandler, // ADC1 Sequence 2
    intDefaultHandler, // ADC1 Sequence 3
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    intDefaultHandler, // GPIO Port J
    intDefaultHandler, // GPIO Port K
    intDefaultHandler, // GPIO Port L
    intDefaultHandler, // SSI2 Rx and Tx
    intDefaultHandler, // SSI3 Rx and Tx
    intDefaultHandler, // UART3 Rx and Tx
    intDefaultHandler, // UART4 Rx and Tx
    intDefaultHandler, // UART5 Rx and Tx
    intDefaultHandler, // UART6 Rx and Tx
    intDefaultHandler, // UART7 Rx and Tx
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    intDefaultHandler, // I2C2 Master and Slave
    intDefaultHandler, // I2C3 Master and Slave
    intDefaultHandler, // Timer 4 subtimer A
    intDefaultHandler, // Timer 4 subtimer B
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    intDefaultHandler, // Timer 5 subtimer A
    intDefaultHandler, // Timer 5 subtimer B
    intDefaultHandler, // Wide Timer 0 subtimer A
    intDefaultHandler, // Wide Timer 0 subtimer B
    intDefaultHandler, // Wide Timer 1 subtimer A
    intDefaultHandler, // Wide Timer 1 subtimer B
    intDefaultHandler, // Wide Timer 2 subtimer A
    intDefaultHandler, // Wide Timer 2 subtimer B
    intDefaultHandler, // Wide Timer 3 subtimer A
    intDefaultHandler, // Wide Timer 3 subtimer B
    intDefaultHandler, // Wide Timer 4 subtimer A
    intDefaultHandler, // Wide Timer 4 subtimer B
    intDefaultHandler, // Wide Timer 5 subtimer A
    intDefaultHandler, // Wide Timer 5 subtimer B
    intDefaultHandler, // FPU
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    intDefaultHandler, // I2C4 Master and Slave
    intDefaultHandler, // I2C5 Master and Slave
    intDefaultHandler, // GPIO Port M
    intDefaultHandler, // GPIO Port N
    intDefaultHandler, // Quadrature Encoder 2
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    @intToPtr(ISRHandler, std.math.maxInt(usize)), // Reserved
    intDefaultHandler, // GPIO Port P (Summary or P0)
    intDefaultHandler, // GPIO Port P1
    intDefaultHandler, // GPIO Port P2
    intDefaultHandler, // GPIO Port P3
    intDefaultHandler, // GPIO Port P4
    intDefaultHandler, // GPIO Port P5
    intDefaultHandler, // GPIO Port P6
    intDefaultHandler, // GPIO Port P7
    intDefaultHandler, // GPIO Port Q (Summary or Q0)
    intDefaultHandler, // GPIO Port Q1
    intDefaultHandler, // GPIO Port Q2
    intDefaultHandler, // GPIO Port Q3
    intDefaultHandler, // GPIO Port Q4
    intDefaultHandler, // GPIO Port Q5
    intDefaultHandler, // GPIO Port Q6
    intDefaultHandler, // GPIO Port Q7
    intDefaultHandler, // GPIO Port R
    intDefaultHandler, // GPIO Port S
    intDefaultHandler, // PWM 1 Generator 0
    intDefaultHandler, // PWM 1 Generator 1
    intDefaultHandler, // PWM 1 Generator 2
    intDefaultHandler, // PWM 1 Generator 3
    intDefaultHandler, // PWM 1 Fault
};

extern nakedcc fn resetISR() noreturn {
    const init_data_length = @ptrToInt(&_edata) - @ptrToInt(&_data);
    const init_data_slice = @ptrCast([*]u8, &_ldata)[0..init_data_length];
    var ram_data_slice = @ptrCast([*]u8, &_data)[0..init_data_length];
    std.mem.copy(u8, init_data_slice, ram_data_slice);

    const bss_slice = @ptrCast([*]u8, &_bss)[0 .. @ptrToInt(&_ebss) - @ptrToInt(&_bss)];
    std.mem.set(u8, bss_slice, 0);

    NVIC_CPAC_R.* = ((NVIC_CPAC_R.* &
        ~(NVIC.CPAC.CP10_M | NVIC.CPAC.CP11_M)) |
        NVIC.CPAC.CP10_FULL | NVIC.CPAC.CP11_FULL);

    main();
}

extern fn nmiSR() void {
    while (true) {}
}

extern fn hardFaultISR() void {
    while (true) {}
}

extern fn intDefaultHandler() void {
    while (true) {}
}
