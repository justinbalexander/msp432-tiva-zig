// TODO: not working when made a package and then original build script
// adds build option
//const build_options = @import("build_options");
//pub use @import(build_options.mcu_selection ++ ".zig");
// work-around:
pub use @import("tm4c123gh6pm" ++ ".zig");

comptime {
    // Allows discovery of files to be included in build
    // TODO: make startup file for tm4c129 parts and others
    _ = @import("startup_tm4c123.zig");
}

pub const ADC = @import("hw_adc.zig");
pub const AES = @import("hw_aes.zig");
pub const CAN = @import("hw_can.zig");
pub const CCM = @import("hw_ccm.zig");
pub const COMP = @import("hw_comp.zig");
pub const DES = @import("hw_des.zig");
pub const EEPROM = @import("hw_eeprom.zig");
pub use @import("hw_emac.zig"); // two namespaces, one file
pub const EPI = @import("hw_epi.zig");
pub const FLASH = @import("hw_flash.zig");
pub const GPIO = @import("hw_gpio.zig");
pub const HIB = @import("hw_hibernate.zig");
pub const I2C = @import("hw_i2c.zig");
pub const LCD = @import("hw_lcd.zig");
pub const MEMMAP = @import("hw_memmap.zig");
pub const NVIC = @import("hw_nvic.zig");
pub const ONEWIRE = @import("hw_onewire.zig");
pub const PWM = @import("hw_pwm.zig");
pub const QEI = @import("hw_qei.zig");
pub const SHAMD5 = @import("hw_shamd5.zig");
pub const SSI = @import("hw_ssi.zig");
pub const SYSCTL = @import("hw_sysctl.zig");
pub const SYSEXC = @import("hw_sysexc.zig");
pub const TIMER = @import("hw_timer.zig");
pub const UART = @import("hw_uart.zig");
pub const UDMA = @import("hw_udma.zig");
pub const USB = @import("hw_usb.zig");
pub const WDT = @import("hw_watchdog.zig");
