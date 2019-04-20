const Builder = @import("std").build.Builder;
const builtin = @import("builtin");

const root_src = "src/startup.zig";
const out_file_name = "msp432-tiva-zig";
const out_dir = "zig-cache";
const linker_script = "link/project.ld";
const default_mcu = "tm4c123gh6pm";

pub fn build(b: *Builder) void {
    const mode = b.standardReleaseOptions();
    const mcu = b.option([]const u8, "microcontroller", "Must match microcontroller listed under \"inc/\" directory\n\tdefault - " ++ default_mcu) orelse default_mcu;

    const arch = builtin.Arch{ .arm = builtin.Arch.Arm32.v7em };
    const environ = builtin.Abi.eabihf;

    const exe = b.addExecutable(out_file_name, root_src);
    exe.addPackagePath("mcu_hw", "./inc/mcu_hw.zig");
    exe.addBuildOption([]const u8, "mcu", b.fmt("\"{}\"", mcu)); // TODO: reference from package?
    exe.setOutputDir(out_dir);
    exe.setBuildMode(mode);
    exe.setTarget(arch, builtin.Os.freestanding, environ);
    exe.setLinkerScriptPath(linker_script);

    b.default_step.dependOn(&exe.step);
    b.installArtifact(exe);
}
