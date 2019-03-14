const Builder = @import("std").build.Builder;
const builtin = @import("builtin");

const root_src = "src/startup.zig";
const out_file_name = "msp432-tiva-zig";
const out_dir = "zig-cache";
const linker_script = "link/project.ld";

pub fn build(b: *Builder) void {
    const mode = b.standardReleaseOptions();

    const arch = builtin.Arch{ .arm = builtin.Arch.Arm32.v7em };
    const environ = builtin.Abi.eabihf;

    const exe = b.addExecutable(out_file_name, root_src);
    exe.setOutputDir(out_dir);
    exe.setBuildMode(mode);
    exe.setTarget(arch, builtin.Os.freestanding, environ);
    exe.setLinkerScriptPath(linker_script);

    b.default_step.dependOn(&exe.step);
    b.installArtifact(exe);
}
