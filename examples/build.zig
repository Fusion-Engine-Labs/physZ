const std = @import("std");
const rlz = @import("raylib_zig");

// Each name maps to src/<name>.zig and gets a `run-<name>` step.
const examples = [_][]const u8{
    "sandbox",
};

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const raylib_dep = b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    });
    const raylib = raylib_dep.module("raylib");
    const raylib_artifact = raylib_dep.artifact("raylib");

    const physz_dep = b.dependency("physZ", .{
        .target = target,
        .optimize = optimize,
    });
    const physz = physz_dep.module("physZ");

    const run_step = b.step("run", "Run the default example (" ++ examples[0] ++ ")");

    for (examples, 0..) |name, i| {
        const exe_mod = b.createModule(.{
            .root_source_file = b.path(b.fmt("src/{s}.zig", .{name})),
            .target = target,
            .optimize = optimize,
        });
        exe_mod.addImport("raylib", raylib);
        exe_mod.addImport("physZ", physz);

        const example_run_step = b.step(b.fmt("run-{s}", .{name}), b.fmt("Run the {s} example", .{name}));
        if (i == 0) run_step.dependOn(example_run_step);

        if (target.query.os_tag == .emscripten) {
            const emsdk = rlz.emsdk;
            const wasm = b.addLibrary(.{
                .name = name,
                .root_module = exe_mod,
            });

            const install_dir: std.Build.InstallDir = .{ .custom = "web" };
            const emcc_flags = emsdk.emccDefaultFlags(b.allocator, .{ .optimize = optimize });
            const emcc_settings = emsdk.emccDefaultSettings(b.allocator, .{ .optimize = optimize });

            const emcc_step = emsdk.emccStep(b, raylib_artifact, wasm, .{
                .optimize = optimize,
                .flags = emcc_flags,
                .settings = emcc_settings,
                .shell_file_path = emsdk.shell(raylib_dep),
                .install_dir = install_dir,
                .embed_paths = &.{.{ .src_path = "resources/" }},
            });
            b.getInstallStep().dependOn(emcc_step);

            const html_filename = b.fmt("{s}.html", .{wasm.name});
            const emrun_step = emsdk.emrunStep(
                b,
                b.getInstallPath(install_dir, html_filename),
                &.{},
            );

            emrun_step.dependOn(emcc_step);
            example_run_step.dependOn(emrun_step);
        } else {
            const exe = b.addExecutable(.{
                .name = name,
                .root_module = exe_mod,
            });
            const install_exe = b.addInstallArtifact(exe, .{});
            b.getInstallStep().dependOn(&install_exe.step);

            const run_cmd = b.addRunArtifact(exe);
            run_cmd.step.dependOn(&install_exe.step);

            example_run_step.dependOn(&run_cmd.step);
        }
    }
}
