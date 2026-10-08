const std = @import("std");
const rl = @import("raylib");
const physZ = @import("physZ");

const World = physZ.World;
const Vec2 = physZ.Vec2;

const fixed_dt: f32 = 1.0 / 60.0;
const max_steps_per_frame: u32 = 8;

const font_size = 20;
const line_height = 24;
const panel_padding = 10;

const Palette = struct {
    bg: rl.Color,
    grid: rl.Color,
    axis: rl.Color,
    ink: rl.Color,
    panel: rl.Color,
    panel_border: rl.Color,
};

// Warm paper, and the same hues dropped to a soft charcoal.
const light_palette: Palette = .{
    .bg = .{ .r = 253, .g = 248, .b = 240, .a = 255 },
    .grid = .{ .r = 238, .g = 230, .b = 220, .a = 255 },
    .axis = .{ .r = 214, .g = 202, .b = 190, .a = 255 },
    .ink = .{ .r = 84, .g = 74, .b = 102, .a = 255 },
    .panel = .{ .r = 255, .g = 255, .b = 255, .a = 220 },
    .panel_border = .{ .r = 230, .g = 218, .b = 236, .a = 255 },
};
const dark_palette: Palette = .{
    .bg = .{ .r = 42, .g = 38, .b = 36, .a = 255 },
    .grid = .{ .r = 62, .g = 56, .b = 52, .a = 255 },
    .axis = .{ .r = 96, .g = 86, .b = 78, .a = 255 },
    .ink = .{ .r = 232, .g = 226, .b = 236, .a = 255 },
    .panel = .{ .r = 54, .g = 50, .b = 56, .a = 230 },
    .panel_border = .{ .r = 96, .g = 86, .b = 108, .a = 255 },
};

const paused_color: rl.Color = .{ .r = 236, .g = 120, .b = 100, .a = 255 };
const running_color: rl.Color = .{ .r = 70, .g = 170, .b = 120, .a = 255 };
const orientation_color: rl.Color = .{ .r = 84, .g = 74, .b = 102, .a = 140 };
const static_color: rl.Color = .{ .r = 196, .g = 190, .b = 212, .a = 255 };
const dynamic_colors = [_]rl.Color{
    .{ .r = 255, .g = 179, .b = 198, .a = 255 }, // pink
    .{ .r = 160, .g = 210, .b = 245, .a = 255 }, // sky
    .{ .r = 170, .g = 230, .b = 190, .a = 255 }, // mint
    .{ .r = 255, .g = 204, .b = 153, .a = 255 }, // peach
    .{ .r = 200, .g = 180, .b = 240, .a = 255 }, // lavender
    .{ .r = 253, .g = 236, .b = 150, .a = 255 }, // butter
};

const Camera = struct {
    center: Vec2 = Vec2.init(0, 4),
    pixels_per_meter: f32 = 50,

    fn toScreen(cam: Camera, p: Vec2) rl.Vector2 {
        const w: f32 = @floatFromInt(rl.getScreenWidth());
        const h: f32 = @floatFromInt(rl.getScreenHeight());
        return .{
            .x = w / 2 + (p.x - cam.center.x) * cam.pixels_per_meter,
            .y = h / 2 - (p.y - cam.center.y) * cam.pixels_per_meter,
        };
    }

    fn toWorld(cam: Camera, s: rl.Vector2) Vec2 {
        const w: f32 = @floatFromInt(rl.getScreenWidth());
        const h: f32 = @floatFromInt(rl.getScreenHeight());
        return Vec2.init(
            cam.center.x + (s.x - w / 2) / cam.pixels_per_meter,
            cam.center.y - (s.y - h / 2) / cam.pixels_per_meter,
        );
    }

    fn zoomAt(cam: *Camera, anchor: rl.Vector2, factor: f32) void {
        const before = cam.toWorld(anchor);
        cam.pixels_per_meter = std.math.clamp(cam.pixels_per_meter * factor, 5, 500);
        const after = cam.toWorld(anchor);
        cam.center = cam.center.add(before.sub(after));
    }
};

pub const StepHook = *const fn (world: *World, step: u64) void;

pub const Options = struct {
    title: [:0]const u8 = "physZ",
    on_step: ?StepHook = null,
    width: i32 = 1280,
    height: i32 = 720,
};

const Sim = struct {
    world: *World,
    // Copy of the world as the scene built it, restored on restart.
    initial: World,
    on_step: ?StepHook,
    paused: bool = true,
    time_scale: f32 = 1,
    accumulator: f32 = 0,
    step_count: u64 = 0,
    sim_time: f64 = 0,
    step_ms: f64 = 0,
    steps_last_frame: u32 = 0,

    fn init(world: *World, on_step: ?StepHook) !Sim {
        var initial = world.*;
        initial.bodies = try world.bodies.clone(world.allocator);
        return .{ .world = world, .initial = initial, .on_step = on_step };
    }

    fn deinit(sim: *Sim) void {
        sim.initial.bodies.deinit(sim.initial.allocator);
    }

    fn restart(sim: *Sim) !void {
        var bodies = sim.world.bodies;
        bodies.clearRetainingCapacity();
        try bodies.appendSlice(sim.world.allocator, sim.initial.bodies.items);

        sim.world.* = sim.initial;
        sim.world.bodies = bodies;

        sim.accumulator = 0;
        sim.step_count = 0;
        sim.sim_time = 0;
    }

    fn stepOnce(sim: *Sim) !void {
        const start = rl.getTime();
        if (sim.on_step) |hook| hook(sim.world, sim.step_count);
        try sim.world.step(fixed_dt);
        sim.step_ms += (rl.getTime() - start) * 1000;
        sim.step_count += 1;
        sim.steps_last_frame += 1;
        sim.sim_time += fixed_dt;
    }

    fn update(sim: *Sim, frame_dt: f32) !void {
        sim.step_ms = 0;
        sim.steps_last_frame = 0;
        if (sim.paused) return;

        sim.accumulator += frame_dt * sim.time_scale;
        var steps: u32 = 0;
        while (sim.accumulator >= fixed_dt and steps < max_steps_per_frame) : (steps += 1) {
            try sim.stepOnce();
            sim.accumulator -= fixed_dt;
        }
        if (steps == max_steps_per_frame) sim.accumulator = 0;
    }
};

pub fn run(world: *World, opts: Options) !void {
    rl.setConfigFlags(.{ .window_resizable = true, .window_maximized = true, .msaa_4x_hint = true });
    rl.initWindow(opts.width, opts.height, opts.title);
    defer rl.closeWindow();

    rl.setTargetFPS(60);

    var sim = try Sim.init(world, opts.on_step);
    defer sim.deinit();

    var camera: Camera = .{};
    var show_help = true;
    var dark = false;

    while (!rl.windowShouldClose()) {
        if (rl.isKeyPressed(.space)) sim.paused = !sim.paused;
        if (rl.isKeyPressed(.r)) try sim.restart();
        if (rl.isKeyPressed(.d)) dark = !dark;
        if (rl.isKeyPressed(.f1) or rl.isKeyPressed(.h)) show_help = !show_help;
        if (rl.isKeyPressed(.home) or rl.isKeyPressed(.c)) camera = .{};
        if (rl.isKeyPressed(.left_bracket)) sim.time_scale = @max(sim.time_scale / 2, 0.125);
        if (rl.isKeyPressed(.right_bracket)) sim.time_scale = @min(sim.time_scale * 2, 8);

        const wheel = rl.getMouseWheelMove();
        if (wheel != 0) camera.zoomAt(rl.getMousePosition(), std.math.pow(f32, 1.1, wheel));
        if (rl.isMouseButtonDown(.right) or rl.isMouseButtonDown(.middle)) {
            const d = rl.getMouseDelta();
            camera.center = camera.center.sub(Vec2.init(d.x, -d.y).scale(1 / camera.pixels_per_meter));
        }

        try sim.update(rl.getFrameTime());
        if (sim.paused and (rl.isKeyPressed(.n) or rl.isKeyPressedRepeat(.n) or
            rl.isKeyPressed(.right) or rl.isKeyPressedRepeat(.right)))
        {
            try sim.stepOnce();
        }

        rl.beginDrawing();
        defer rl.endDrawing();

        const palette = if (dark) dark_palette else light_palette;
        rl.clearBackground(palette.bg);
        drawGrid(camera, palette);
        drawBodies(sim.world, camera);
        drawStats(&sim, palette);
        if (show_help) drawHelp(palette);
    }
}

fn drawGrid(cam: Camera, palette: Palette) void {
    const top_left = cam.toWorld(.{ .x = 0, .y = 0 });
    const bottom_right = cam.toWorld(.{
        .x = @floatFromInt(rl.getScreenWidth()),
        .y = @floatFromInt(rl.getScreenHeight()),
    });

    var spacing: f32 = 1;
    while (spacing * cam.pixels_per_meter < 20) spacing *= 5;

    var x = @floor(top_left.x / spacing) * spacing;
    while (x <= bottom_right.x) : (x += spacing) {
        const color = if (x == 0) palette.axis else palette.grid;
        rl.drawLineV(cam.toScreen(Vec2.init(x, top_left.y)), cam.toScreen(Vec2.init(x, bottom_right.y)), color);
    }
    var y = @floor(bottom_right.y / spacing) * spacing;
    while (y <= top_left.y) : (y += spacing) {
        const color = if (y == 0) palette.axis else palette.grid;
        rl.drawLineV(cam.toScreen(Vec2.init(top_left.x, y)), cam.toScreen(Vec2.init(bottom_right.x, y)), color);
    }
}

fn drawBodies(world: *const World, cam: Camera) void {
    var dynamic_index: usize = 0;
    for (world.bodies.items) |body| {
        const color = switch (body.kind) {
            .static => static_color,
            .dynamic => blk: {
                defer dynamic_index += 1;
                break :blk dynamic_colors[dynamic_index % dynamic_colors.len];
            },
        };
        const center = cam.toScreen(body.position);
        const angle = body.angle;

        switch (body.shape) {
            .circle => |c| {
                const r = c.radius * cam.pixels_per_meter;
                rl.drawCircleV(center, r, color);
                // Radius line so spin is visible.
                const rim = body.position.add(Vec2.init(@cos(angle), @sin(angle)).scale(c.radius));
                rl.drawLineEx(center, cam.toScreen(rim), 2, orientation_color);
            },
            .box => |b| {
                const size: rl.Vector2 = .{
                    .x = 2 * b.half_extents.x * cam.pixels_per_meter,
                    .y = 2 * b.half_extents.y * cam.pixels_per_meter,
                };
                const rect: rl.Rectangle = .{ .x = center.x, .y = center.y, .width = size.x, .height = size.y };
                // World angles are counter-clockwise with y up; raylib rotates clockwise with y down.
                rl.drawRectanglePro(rect, .{ .x = size.x / 2, .y = size.y / 2 }, -std.math.radiansToDegrees(angle), color);
            },
        }
    }
}

fn drawStats(sim: *const Sim, palette: Palette) void {
    var bufs: [9][64]u8 = undefined;
    var lines: [9][:0]const u8 = undefined;
    var n: usize = 0;

    const fps = rl.getFPS();
    const frame_ms = rl.getFrameTime() * 1000;
    const format = struct {
        fn f(buf: *[64]u8, comptime fmt: []const u8, args: anytype) [:0]const u8 {
            return std.mem.printSentinel(buf, fmt, args, 0) catch "<overflow>";
        }
    }.f;

    lines[n] = format(&bufs[n], "{s}", .{if (sim.paused) "PAUSED" else "RUNNING"});
    n += 1;
    lines[n] = format(&bufs[n], "FPS: {d}", .{fps});
    n += 1;
    lines[n] = format(&bufs[n], "Frame: {d:.2} ms", .{frame_ms});
    n += 1;
    lines[n] = format(&bufs[n], "Physics: {d:.3} ms ({d} steps)", .{ sim.step_ms, sim.steps_last_frame });
    n += 1;
    lines[n] = format(&bufs[n], "Bodies: {d}", .{sim.world.bodies.items.len});
    n += 1;
    lines[n] = format(&bufs[n], "Step: {d}", .{sim.step_count});
    n += 1;
    lines[n] = format(&bufs[n], "Sim time: {d:.2} s", .{sim.sim_time});
    n += 1;
    lines[n] = format(&bufs[n], "Speed: {d:.3}x", .{sim.time_scale});
    n += 1;
    lines[n] = format(&bufs[n], "dt: {d:.4} s", .{fixed_dt});
    n += 1;

    var max_width: i32 = 0;
    for (lines[0..n]) |line| max_width = @max(max_width, rl.measureText(line, font_size));

    const panel_w = max_width + 2 * panel_padding;
    const panel_h: i32 = @as(i32, @intCast(n)) * line_height + 2 * panel_padding - (line_height - font_size);
    const panel_x = rl.getScreenWidth() - panel_w - panel_padding;
    const panel_y = panel_padding;

    drawPanel(panel_x, panel_y, panel_w, panel_h, palette);
    for (lines[0..n], 0..) |line, i| {
        const color: rl.Color = if (i == 0) (if (sim.paused) paused_color else running_color) else palette.ink;
        rl.drawText(line, panel_x + panel_padding, panel_y + panel_padding + @as(i32, @intCast(i)) * line_height, font_size, color);
    }
}

fn drawHelp(palette: Palette) void {
    const lines = [_][:0]const u8{
        "Space      play / pause",
        "N / Right  step (while paused)",
        "R          restart",
        "[ / ]      slower / faster",
        "RMB drag   pan",
        "Wheel      zoom",
        "C / Home   reset camera",
        "D          dark mode",
        "H / F1     toggle this help",
    };

    const panel_h: i32 = lines.len * line_height + 2 * panel_padding - (line_height - font_size);
    var max_width: i32 = 0;
    for (lines) |line| max_width = @max(max_width, rl.measureText(line, font_size));

    const panel_x = panel_padding;
    const panel_y = rl.getScreenHeight() - panel_h - panel_padding;
    drawPanel(panel_x, panel_y, max_width + 2 * panel_padding, panel_h, palette);
    for (lines, 0..) |line, i| {
        rl.drawText(line, panel_x + panel_padding, panel_y + panel_padding + @as(i32, @intCast(i)) * line_height, font_size, palette.ink);
    }
}

fn drawPanel(x: i32, y: i32, w: i32, h: i32, palette: Palette) void {
    const rect: rl.Rectangle = .{
        .x = @floatFromInt(x),
        .y = @floatFromInt(y),
        .width = @floatFromInt(w),
        .height = @floatFromInt(h),
    };
    rl.drawRectangleRounded(rect, 0.15, 8, palette.panel);
    rl.drawRectangleRoundedLinesEx(rect, 0.15, 8, 2, palette.panel_border);
}
