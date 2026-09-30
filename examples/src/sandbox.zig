const std = @import("std");
const physZ = @import("physZ");

const visualize = @import("visualize.zig");

const World = physZ.World;
const Vec2 = physZ.Vec2;

pub fn main(init: std.process.Init) !void {
    var world = World.init(init.gpa, .{});
    defer world.deinit();

    _ = try world.createBody(.{
        .shape = .{ .circle = .{ .radius = 0.5 } },
        .position = Vec2.init(-3, 4),
    });
    _ = try world.createBody(.{
        .shape = .{ .circle = .{ .radius = 1 } },
        .position = Vec2.init(0, 4),
    });
    _ = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.75, 0.75) } },
        .position = Vec2.init(3, 4),
    });

    try visualize.run(&world, .{
        .title = "physZ - sandbox",
        .bounds = .{ .min = Vec2.init(-6, 0), .max = Vec2.init(6, 8) },
    });
}
