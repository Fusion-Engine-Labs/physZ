const std = @import("std");
const physZ = @import("physZ");

const visualize = @import("visualize.zig");

const World = physZ.World;
const RigidBody = physZ.RigidBody;
const Vec2 = physZ.Vec2;

const arena_half_width = 12;
const arena_height = 8;

const drive_steps = 60;

// Torque lockstep: a 1×1 box at density 1 has m = 1 and I = m·(hx² + hy²)/3 = 1/6.
// A disc has I = ½·m·r² = ½·π·r⁴ at density 1, so this radius gives it the same I.
const lockstep_inertia = 1.0 / 6.0;
const lockstep_radius = std.math.pow(f32, 1.0 / (3.0 * std.math.pi), 0.25);
// Same torque on the same I for 1 s: both reach ω = 2π (one turn per second).
const lockstep_torque = lockstep_inertia * 2.0 * std.math.pi / (drive_steps / 60.0);

var lockstep_box: RigidBody.Id = undefined;
var lockstep_circle: RigidBody.Id = undefined;
var pushed_bar: RigidBody.Id = undefined;

pub fn main(init: std.process.Init) !void {
    var world = World.init(init.gpa, .{});
    defer world.deinit();

    _ = try world.createBody(.{
        .kind = .static,
        .shape = .{ .box = .{ .half_extents = Vec2.init(2.3, 0.1) } },
        .position = Vec2.init(-8.5, 2.5),
        .angle = -0.5,
    });
    _ = try world.createBody(.{
        .shape = .{ .circle = .{ .radius = 0.4 } },
        .position = Vec2.init(-9.8, 4.3),
    });
    _ = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.3, 0.3) } },
        .position = Vec2.init(-8.3, 3.2),
        .angle = -0.5,
    });

    for ([_]f32{ 0, 0.5, 0.9 }, 0..) |e, i| {
        _ = try world.createBody(.{
            .shape = .{ .circle = .{ .radius = 0.35 } },
            .position = Vec2.init(-5.5 + @as(f32, @floatFromInt(i)), 6),
            .restitution = e,
        });
    }

    pushed_bar = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.8, 0.1) } },
        .position = Vec2.init(-1.8, 0.1),
    });

    _ = try world.createBody(.{
        .shape = .{ .circle = .{ .radius = 1 } },
        .position = Vec2.init(0.5, 4),
    });
    _ = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.75, 0.75) } },
        .position = Vec2.init(3, 4),
    });
    _ = try world.createBody(.{
        .shape = .{ .circle = .{ .radius = 0.5 } },
        .position = Vec2.init(3.2, 5.6),
    });

    _ = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.6, 0.6) } },
        .position = Vec2.init(5.2, 3),
        .angle = 0.7,
    });

    _ = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.8, 0.15) } },
        .position = Vec2.init(6.8, 6),
        .angular_velocity = 5,
    });
    _ = try world.createBody(.{
        .shape = .{ .circle = .{ .radius = 0.4 } },
        .position = Vec2.init(8.6, 6.5),
        .angular_velocity = -3,
    });

    _ = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.5, 0.5) } },
        .position = Vec2.init(8, 0.5),
    });
    _ = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.9, 0.1) } },
        .position = Vec2.init(8.7, 1.1),
    });

    lockstep_box = try world.createBody(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(0.5, 0.5) } },
        .position = Vec2.init(10.2, 0.5),
    });
    lockstep_circle = try world.createBody(.{
        .shape = .{ .circle = .{ .radius = lockstep_radius } },
        .position = Vec2.init(11.3, lockstep_radius),
    });

    const half_t = 0.1;
    _ = try world.createBody(.{
        .kind = .static,
        .shape = .{ .box = .{ .half_extents = Vec2.init(arena_half_width + 2 * half_t, half_t) } },
        .position = Vec2.init(0, -half_t),
        .restitution = 1,
    });
    _ = try world.createBody(.{
        .kind = .static,
        .shape = .{ .box = .{ .half_extents = Vec2.init(arena_half_width + 2 * half_t, half_t) } },
        .position = Vec2.init(0, arena_height + half_t),
        .restitution = 1,
    });
    _ = try world.createBody(.{
        .kind = .static,
        .shape = .{ .box = .{ .half_extents = Vec2.init(half_t, arena_height / 2) } },
        .position = Vec2.init(-arena_half_width - half_t, arena_height / 2),
        .restitution = 1,
    });
    _ = try world.createBody(.{
        .kind = .static,
        .shape = .{ .box = .{ .half_extents = Vec2.init(half_t, arena_height / 2) } },
        .position = Vec2.init(arena_half_width + half_t, arena_height / 2),
        .restitution = 1,
    });

    try visualize.run(&world, .{ .title = "physZ - sandbox", .on_step = drive });
}

fn drive(world: *World, step: u64) void {
    if (step >= drive_steps) return;

    world.getBodyMut(lockstep_box).applyTorque(lockstep_torque);
    world.getBodyMut(lockstep_circle).applyTorque(lockstep_torque);

    const bar = world.getBodyMut(pushed_bar);
    bar.applyForceAt(Vec2.init(0, 1), bar.position.add(Vec2.init(0.8, 0)));
}
