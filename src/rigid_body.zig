const std = @import("std");

const Shape = @import("shape.zig").Shape;
const Vec2 = @import("math/vec2.zig");

const RigidBody = @This();

pub const Kind = enum {
    static,
    dynamic,
};

pub const Id = struct {
    index: usize,
};

kind: Kind,
shape: Shape,
position: Vec2,
velocity: Vec2,
force: Vec2,
inv_mass: f32,
restitution: f32,

pub const InitOptions = struct {
    kind: Kind = .dynamic,
    shape: Shape,
    position: Vec2 = .zero,
    velocity: Vec2 = .zero,
    density: f32 = 1,
    restitution: f32 = 0.2,
};

pub fn init(opts: InitOptions) RigidBody {
    const inv_mass: f32 = switch (opts.kind) {
        .static => 0,
        .dynamic => 1 / (opts.shape.area() * opts.density),
    };

    return .{
        .kind = opts.kind,
        .shape = opts.shape,
        .position = opts.position,
        .velocity = opts.velocity,
        .force = .zero,
        .inv_mass = inv_mass,
        .restitution = opts.restitution,
    };
}

pub fn applyForce(body: *RigidBody, force: Vec2) void {
    body.force = body.force.add(force);
}

const testing = std.testing;

test "dynamic body mass comes from area times density" {
    const body = RigidBody.init(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(1, 1) } },
        .density = 2,
    });

    // Area 4, density 2, mass 8.
    try testing.expectEqual(@as(f32, 1.0 / 8.0), body.inv_mass);
}

test "static body has zero inverse mass" {
    const body = RigidBody.init(.{
        .kind = .static,
        .shape = .{ .circle = .{ .radius = 1 } },
    });

    try testing.expectEqual(@as(f32, 0), body.inv_mass);
}
