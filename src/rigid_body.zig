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
torque: f32,
angle: f32,
inv_mass: f32,
restitution: f32,
angular_velocity: f32,
inv_inertia: f32,

pub const InitOptions = struct {
    kind: Kind = .dynamic,
    shape: Shape,
    position: Vec2 = .zero,
    velocity: Vec2 = .zero,
    density: f32 = 1,
    restitution: f32 = 0.2,
    angular_velocity: f32 = 0,
    angle: f32 = 0,
};

pub fn init(opts: InitOptions) RigidBody {
    const inv_mass: f32 = switch (opts.kind) {
        .static => 0,
        .dynamic => 1 / (opts.shape.area() * opts.density),
    };

    const inv_inertia: f32 = switch (opts.kind) {
        .static => 0,
        .dynamic => 1 / opts.shape.inertia(opts.shape.area() * opts.density),
    };

    return .{
        .kind = opts.kind,
        .shape = opts.shape,
        .position = opts.position,
        .velocity = opts.velocity,
        .force = .zero,
        .torque = 0,
        .angle = opts.angle,
        .inv_mass = inv_mass,
        .restitution = opts.restitution,
        .angular_velocity = opts.angular_velocity,
        .inv_inertia = inv_inertia,
    };
}

pub fn applyForce(body: *RigidBody, force: Vec2) void {
    body.force = body.force.add(force);
}

pub fn applyForceAt(body: *RigidBody, force: Vec2, point: Vec2) void {
    const r = point.sub(body.position);

    body.force = body.force.add(force);
    body.torque += r.cross(force);
}

pub fn applyTorque(body: *RigidBody, torque: f32) void {
    body.torque += torque;
}

const testing = std.testing;

test "dynamic body mass comes from area times density" {
    const body = RigidBody.init(.{
        .shape = .{ .box = .{ .half_extents = Vec2.init(1, 1) } },
        .density = 2,
    });

    try testing.expectEqual(@as(f32, 1.0 / 8.0), body.inv_mass);
}

test "static body has zero inverse mass" {
    const body = RigidBody.init(.{
        .kind = .static,
        .shape = .{ .circle = .{ .radius = 1 } },
    });

    try testing.expectEqual(@as(f32, 0), body.inv_mass);
    try testing.expectEqual(@as(f32, 0), body.inv_inertia);
}
