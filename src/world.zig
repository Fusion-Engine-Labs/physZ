const std = @import("std");

const RigidBody = @import("rigid_body.zig");
const resolver = @import("resolver.zig");
const collider = @import("collider.zig");
const Vec2 = @import("math/vec2.zig");

const World = @This();

allocator: std.mem.Allocator,
bodies: std.ArrayList(RigidBody),
gravity: f32,
// How many times per step to run solver
velocity_iterations: u32,

pub const InitOptions = struct {
    gravity: f32 = -9.81,
    velocity_iterations: u32 = 8,
};

pub fn init(allocator: std.mem.Allocator, opts: InitOptions) World {
    return .{
        .allocator = allocator,
        .bodies = .empty,
        .gravity = opts.gravity,
        .velocity_iterations = opts.velocity_iterations,
    };
}

pub fn deinit(world: *World) void {
    world.bodies.deinit(world.allocator);
    world.* = undefined;
}

pub fn step(world: *World, dt: f32) !void {
    for (world.bodies.items) |*b| {
        if (b.kind == .static) {
            continue;
        }

        // linear
        b.velocity.y += world.gravity * dt;
        b.velocity = b.velocity.add(b.force.scale(b.inv_mass * dt));
        b.position = b.position.add(b.velocity.scale(dt));

        // angular
        b.angular_velocity += b.torque * b.inv_inertia * dt;
        b.angle += b.angular_velocity * dt;

        // resets
        b.force = .zero;
        b.torque = 0;
    }

    var contacts: std.ArrayList(collider.Contact) = .empty;
    defer contacts.deinit(world.allocator);

    for (world.bodies.items, 0..) |*a, i| {
        for (world.bodies.items[i + 1 ..]) |*b| {
            if (a.inv_mass + b.inv_mass == 0) {
                continue;
            }

            if (collider.collide(a.*, b.*)) |m| {
                try contacts.append(world.allocator, .{ .a = a, .b = b, .m = m });
            }
        }
    }

    for (0..world.velocity_iterations) |_| {
        for (contacts.items) |c| {
            resolver.resolveVelocity(&c);
        }
    }

    for (contacts.items) |c| {
        resolver.correctPosition(&c);
    }
}

pub fn createBody(world: *World, opts: RigidBody.InitOptions) !RigidBody.Id {
    const index: u32 = @intCast(world.bodies.items.len);
    try world.bodies.append(world.allocator, RigidBody.init(opts));

    return .{ .index = index };
}

pub fn getBody(world: *const World, id: RigidBody.Id) RigidBody {
    return world.bodies.items[id.index];
}

pub fn getBodyMut(world: *World, id: RigidBody.Id) *RigidBody {
    return &world.bodies.items[id.index];
}

const testing = std.testing;

test "init applies default options" {
    var world = World.init(testing.allocator, .{});
    defer world.deinit();

    try testing.expectEqual(@as(f32, -9.81), world.gravity);
    try testing.expectEqual(@as(u32, 8), world.velocity_iterations);
    try testing.expectEqual(@as(usize, 0), world.bodies.items.len);
}

test "init respects custom options" {
    var world = World.init(testing.allocator, .{ .gravity = -1, .velocity_iterations = 2 });
    defer world.deinit();

    try testing.expectEqual(@as(f32, -1), world.gravity);
    try testing.expectEqual(@as(u32, 2), world.velocity_iterations);
}

test "createBody returns sequential ids" {
    var world = World.init(testing.allocator, .{});
    defer world.deinit();

    const a = try world.createBody(.{ .shape = .{ .circle = .{ .radius = 1 } } });
    const b = try world.createBody(.{ .shape = .{ .circle = .{ .radius = 2 } } });

    try testing.expectEqual(@as(usize, 0), a.index);
    try testing.expectEqual(@as(usize, 1), b.index);
    try testing.expectEqual(@as(usize, 2), world.bodies.items.len);
}

test "getBody returns the body created with the given options" {
    var world = World.init(testing.allocator, .{});
    defer world.deinit();

    const id = try world.createBody(.{
        .kind = .static,
        .shape = .{ .circle = .{ .radius = 1 } },
        .position = Vec2.init(3, 4),
    });

    const body = world.getBody(id);
    try testing.expectEqual(RigidBody.Kind.static, body.kind);
    try testing.expectEqual(@as(f32, 3), body.position.x);
    try testing.expectEqual(@as(f32, 4), body.position.y);
}

test "getBodyMut allows modifying a body in place" {
    var world = World.init(testing.allocator, .{});
    defer world.deinit();

    const id = try world.createBody(.{ .shape = .{ .circle = .{ .radius = 1 } } });
    world.getBodyMut(id).velocity = Vec2.init(1, 2);

    try testing.expectEqual(@as(f32, 1), world.getBody(id).velocity.x);
    try testing.expectEqual(@as(f32, 2), world.getBody(id).velocity.y);
}

test "step integrates dynamic bodies and resolves contacts against static ones" {
    var world = World.init(testing.allocator, .{ .gravity = -10 });
    defer world.deinit();

    const floor = try world.createBody(.{ .kind = .static, .shape = .{ .box = .{ .half_extents = .one } } });
    _ = try world.createBody(.{ .kind = .static, .shape = .{ .box = .{ .half_extents = .one } } });
    const ball = try world.createBody(.{ .shape = .{ .circle = .{ .radius = 1 } }, .position = .init(0, 1.9) });
    world.getBodyMut(ball).applyForce(.init(1, 0));

    try world.step(0.1);

    const body = world.getBody(ball);
    try testing.expectEqual(Vec2.zero, world.getBody(floor).position);
    try testing.expectEqual(Vec2.zero, body.force);
    try testing.expect(body.velocity.x > 0);
    try testing.expectApproxEqAbs(@as(f32, 0.2), body.velocity.y, 1e-5);
    try testing.expect(body.position.y > 1.8);
}

test "step integrates torque into spin for boxes and circles" {
    var world = World.init(testing.allocator, .{ .gravity = 0 });
    defer world.deinit();

    // Unit box at density 1: I = 1/6. Unit circle: I = π/2.
    const box = try world.createBody(.{ .shape = .{ .box = .{ .half_extents = .init(0.5, 0.5) } } });
    const ball = try world.createBody(.{ .shape = .{ .circle = .{ .radius = 1 } }, .position = .init(10, 0), .angular_velocity = 1 });

    const b = world.getBodyMut(box);
    b.applyTorque(1);
    b.applyForceAt(.init(0, 1), .init(1, 0));
    world.getBodyMut(ball).applyTorque(std.math.pi);

    try world.step(0.5);

    try testing.expectApproxEqAbs(@as(f32, 6), world.getBody(box).angular_velocity, 1e-5);
    try testing.expectApproxEqAbs(@as(f32, 3), world.getBody(box).angle, 1e-5);
    try testing.expectEqual(@as(f32, 0.5), world.getBody(box).velocity.y);
    try testing.expectEqual(@as(f32, 0), world.getBody(box).torque);
    try testing.expectApproxEqAbs(@as(f32, 2), world.getBody(ball).angular_velocity, 1e-5);
}
