const std = @import("std");

const RigidBody = @import("rigid_body.zig");

const World = @This();

allocator: std.mem.Allocator,
bodies: std.ArrayList(RigidBody),
gravity: f32,
// How many times per step to run solver
velocity_iterations: u32,

pub const Options = struct {
    gravity: f32 = -9.81,
    velocity_iterations: u32 = 8,
};

pub fn init(allocator: std.mem.Allocator, options: Options) World {
    return .{
        .allocator = allocator,
        .bodies = .empty,
        .gravity = options.gravity,
        .velocity_iterations = options.velocity_iterations,
    };
}

pub fn deinit(world: *World) void {
    world.bodies.deinit(world.allocator);
    world.* = undefined;
}

pub fn step(world: *World, dt: f32) !void {
    _ = dt;
    _ = world;
}
