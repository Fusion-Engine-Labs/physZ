const std = @import("std");

const Vec2 = @import("math/vec2.zig");

pub const Circle = struct {
    radius: f32,
};

pub const Box = struct {
    half_extents: Vec2,
};

pub const Shape = union(enum) {
    circle: Circle,
    box: Box,

    pub fn area(shape: Shape) f32 {
        return switch (shape) {
            .circle => |c| std.math.pi * c.radius * c.radius,
            .box => |b| 4 * b.half_extents.x * b.half_extents.y,
        };
    }
};

const testing = std.testing;

test "area" {
    const circle: Shape = .{ .circle = .{ .radius = 2 } };
    const box: Shape = .{ .box = .{ .half_extents = Vec2.init(1, 3) } };

    try testing.expectApproxEqAbs(@as(f32, 4 * std.math.pi), circle.area(), 1e-5);
    try testing.expectEqual(@as(f32, 12), box.area());
}
