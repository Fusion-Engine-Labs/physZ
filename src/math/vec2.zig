const std = @import("std");

const Vec2 = @This();

x: f32,
y: f32,

pub const zero: Vec2 = .{ .x = 0, .y = 0 };
pub const one: Vec2 = .{ .x = 1, .y = 1 };

pub fn init(x: f32, y: f32) Vec2 {
    return .{ .x = x, .y = y };
}

pub fn add(a: Vec2, b: Vec2) Vec2 {
    return .{ .x = a.x + b.x, .y = a.y + b.y };
}

pub fn sub(a: Vec2, b: Vec2) Vec2 {
    return .{ .x = a.x - b.x, .y = a.y - b.y };
}

pub fn scale(v: Vec2, s: f32) Vec2 {
    return .{ .x = v.x * s, .y = v.y * s };
}

pub fn negate(v: Vec2) Vec2 {
    return .{ .x = -v.x, .y = -v.y };
}

pub fn dot(a: Vec2, b: Vec2) f32 {
    return a.x * b.x + a.y * b.y;
}

pub fn lengthSquared(v: Vec2) f32 {
    return v.dot(v);
}

pub fn length(v: Vec2) f32 {
    return std.math.sqrt(v.lengthSquared());
}

const testing = std.testing;

test "add, sub, scale, negate" {
    const a = Vec2.init(1, 2);
    const b = Vec2.init(3, -4);

    try testing.expectEqual(Vec2.init(4, -2), a.add(b));
    try testing.expectEqual(Vec2.init(-2, 6), a.sub(b));
    try testing.expectEqual(Vec2.init(2, 4), a.scale(2));
    try testing.expectEqual(Vec2.init(-1, -2), a.negate());
}

test "dot and length" {
    const v = Vec2.init(3, 4);

    try testing.expectEqual(@as(f32, 25), v.lengthSquared());
    try testing.expectEqual(@as(f32, 5), v.length());
    try testing.expectEqual(@as(f32, 0), Vec2.init(1, 0).dot(Vec2.init(0, 1)));
}
