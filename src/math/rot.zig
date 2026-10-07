const std = @import("std");
const math = std.math;

const Vec2 = @import("vec2.zig");

const Rot = @This();

sin: f32,
cos: f32,

pub fn fromAngle(angle: f32) Rot {
    return .{
        .sin = math.sin(angle),
        .cos = math.cos(angle),
    };
}

pub fn apply(self: Rot, v: Vec2) Vec2 {
    return .{
        .x = self.cos * v.x - self.sin * v.y,
        .y = self.sin * v.x + self.cos * v.y,
    };
}

pub fn applyInv(self: Rot, v: Vec2) Vec2 {
    return .{
        .x = self.cos * v.x + self.sin * v.y,
        .y = -self.sin * v.x + self.cos * v.y,
    };
}

test "fromAngle, apply, applyInv" {
    const r = fromAngle(math.atan2(@as(f32, 4), 3));
    const v = Vec2.init(1, 2);
    try std.testing.expectApproxEqAbs(-1, r.apply(v).x, 1e-6);
    try std.testing.expectApproxEqAbs(2, r.apply(v).y, 1e-6);
    try std.testing.expectApproxEqAbs(2.2, r.applyInv(v).x, 1e-6);
    try std.testing.expectApproxEqAbs(0.4, r.applyInv(v).y, 1e-6);
}
