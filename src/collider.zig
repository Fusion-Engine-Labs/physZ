const std = @import("std");

const Vec2 = @import("math/vec2.zig");
const RigidBody = @import("rigid_body.zig");

pub const Contact = struct {
    a: *RigidBody,
    b: *RigidBody,
    m: Manifold,
};

pub const Manifold = struct {
    normal: Vec2,
    depth: f32,
};

pub fn collide(a: RigidBody, b: RigidBody) ?Manifold {
    return switch (a.shape) {
        .box => |ab| switch (b.shape) {
            .box => |bb| boxBox(a.position, ab.half_extents, b.position, bb.half_extents),
            .circle => |bc| circleBox(b.position, bc.radius, a.position, ab.half_extents),
        },
        .circle => |ac| switch (b.shape) {
            .box => |bb| flip(circleBox(a.position, ac.radius, b.position, bb.half_extents)),
            .circle => |bc| circleCircle(a.position, ac.radius, b.position, bc.radius),
        },
    };
}

fn flip(m: ?Manifold) ?Manifold {
    const v = m orelse return null;
    return .{ .normal = v.normal.negate(), .depth = v.depth };
}

fn boxBox(pa: Vec2, ha: Vec2, pb: Vec2, hb: Vec2) ?Manifold {
    const d = pb.sub(pa);
    const ox = ha.x + hb.x - @abs(d.x);
    const oy = ha.y + hb.y - @abs(d.y);

    if (ox <= 0 or oy <= 0) {
        return null;
    }

    if (ox < oy) {
        return .{ .normal = .init(direction(d.x), 0), .depth = ox };
    }

    return .{ .normal = .init(0, direction(d.y)), .depth = oy };
}

inline fn direction(x: f32) f32 {
    return if (x < 0) -1 else 1;
}

fn circleBox(c: Vec2, r: f32, pb: Vec2, hb: Vec2) ?Manifold {
    const local = c.sub(pb);
    const closest = Vec2.init(
        std.math.clamp(local.x, -hb.x, hb.x),
        std.math.clamp(local.y, -hb.y, hb.y),
    );

    const diff = local.sub(closest);
    const dist_sq = diff.lengthSquared();
    if (dist_sq >= r * r) {
        return null;
    }

    // centre is inside the box, push out through the nearest face.
    if (dist_sq == 0) {
        const fx = hb.x - @abs(local.x);
        const fy = hb.y - @abs(local.y);

        if (fx < fy) {
            return .{ .normal = .init(direction(local.x), 0), .depth = r + fx };
        }

        return .{ .normal = .init(0, direction(local.y)), .depth = r + fy };
    }

    const dist = @sqrt(dist_sq);
    return .{ .normal = diff.scale(1 / dist), .depth = r - dist };
}

fn circleCircle(pa: Vec2, ra: f32, pb: Vec2, rb: f32) ?Manifold {
    const d = pb.sub(pa);
    const dist_sq = d.lengthSquared();
    const r = ra + rb;
    if (dist_sq >= r * r) {
        return null;
    }

    if (dist_sq == 0) {
        return .{ .normal = .init(0, 1), .depth = r };
    }

    const dist = @sqrt(dist_sq);
    return .{ .normal = d.scale(1 / dist), .depth = r - dist };
}

const testing = std.testing;

fn box(x: f32, y: f32) RigidBody {
    return .init(.{ .shape = .{ .box = .{ .half_extents = .one } }, .position = .init(x, y) });
}

fn circle(x: f32, y: f32) RigidBody {
    return .init(.{ .shape = .{ .circle = .{ .radius = 1 } }, .position = .init(x, y) });
}

test "collide returns a manifold pointing from a to b for overlapping shapes" {
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 0.5 }, collide(box(0, 0), box(1.5, 0.5)).?);
    try testing.expectEqual(Manifold{ .normal = .init(0, -1), .depth = 0.5 }, collide(box(0, 0), box(0.5, -1.5)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 0.5 }, collide(box(0, 0), circle(1.5, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 0.5 }, collide(circle(0, 0), box(1.5, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 1 }, collide(circle(0, 0), circle(1, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(0, 1), .depth = 2 }, collide(circle(0, 0), circle(0, 0)).?);
}

test "collide pushes a circle whose centre is inside a box out through the nearest face" {
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 1.5 }, collide(box(0, 0), circle(0.5, 0.25)).?);
    try testing.expectEqual(Manifold{ .normal = .init(-1, 0), .depth = 1.5 }, collide(circle(0.5, 0.25), box(0, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(0, -1), .depth = 1.25 }, collide(box(0, 0), circle(0.5, -0.75)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 1 }, collide(box(0, 0), circle(1, 0)).?);
}

test "collide returns a unit normal when centres coincide" {
    try testing.expectEqual(Manifold{ .normal = .init(0, 1), .depth = 2 }, collide(box(0, 0), box(0, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(0, 1), .depth = 2 }, collide(box(0, 0), circle(0, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 1.5 }, collide(box(0, 0), box(0.5, 0)).?);
}

test "collide returns null for separated shapes" {
    try testing.expectEqual(null, collide(box(0, 0), box(3, 0)));
    try testing.expectEqual(null, collide(box(0, 0), circle(3, 0)));
    try testing.expectEqual(null, collide(circle(0, 0), box(3, 0)));
    try testing.expectEqual(null, collide(circle(0, 0), circle(3, 0)));
}
