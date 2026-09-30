const std = @import("std");

const Vec2 = @import("math/vec2.zig");
const RigidBody = @import("rigid_body.zig");

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
            .circle => null,
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
        return .{ .normal = Vec2.init(std.math.sign(d.x), 0), .depth = ox };
    }

    return .{ .normal = Vec2.init(0, std.math.sign(d.y)), .depth = oy };
}

fn circleBox(c: Vec2, r: f32, pb: Vec2, hb: Vec2) ?Manifold {
    const local = c.sub(pb);
    const closest = pb.add(Vec2.init(
        std.math.clamp(local.x, -hb.x, hb.x),
        std.math.clamp(local.y, -hb.y, hb.y),
    ));

    const diff = c.sub(closest);
    const dist_sq = diff.lengthSquared();
    if (dist_sq >= r * r or dist_sq == 0) {
        return null;
    }

    const dist = @sqrt(dist_sq);
    return .{ .normal = diff.scale(1 / dist), .depth = r - dist };
}
