const std = @import("std");

const Vec2 = @import("math/vec2.zig");
const Rot = @import("math/rot.zig");
const RigidBody = @import("rigid_body.zig");

pub const Contact = struct {
    a: *RigidBody,
    b: *RigidBody,
    m: Manifold,
};

pub const Manifold = struct {
    normal: Vec2,
    depth: f32,
    point: Vec2,
};

pub fn collide(a: RigidBody, b: RigidBody) ?Manifold {
    return switch (a.shape) {
        .box => |ab| switch (b.shape) {
            .box => |bb| boxBox(a.position, a.angle, ab.half_extents, b.position, b.angle, bb.half_extents),
            .circle => |bc| circleBox(b.position, bc.radius, a.position, a.angle, ab.half_extents),
        },
        .circle => |ac| switch (b.shape) {
            .box => |bb| flip(circleBox(a.position, ac.radius, b.position, b.angle, bb.half_extents)),
            .circle => |bc| circleCircle(a.position, ac.radius, b.position, bc.radius),
        },
    };
}

fn flip(m: ?Manifold) ?Manifold {
    const v = m orelse return null;
    return .{ .normal = v.normal.negate(), .depth = v.depth, .point = v.point };
}

const Frame = struct {
    pos: Vec2,
    rot: Rot,
    half: Vec2,

    fn radius(self: Frame, axis: Vec2) f32 {
        const local = self.rot.applyInv(axis);
        return self.half.x * @abs(local.x) + self.half.y * @abs(local.y);
    }

    fn support(self: Frame, dir: Vec2) Vec2 {
        const local_dir = self.rot.applyInv(dir);
        const corners = [_]Vec2{
            .{ .x = -self.half.x, .y = -self.half.y },
            .{ .x = self.half.x, .y = -self.half.y },
            .{ .x = -self.half.x, .y = self.half.y },
            .{ .x = self.half.x, .y = self.half.y },
        };

        var best = corners[0].dot(local_dir);
        for (corners[1..]) |corner| best = @max(best, corner.dot(local_dir));

        var sum = Vec2.zero;
        var count: f32 = 0;
        for (corners) |corner| {
            if (corner.dot(local_dir) >= best - 1e-5) {
                sum = sum.add(corner);
                count += 1;
            }
        }
        return self.pos.add(self.rot.apply(sum.scale(1 / count)));
    }
};

fn boxBox(pa: Vec2, aa: f32, ha: Vec2, pb: Vec2, ab: f32, hb: Vec2) ?Manifold {
    const a = Frame{ .pos = pa, .rot = Rot.fromAngle(aa), .half = ha };
    const b = Frame{ .pos = pb, .rot = Rot.fromAngle(ab), .half = hb };
    const d = pb.sub(pa);

    // Y before X so equal overlaps keep the old upright result: a vertical normal.
    const axes = [_]struct { axis: Vec2, from_a: bool }{
        .{ .axis = a.rot.apply(.{ .x = 0, .y = 1 }), .from_a = true },
        .{ .axis = a.rot.apply(.{ .x = 1, .y = 0 }), .from_a = true },
        .{ .axis = b.rot.apply(.{ .x = 0, .y = 1 }), .from_a = false },
        .{ .axis = b.rot.apply(.{ .x = 1, .y = 0 }), .from_a = false },
    };

    var best_depth: f32 = std.math.inf(f32);
    var best_normal: Vec2 = undefined;
    var best_from_a = true;

    for (axes) |entry| {
        var axis = entry.axis;
        const dist = d.dot(axis);
        if (dist < 0) axis = axis.negate();

        const overlap = a.radius(axis) + b.radius(axis) - @abs(dist);
        if (overlap <= 0) return null;
        if (overlap < best_depth) {
            best_depth = overlap;
            best_normal = axis;
            best_from_a = entry.from_a;
        }
    }

    const point = if (best_from_a)
        facePoint(b, a, best_normal)
    else
        facePoint(a, b, best_normal.negate());

    return .{ .normal = best_normal, .depth = best_depth, .point = point };
}

fn facePoint(incident: Frame, reference: Frame, face_normal: Vec2) Vec2 {
    const vertex = incident.support(face_normal.negate());
    const t = vertex.sub(reference.pos).dot(face_normal) - reference.radius(face_normal);
    const on_plane = vertex.sub(face_normal.scale(t));

    const local = reference.rot.applyInv(on_plane.sub(reference.pos));
    const n_local = reference.rot.applyInv(face_normal);
    const clamped: Vec2 = if (@abs(n_local.x) > @abs(n_local.y))
        .{ .x = direction(n_local.x) * reference.half.x, .y = std.math.clamp(local.y, -reference.half.y, reference.half.y) }
    else
        .{ .x = std.math.clamp(local.x, -reference.half.x, reference.half.x), .y = direction(n_local.y) * reference.half.y };

    return reference.pos.add(reference.rot.apply(clamped));
}

inline fn direction(x: f32) f32 {
    return if (x < 0) -1 else 1;
}

fn circleBox(c: Vec2, r: f32, pb: Vec2, angle: f32, hb: Vec2) ?Manifold {
    const rot = Rot.fromAngle(angle);
    const local = rot.applyInv(c.sub(pb));
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
            const n = Vec2.init(direction(local.x), 0);
            return .{
                .normal = rot.apply(n),
                .depth = r + fx,
                .point = pb.add(rot.apply(.{ .x = n.x * hb.x, .y = local.y })),
            };
        }

        const n = Vec2.init(0, direction(local.y));
        return .{
            .normal = rot.apply(n),
            .depth = r + fy,
            .point = pb.add(rot.apply(.{ .x = local.x, .y = n.y * hb.y })),
        };
    }

    const dist = @sqrt(dist_sq);
    return .{
        .normal = rot.apply(diff.scale(1 / dist)),
        .depth = r - dist,
        .point = pb.add(rot.apply(closest)),
    };
}

fn circleCircle(pa: Vec2, ra: f32, pb: Vec2, rb: f32) ?Manifold {
    const d = pb.sub(pa);
    const dist_sq = d.lengthSquared();
    const r = ra + rb;
    if (dist_sq >= r * r) {
        return null;
    }

    const normal: Vec2 = if (dist_sq == 0)
        .init(0, 1)
    else
        d.scale(1 / @sqrt(dist_sq));
    const point = pa.add(normal.scale(ra)).add(pb.sub(normal.scale(rb))).scale(0.5);

    return .{ .normal = normal, .depth = r - @sqrt(dist_sq), .point = point };
}

const testing = std.testing;

fn box(x: f32, y: f32) RigidBody {
    return .init(.{ .shape = .{ .box = .{ .half_extents = .one } }, .position = .init(x, y) });
}

fn circle(x: f32, y: f32) RigidBody {
    return .init(.{ .shape = .{ .circle = .{ .radius = 1 } }, .position = .init(x, y) });
}

test "collide returns a manifold pointing from a to b for overlapping shapes" {
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 0.5, .point = .init(1, 0.5) }, collide(box(0, 0), box(1.5, 0.5)).?);
    try testing.expectEqual(Manifold{ .normal = .init(0, -1), .depth = 0.5, .point = .init(0.5, -1) }, collide(box(0, 0), box(0.5, -1.5)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 0.5, .point = .init(1, 0) }, collide(box(0, 0), circle(1.5, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 0.5, .point = .init(0.5, 0) }, collide(circle(0, 0), box(1.5, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 1, .point = .init(0.5, 0) }, collide(circle(0, 0), circle(1, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(0, 1), .depth = 2, .point = .zero }, collide(circle(0, 0), circle(0, 0)).?);
}

test "collide pushes a circle whose centre is inside a box out through the nearest face" {
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 1.5, .point = .init(1, 0.25) }, collide(box(0, 0), circle(0.5, 0.25)).?);
    try testing.expectEqual(Manifold{ .normal = .init(-1, 0), .depth = 1.5, .point = .init(1, 0.25) }, collide(circle(0.5, 0.25), box(0, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(0, -1), .depth = 1.25, .point = .init(0.5, -1) }, collide(box(0, 0), circle(0.5, -0.75)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 1, .point = .init(1, 0) }, collide(box(0, 0), circle(1, 0)).?);
}

test "collide returns a unit normal when centres coincide" {
    try testing.expectEqual(Manifold{ .normal = .init(0, 1), .depth = 2, .point = .init(0, 1) }, collide(box(0, 0), box(0, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(0, 1), .depth = 2, .point = .init(0, 1) }, collide(box(0, 0), circle(0, 0)).?);
    try testing.expectEqual(Manifold{ .normal = .init(1, 0), .depth = 1.5, .point = .init(1, 0) }, collide(box(0, 0), box(0.5, 0)).?);
}

test "collide uses box angle instead of the axis-aligned bounds" {
    const diamond = RigidBody.init(.{
        .shape = .{ .box = .{ .half_extents = .one } },
        .angle = std.math.pi / 4.0,
    });
    const upright = RigidBody.init(.{ .shape = .{ .box = .{ .half_extents = .one } } });
    const corner = RigidBody.init(.{
        .shape = .{ .box = .{ .half_extents = .init(0.05, 0.05) } },
        .position = .init(1, 1),
    });
    const beside_vertex = RigidBody.init(.{
        .shape = .{ .box = .{ .half_extents = .init(0.05, 0.05) } },
        .position = .init(1.2, 0),
    });

    try testing.expectEqual(null, collide(diamond, corner));
    try testing.expect(collide(diamond, beside_vertex) != null);
    try testing.expect(collide(upright, corner) != null);
    try testing.expectEqual(null, collide(upright, beside_vertex));
}

test "collide hits a rotated box on its oriented surface" {
    const plank = RigidBody.init(.{
        .shape = .{ .box = .{ .half_extents = .init(2, 0.1) } },
        .angle = std.math.pi / 2.0,
    });
    const clear = RigidBody.init(.{
        .shape = .{ .circle = .{ .radius = 0.5 } },
        .position = .init(1, 0),
    });
    const hit = RigidBody.init(.{
        .shape = .{ .circle = .{ .radius = 0.5 } },
        .position = .init(0.4, 0),
    });

    try testing.expectEqual(null, collide(plank, clear));

    const m = collide(plank, hit).?;
    try testing.expectApproxEqAbs(@as(f32, 1), m.normal.x, 1e-4);
    try testing.expectApproxEqAbs(@as(f32, 0), m.normal.y, 1e-4);
    try testing.expectApproxEqAbs(@as(f32, 0.2), m.depth, 1e-4);
    try testing.expectApproxEqAbs(@as(f32, 0.1), m.point.x, 1e-4);
    try testing.expectApproxEqAbs(@as(f32, 0), m.point.y, 1e-4);

    const flipped = collide(hit, plank).?;
    try testing.expectApproxEqAbs(@as(f32, -1), flipped.normal.x, 1e-4);
    try testing.expectApproxEqAbs(m.point.x, flipped.point.x, 1e-4);
    try testing.expectApproxEqAbs(m.point.y, flipped.point.y, 1e-4);
}

test "collide returns null for separated shapes" {
    try testing.expectEqual(null, collide(box(0, 0), box(3, 0)));
    try testing.expectEqual(null, collide(box(0, 0), circle(3, 0)));
    try testing.expectEqual(null, collide(circle(0, 0), box(3, 0)));
    try testing.expectEqual(null, collide(circle(0, 0), circle(3, 0)));
}
