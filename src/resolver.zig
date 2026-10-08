const Contact = @import("collider.zig").Contact;
const RigidBody = @import("rigid_body.zig");
const Vec2 = @import("math/vec2.zig");

const Basis = struct {
    ra: Vec2,
    rb: Vec2,
    ra_n: f32,
    rb_n: f32,
    k: f32,
};

fn basis(contact: *const Contact) Basis {
    const n = contact.m.normal;
    const ra = contact.m.point.sub(contact.a.position);
    const rb = contact.m.point.sub(contact.b.position);
    const ra_n = ra.cross(n);
    const rb_n = rb.cross(n);
    return .{
        .ra = ra,
        .rb = rb,
        .ra_n = ra_n,
        .rb_n = rb_n,
        .k = contact.a.inv_mass + contact.b.inv_mass + ra_n * ra_n * contact.a.inv_inertia + rb_n * rb_n * contact.b.inv_inertia,
    };
}

pub fn resolveVelocity(contact: *const Contact) void {
    const b = basis(contact);
    const n = contact.m.normal;
    const va = contact.a.velocity.add(Vec2.crossSV(contact.a.angular_velocity, b.ra));
    const vb = contact.b.velocity.add(Vec2.crossSV(contact.b.angular_velocity, b.rb));
    const vn = vb.sub(va).dot(n);
    if (vn < 0) {
        const e = @min(contact.a.restitution, contact.b.restitution);
        const j = -(1 + e) * vn / b.k;
        contact.a.velocity = contact.a.velocity.sub(n.scale(j * contact.a.inv_mass));
        contact.b.velocity = contact.b.velocity.add(n.scale(j * contact.b.inv_mass));
        contact.a.angular_velocity -= b.ra_n * j * contact.a.inv_inertia;
        contact.b.angular_velocity += b.rb_n * j * contact.b.inv_inertia;
    }
}

pub fn correctPosition(contact: *const Contact) void {
    const b = basis(contact);
    const n = contact.m.normal;

    const slop = 0.01;
    const percent = 0.8;
    const mag = @max(contact.m.depth - slop, 0) * percent / b.k;
    const corr = n.scale(mag);
    contact.a.position = contact.a.position.sub(corr.scale(contact.a.inv_mass));
    contact.b.position = contact.b.position.add(corr.scale(contact.b.inv_mass));
    contact.a.angle -= b.ra_n * mag * contact.a.inv_inertia;
    contact.b.angle += b.rb_n * mag * contact.b.inv_inertia;
}

const testing = @import("std").testing;

test "resolveVelocity bounces approaching bodies and correctPosition separates them" {
    var a = RigidBody.init(.{ .shape = .{ .box = .{ .half_extents = .one } }, .velocity = .init(1, 0), .restitution = 1 });
    var b = RigidBody.init(.{ .shape = .{ .box = .{ .half_extents = .one } }, .velocity = .init(-1, 0), .restitution = 1 });
    const contact: Contact = .{ .a = &a, .b = &b, .m = .{ .normal = .init(1, 0), .depth = 0.51, .point = .zero } };

    for (0..2) |_| {
        resolveVelocity(&contact);
        try testing.expectEqual(@as(f32, -1), a.velocity.x);
        try testing.expectEqual(@as(f32, 1), b.velocity.x);
        try testing.expectEqual(@as(f32, 0), a.angular_velocity);
        try testing.expectEqual(@as(f32, 0), b.angular_velocity);
    }

    correctPosition(&contact);
    try testing.expectApproxEqAbs(@as(f32, -0.2), a.position.x, 1e-6);
    try testing.expectApproxEqAbs(@as(f32, 0.2), b.position.x, 1e-6);
    try testing.expectEqual(@as(f32, 0), a.angle);
    try testing.expectEqual(@as(f32, 0), b.angle);
}

test "resolveVelocity turns an offset hit into spin" {
    var box_body = RigidBody.init(.{
        .shape = .{ .box = .{ .half_extents = .one } },
        .velocity = .init(0, -1),
        .restitution = 1,
    });
    var floor = RigidBody.init(.{
        .kind = .static,
        .shape = .{ .box = .{ .half_extents = .one } },
        .restitution = 1,
    });
    const contact: Contact = .{
        .a = &box_body,
        .b = &floor,
        .m = .{ .normal = .init(0, -1), .depth = 0.1, .point = .init(0.5, -1) },
    };

    resolveVelocity(&contact);
    try testing.expectApproxEqAbs(@as(f32, 5.0 / 11.0), box_body.velocity.y, 1e-5);
    try testing.expectApproxEqAbs(@as(f32, 0), box_body.velocity.x, 1e-5);
    try testing.expectApproxEqAbs(@as(f32, 12.0 / 11.0), box_body.angular_velocity, 1e-5);
    try testing.expectEqual(@as(f32, 0), floor.angular_velocity);
}

test "resolveVelocity keeps circle spin when the contact lies along the normal" {
    var ball = RigidBody.init(.{
        .shape = .{ .circle = .{ .radius = 1 } },
        .velocity = .init(1, 0),
        .angular_velocity = 3,
        .restitution = 1,
    });
    var wall = RigidBody.init(.{
        .kind = .static,
        .shape = .{ .box = .{ .half_extents = .one } },
        .restitution = 1,
    });
    const contact: Contact = .{
        .a = &ball,
        .b = &wall,
        .m = .{ .normal = .init(1, 0), .depth = 0.1, .point = .init(1, 0) },
    };

    resolveVelocity(&contact);
    try testing.expectEqual(@as(f32, 3), ball.angular_velocity);
    try testing.expect(ball.velocity.x < 0);
}

test "correctPosition rotates an offset overlap out of the contact" {
    var box_body = RigidBody.init(.{ .shape = .{ .box = .{ .half_extents = .one } } });
    var floor = RigidBody.init(.{
        .kind = .static,
        .shape = .{ .box = .{ .half_extents = .one } },
    });
    const contact: Contact = .{
        .a = &box_body,
        .b = &floor,
        .m = .{ .normal = .init(0, -1), .depth = 0.51, .point = .init(0.5, -1) },
    };

    correctPosition(&contact);
    try testing.expectApproxEqAbs(@as(f32, 16.0 / 55.0), box_body.position.y, 1e-5);
    try testing.expectApproxEqAbs(@as(f32, 12.0 / 55.0), box_body.angle, 1e-5);
    try testing.expectEqual(@as(f32, 0), floor.position.x);
    try testing.expectEqual(@as(f32, 0), floor.angle);
}
