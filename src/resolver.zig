const Manifold = @import("collider.zig").Manifold;
const RigidBody = @import("rigid_body.zig");

pub fn resolve(a: *RigidBody, b: *RigidBody, m: Manifold) void {
    const inv_sum = a.inv_mass + b.inv_mass;

    const vn = b.velocity.sub(a.velocity).dot(m.normal);
    if (vn < 0) {
        const e = @min(a.restitution, b.restitution);
        const j = -(1 + e) * vn / inv_sum;
        a.velocity = a.velocity.sub(m.normal.scale(j * a.inv_mass));
        b.velocity = b.velocity.sub(m.normal.scale(j * b.inv_mass));
    }

    const slop = 0.01;
    const percent = 0.8;
    const corr = m.normal.scale(@max(m.depth - slop, 0) / inv_sum * percent);
    a.position = a.position.sub(corr.scale(a.inv_mass));
    b.position = b.position.sub(corr.scale(b.inv_mass));
}
