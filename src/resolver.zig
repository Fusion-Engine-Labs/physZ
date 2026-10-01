const Contact = @import("collider.zig").Contact;
const RigidBody = @import("rigid_body.zig");

pub fn resolveVelocity(contact: *const Contact) void {
    const inv_sum = contact.a.inv_mass + contact.b.inv_mass;

    const vn = contact.b.velocity.sub(contact.a.velocity).dot(contact.m.normal);
    if (vn < 0) {
        const e = @min(contact.a.restitution, contact.b.restitution);
        const j = -(1 + e) * vn / inv_sum;
        contact.a.velocity = contact.a.velocity.sub(contact.m.normal.scale(j * contact.a.inv_mass));
        contact.b.velocity = contact.b.velocity.sub(contact.m.normal.scale(j * contact.b.inv_mass));
    }
}

pub fn correctPosition(contact: *const Contact) void {
    const inv_sum = contact.a.inv_mass + contact.b.inv_mass;

    const slop = 0.01;
    const percent = 0.8;
    const corr = contact.m.normal.scale(@max(contact.m.depth - slop, 0) / inv_sum * percent);
    contact.a.position = contact.a.position.sub(corr.scale(contact.a.inv_mass));
    contact.b.position = contact.b.position.add(corr.scale(contact.b.inv_mass));
}
