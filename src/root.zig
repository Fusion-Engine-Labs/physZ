//! physZ: a physics library for Zig.

pub const RigidBody = @import("rigid_body.zig");
pub const resolver = @import("resolver.zig");
pub const collider = @import("collider.zig");
pub const Vec2 = @import("math/vec2.zig");
pub const World = @import("world.zig");
pub const Shape = @import("shape.zig");

test {
    @import("std").testing.refAllDecls(@This());

    _ = @import("rigid_body.zig");
    _ = @import("math/vec2.zig");
    _ = @import("math/rot.zig");
    _ = @import("resolver.zig");
    _ = @import("collider.zig");
    _ = @import("world.zig");
    _ = @import("shape.zig");
}
