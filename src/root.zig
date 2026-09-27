//! physZ: a physics library for Zig.

const Vec2 = @import("math/vec2.zig");

test {
    @import("std").testing.refAllDecls(@This());

    _ = @import("math/vec2.zig");
}
