# physZ
A physics engine written in Zig

## Running the examples

The examples live in `examples/` and are built from that directory:

```sh
cd examples
```

### Native

```sh
zig build run              # runs the default example (sandbox)
zig build run-sandbox      # runs a specific example by name
```

### Web

```sh
zig build run -Dtarget=wasm32-emscripten
```

To build without launching, then serve the output yourself:

```sh
zig build -Dtarget=wasm32-emscripten
python3 -m http.server -d zig-out/web 8080
# open http://localhost:8080/sandbox.html
```
