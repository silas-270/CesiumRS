# Kotlin integration

CesiumRS builds as a `cdylib` (`libcesium_rs.so`) and exposes two independent surfaces to
the JVM:

| Surface | Mechanism | Source | Use it for |
|---|---|---|---|
| **Headless** | C ABI, called through [JNA](https://github.com/java-native-access/jna) | `src/headless/api.rs` | Rendering a static globe with route arcs to a PNG — no window, no surface |
| **Live** | JNI, `Java_…` exports | `src/android_jni.rs` | The interactive flight view inside an Android `GameActivity` |

[Blocktime](https://github.com/silas-270/Blocktime) uses both and is the reference
integration. Its `docs/engine.md` documents the live bridge call by call; this page covers
building the library and the headless API.

## Building the library

**Desktop** (for JVM tests or tools):

```bash
cargo build --release
# → target/release/libcesium_rs.so   (Linux)
#   target/release/libcesium_rs.dylib (macOS)
#   target/release/cesium_rs.dll      (Windows)
```

Point JNA at it with `-Djna.library.path=target/release`.

**Android** needs [`cargo-ndk`](https://github.com/bbqsrc/cargo-ndk) and an NDK:

```bash
cargo ndk -t arm64-v8a -o <app>/src/main/jniLibs build --release \
    --no-default-features
```

The default features include the desktop test harnesses and the egui debug panel, which an
app does not want. City labels are drawn by the engine's own pipeline and need no feature;
`debug_panel` adds only egui, whose window is not shown on Android. Add `perf_trace` for
ATrace spans in a profiling build (see [architecture.md](architecture.md#cargo-features)).

## The headless API

Every headless render uses the bundled Natural Earth vector map, rasterised on the CPU, so
it never touches the network and looks the same on- and offline. Each call blocks until
the PNG is written and returns `true` on success.

```rust
#[repr(C)] pub struct LatLon        { pub lat: f64, pub lon: f64 }
#[repr(C)] pub struct HeadlessRoute { pub start: LatLon, pub end: LatLon }

// Default framing: the horizon view below with altitude 1.0, back 11°, pitch 46°.
extern "C" fn render_routes_headless(
    width: u32, height: u32,
    routes: *const HeadlessRoute, routes_count: usize,
    out_path: *const c_char,
) -> bool;

// Camera on a sphere around the first route's start: distance from the Earth's centre
// in megametres (the engine's unit; Earth's radius is ~6.378), tilt towards north and
// pan towards east, in degrees.
extern "C" fn render_routes_headless_custom(
    width: u32, height: u32,
    routes: *const HeadlessRoute, routes_count: usize,
    out_path: *const c_char,
    distance: f32, tilt_deg: f32, pan_deg: f32,
) -> bool;

// An "over the horizon" view: `altitude` megametres above the surface, pulled back
// `back_deg` of arc from the first route's start, then oriented by pitch (down from the
// horizon), heading and roll.
extern "C" fn render_routes_headless_horizon(
    width: u32, height: u32,
    routes: *const HeadlessRoute, routes_count: usize,
    out_path: *const c_char,
    altitude: f32, back_deg: f32, pitch_deg: f32, heading_deg: f32, roll_deg: f32,
) -> bool;
```

The first route's `start` is treated as the hub the camera frames.

## Kotlin bindings (JNA)

```kotlin
dependencies {
    implementation("net.java.dev.jna:jna:5.19.1@aar") // plain "jna:5.19.1" on desktop
}
```

```kotlin
import com.sun.jna.Library
import com.sun.jna.Native
import com.sun.jna.Structure

@Structure.FieldOrder("lat", "lon")
open class LatLon : Structure() {
    @JvmField var lat: Double = 0.0
    @JvmField var lon: Double = 0.0
    class ByValue : LatLon(), Structure.ByValue
}

@Structure.FieldOrder("start", "end")
open class HeadlessRoute : Structure() {
    @JvmField var start: LatLon.ByValue = LatLon.ByValue()
    @JvmField var end: LatLon.ByValue = LatLon.ByValue()
}

interface CesiumRs : Library {
    fun render_routes_headless(
        width: Int, height: Int,
        routes: HeadlessRoute, routesCount: Long,
        outPath: String,
    ): Boolean

    companion object {
        val INSTANCE: CesiumRs by lazy { Native.load("cesium_rs", CesiumRs::class.java) }
    }
}
```

### Passing the route array

Rust reads `routes` as one contiguous block of `routes_count` structs. A Kotlin
`arrayOf(HeadlessRoute(), …)` is *not* contiguous — each element is its own allocation.
Allocate the block with `Structure.toArray`, fill it, `write()` each element, and pass the
**first** element:

```kotlin
@Suppress("UNCHECKED_CAST")
fun renderRoutes(routes: List<Pair<LatLon, LatLon>>, outPath: String): Boolean {
    if (routes.isEmpty()) return false
    val block = HeadlessRoute().toArray(routes.size) as Array<HeadlessRoute>
    routes.forEachIndexed { i, (from, to) ->
        block[i].start.lat = from.lat; block[i].start.lon = from.lon
        block[i].end.lat = to.lat;     block[i].end.lon = to.lon
        block[i].write()
    }
    return CesiumRs.INSTANCE.render_routes_headless(
        1920, 1080, block[0], routes.size.toLong(), outPath,
    )
}
```

Calls block for the length of a render, so keep them off the main thread.

## The live bridge

The JNI exports in `src/android_jni.rs` are named for Blocktime's
`com.silas270.blocktime.engine.live.CesiumLiveJniBridge`. Another app has to either use the
same class name or rename the exports. The engine runs on the `GameActivity` it is
launched into (`android_main` in `src/lib.rs`); the JNI calls set the pending flight,
progress, camera mode, map style and terrain, and read telemetry back.
