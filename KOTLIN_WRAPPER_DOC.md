# Kotlin Wrapper Integration Guide for CesiumRS Headless Renderer

Since the CesiumRS crate outputs a standard dynamic C library (`cdylib`), you can easily call it from Kotlin using Java Native Access (JNA) or JNI. JNA is recommended as it requires zero C/C++ boilerplate.

---

## 1. Build the Shared Library

Compile the Rust project to produce the shared library file:

```bash
cargo build --release
```

Depending on your OS, this will produce:
- **Linux:** `target/release/libcesium_rs.so`
- **macOS:** `target/release/libcesium_rs.dylib`
- **Windows:** `target/release/cesium_rs.dll`

---

## 2. Kotlin Project Setup

Add the JNA dependency to your `build.gradle.kts`:

```kotlin
dependencies {
    implementation("net.java.dev.jna:jna:5.14.0")
}
```

Place the compiled shared library in your project's search path, or configure the JVM system property `jna.library.path` to point to `target/release/`.

---

## 3. Kotlin Bindings (JNA)

Create the Kotlin mappings matching the `#[repr(C)]` structs and `extern "C"` function from `src/headless/api.rs`:

```kotlin
import com.sun.jna.Structure
import com.sun.jna.Library
import com.sun.jna.Native

// Equivalent to Rust's LatLon
@Structure.FieldOrder("lat", "lon")
open class LatLon : Structure() {
    @JvmField var lat: Double = 0.0
    @JvmField var lon: Double = 0.0

    class ByValue : LatLon(), Structure.ByValue
}

// Equivalent to Rust's HeadlessRoute
@Structure.FieldOrder("start", "end")
open class HeadlessRoute : Structure() {
    @JvmField var start: LatLon = LatLon()
    @JvmField var end: LatLon = LatLon()

    class ByValue : HeadlessRoute(), Structure.ByValue
}

// Interface to load the shared library
interface CesiumRSLibrary : Library {
    companion object {
        val INSTANCE: CesiumRSLibrary = Native.load("cesium_rs", CesiumRSLibrary::class.java)
    }

    fun render_routes_headless(
        width: Int,
        height: Int,
        routes: Array<HeadlessRoute>,
        routesCount: Long,
        outPath: String
    ): Boolean
}
```

---

## 4. Usage Example

Here is how you can use the loaded library from Kotlin:

```kotlin
fun main() {
    // 1. Define routes originating from a hub (e.g., Dubai - DXB)
    val dxb = LatLon().apply { lat = 25.2532; lon = 55.3657 }
    
    val routes = arrayOf(
        HeadlessRoute().apply {
            start = dxb
            end = LatLon().apply { lat = 40.6413; lon = -73.7781 } // JFK
        },
        HeadlessRoute().apply {
            start = dxb
            end = LatLon().apply { lat = 51.4700; lon = -0.4543 }  // LHR
        },
        HeadlessRoute().apply {
            start = dxb
            end = LatLon().apply { lat = -33.9399; lon = 151.1753 } // SYD
        },
        HeadlessRoute().apply {
            start = dxb
            end = LatLon().apply { lat = 35.7720; lon = 140.3929 }  // NRT
        }
    )

    // 2. Call the renderer
    val outputFilePath = "routes_export.png"
    println("Triggering route rendering to $outputFilePath...")
    
    val success = CesiumRSLibrary.INSTANCE.render_routes_headless(
        width = 1920,
        height = 1080,
        routes = routes,
        routesCount = routes.size.toLong(),
        outPath = outputFilePath
    )

    if (success) {
        println("Successfully generated dynamic routes map!")
    } else {
        println("Failed to render routes. Check application logs for details.")
    }
}
```
