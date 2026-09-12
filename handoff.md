# Handoff — 2026-09-11

Written so the next session can start cold without re-deriving anything. **The performance
test is ready to run right now; nothing needs building first.** Go straight to §1.

---

## 1. Run the performance test (start here)

The instrumented APK is **already installed on the phone** (`com.example.focusflight`,
Galaxy S23 / `RFCX20QDV0T`). Nothing needs rebuilding unless CesiumRS source has changed
since this handoff.

```sh
# 1. Plug the phone in, unlock it, accept the USB-debugging prompt.
adb devices          # expect: RFCX20QDV0T   device

# 2. On the phone: open Focusflight, start a flight, put it in COCKPIT view.
#    Leave it in the foreground, screen on.

# 3. From the CesiumRS repo root:
tools/phone_soak.sh 3600 30      # 1 hour, sampling every 30s
tools/phone_soak.sh              # or 10 minutes, every 20s (the default)
```

The script refuses to start if the app is not actually rendering, so a bad setup fails in
5 seconds instead of an hour. It samples frame-time percentiles, jank %, total PSS, native
heap, battery temperature and prime-core clock, then prints a first-third vs last-third
trend table. Artifacts land in `tools/soak_<HHMMSS>/` (`samples.csv`, `logcat.txt`).

**What the run is looking for** (none of this has been measured yet — it's the open
question):

| symptom in the trend table | means |
|---|---|
| p90/p99 frame time climbing | thermal throttling, or something degrading per-frame |
| total PSS / native heap climbing steadily | a leak |
| prime clock dropping, temp rising | throttling — expected if charging; note the charge state |
| jank % rising | the above, showing up where the user would feel it |

**Charging skews it.** Charging keeps the phone warm and makes throttling *more* likely.
That is the harsher test; for a fair one, run unplugged (but `adb` over USB charges
anyway — use `adb tcpip`/wireless debugging if you want a genuinely unplugged run).

### If the APK needs rebuilding

Only if CesiumRS source changed. **Never compile CesiumRS locally** (see the memory note
`build-with-cargo-remote`); build on `lxhalle` and copy the `.so` in:

```sh
HOST=kams@lxhalle.in.tum.de; RDIR=/var/tmp/kams_builds/4484484651198947054
cd ~/CesiumRS
tar czf - src crates Cargo.toml Cargo.lock | ssh "$HOST" "cd $RDIR && tar xzf -"

# profiling profile == release codegen + debug symbols; perf_trace exports the
# nativeRunPerfScenario JNI hook that run_perf_scenario.sh needs.
ssh "$HOST" "cd $RDIR && source ~/.cargo/env; \
  export ANDROID_NDK_HOME=~/android-ndk-setup/android-ndk-r27b; \
  cargo ndk --target aarch64-linux-android build --lib --profile profiling \
    --no-default-features --features debug_panel,perf_trace"

# strip (unstripped is 416 MB and bloats the APK past 400 MB), then copy back.
# scp and plain `ssh host cat file` are BOTH corrupted by lxhalle's login banner —
# use base64 + grep, which is the only transfer that reliably survives it:
ssh "$HOST" "cd $RDIR && ~/android-ndk-setup/android-ndk-r27b/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip \
  --strip-debug -o /tmp/libcesium_prof.so target/aarch64-linux-android/profiling/libcesium_rs.so"
ssh "$HOST" "base64 -w0 /tmp/libcesium_prof.so" 2>/dev/null \
  | tr -d '\r' | grep -oE '[A-Za-z0-9+/=]{200,}' | base64 -d \
  > ~/Focusflight/app/src/main/jniLibs/arm64-v8a/libcesium_rs.so

# -x cargoNdkBuild stops Gradle kicking off a LOCAL CesiumRS compile.
cd ~/Focusflight && ./gradlew :app:assembleDebug -x cargoNdkBuild
adb install -r -d app/build/outputs/apk/debug/app-debug.apk
```

⚠️ `~/Focusflight/app/src/main/jniLibs/arm64-v8a/libcesium_rs.so` was **restored to its
pre-session build** at the end of this session. The phone has the instrumented one
installed; the checked-out file does not match it. Rebuild as above if you need them to
agree.

---

## 2. Already measured — don't redo these

On-device (Galaxy S23, Snapdragon 8 Gen 2), release-equivalent codegen. **Verdict: load
time is a non-issue on device. Closed.**

| | phone | laptop (release) |
|---|---:|---:|
| A350 exterior — loads at startup | **131 ms** | 135 ms |
| 787 cockpit — loads on first cockpit entry | **120 ms** | 65 ms |

Per-stage, cockpit: asset read 28.7 / build_atlas 10.2 / glTF parse 17.4 / mesh walk 10.4 /
vertex buffers 0.6 / mip chain 23.1 / texture upload 16.3 / pipeline 13.1 ms.
Per-stage, A350: glTF parse 26.8 / mesh walk 1.3 / vertex buffers 15.9 / mip chain 64.0 /
texture upload 7.0 / pipeline 16.3 ms.

The pure-CPU stages are **as fast or faster on the phone** than on the laptop (prime core
boosts to 3.36 GHz). The cockpit's extra 55 ms vs laptop is I/O and driver work — reading
4.3 MB out of the APK rather than the page cache, plus GPU upload and pipeline creation.
Battery temp went *down* during the load test (37.7 → 36.8 °C) while fast-charging.

Both loads are **lazy and one-time**: the A350 at startup, the cockpit on first entry to
cockpit view (`tracker.rs`, the `self.cockpit_renderer.is_none()` guard). Framerate is
unaffected once loaded. The ~500 ms the user originally noticed was a **debug build**;
debug is ~11× slower here and does not ship.

Timings come from permanent `log::info!("[loadtime] …")` lines in
`ModelRenderer::new_with_options` and `cockpit_model::load` — read them with
`adb logcat -d | grep loadtime`.

---

## 3. Uncommitted work in this repo

Branch `main`, last commit `ab43bcf`. Everything below is **uncommitted**. Note that
`telemetry/generator.rs`, `tracker.rs` and `tests/flight_plan.rs` had pre-existing edits
from *before* this session — those are not mine.

**New files**
- `A350-1000.glb` — replacement exterior model (Airbus A350-1000, textured)
- `crates/cesium-flight/src/aircraft_model.rs` — everything model-specific for it
- `crates/cesium-flight/src/cockpit_screens.rs` — the flight-deck display atlas
- `tools/phone_soak.sh` — the soak harness above

**Modified**
- `crates/cesium-engine/src/render/model_pipeline/pipeline.rs` — new `ModelOptions`
  fields (`origin_offset`, `post_scale`, `uv_override`, `material_unlit`,
  `max_texture_size`), gamma-correct mip chain, `unlit` vertex attribute, loadtime logging
- `crates/cesium-engine/src/render/model_pipeline/shader.wgsl` — `unlit` passthrough +
  emissive mix
- `crates/cesium-flight/src/cockpit_model.rs` — wires the atlas in, forces the display
  material to full value
- `crates/cesium-flight/src/lib.rs` — two new module declarations

### What changed, in one line each

1. **A350-1000 swap.** Old `A350.glb` kept deliberately (nothing references it) as a
   fallback. Pivot and scale matched to the outgoing mesh so nothing shifted or resized —
   constants and their derivation are documented in `aircraft_model.rs`.
2. **Mipmaps.** Model textures now get a gamma-correct mip chain; without it the livery's
   panel lines crawled. Capped at 1024² for the aircraft, which is **bit-identical** on
   screen and saves ~16 MB.
3. **Cockpit screens.** The four forward displays now show a PFD and two NDs, painted in
   code. Static by design.

---

## 4. Loose ends

- **`run_perf_scenario.sh` scenario 4 doesn't work from a cold launch.** The receiver
  fires and `api.rs::run_perf_scenario` sends `CameraSetMode(Cockpit)`, but the tracker
  doesn't act on it unless a flight is loaded — so cockpit view has to be reached by hand.
  Worth fixing if that script matters. Also note `am broadcast` needs an explicit
  component (`-n com.example.focusflight/.engine.live.PerfScenarioReceiver`) on Android 16;
  the implicit form in the script is silently dropped.
- **Mip-chain LUT — the one real optimisation left.** `build_mip_chain` does three `powf`
  per output texel for linear→sRGB; that's 64 ms of the A350's 131 ms startup on device,
  the single largest item. A lookup table (the forward direction already has one) should
  cut it severalfold with no visual change. ~20 lines. Optional, not a fix.
- **The two NDs are identical**, which a real flight deck wouldn't be. Give the first
  officer's a different range, or make one an EICAS page. Values are five constants at the
  top of `cockpit_screens.rs`.
- **6 pre-existing test failures** in `cargo test --lib` (`test_touch` ×2, three flight
  telemetry tests, `test_fuzz_frustum_coverage`). Verified these fail at pristine `HEAD`
  too — unrelated to any of the above, but don't be alarmed by them.

---

## 5. Gotchas that cost time this session

- **lxhalle's login banner corrupts binary transfers.** `scp` fails outright and
  `ssh host "cat file"` silently prepends the banner. The base64 + `grep -oE` pipeline in
  §1 is what actually works.
- **`cargo test -- --ignored` and `--include-ignored` are mutually exclusive.** Use
  `--include-ignored` to run both ignored and normal tests.
- **Debug vs release is ~11×** for this workload. Always state which profile a number came
  from.
