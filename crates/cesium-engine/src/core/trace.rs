//! Android ATrace spans for on-device CPU profiling, gated behind the
//! `perf_trace` feature so normal release builds pay zero cost.
//!
//! Sections show up in a Perfetto trace on the same timeline as standard
//! Android system tracks (gfx, sched, JNI, GC), which is what lets a captured
//! trace attribute engine subsystems (tile streaming, cockpit rendering, ...)
//! against real device activity instead of just an isolated frame-time number.

#[cfg(all(target_os = "android", feature = "perf_trace"))]
pub struct ScopedTrace(ndk::trace::Section);

#[cfg(all(target_os = "android", feature = "perf_trace"))]
impl ScopedTrace {
    /// Prefer a `&'static str` literal at per-frame call sites — no per-frame
    /// `format!` — since each call still pays a `CString` allocation inside
    /// `ndk::trace` regardless. A dynamically built name (e.g. a scenario
    /// marker) is fine for the rare, non-per-frame call sites.
    #[inline]
    pub fn new(name: &str) -> Self {
        match ndk::trace::Section::new(name) {
            Ok(section) => Self(section),
            Err(_) => unreachable!("trace section name had an interior NUL byte"),
        }
    }
}

#[cfg(not(all(target_os = "android", feature = "perf_trace")))]
pub struct ScopedTrace;

#[cfg(not(all(target_os = "android", feature = "perf_trace")))]
impl ScopedTrace {
    #[inline(always)]
    pub fn new(_name: &str) -> Self {
        Self
    }
}
