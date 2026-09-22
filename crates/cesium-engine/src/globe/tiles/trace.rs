//! Tile lifecycle trace: one CSV row per event to `tile_trace.csv`. Off until
//! [`enable`] is called (the windowed app does); every call is a single atomic load
//! while it is off. Analyse with `tools/tile_churn.py`.

use crate::globe::quadtree::TileId;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::Instant;

pub const TRACE_FILE: &str = "tile_trace.csv";

static ENABLED: AtomicBool = AtomicBool::new(false);
static FRAME: AtomicU64 = AtomicU64::new(0);
static SINK: Mutex<Option<(BufWriter<File>, Instant)>> = Mutex::new(None);

pub fn enable() {
    match File::create(TRACE_FILE) {
        Ok(f) => {
            let mut out = BufWriter::new(f);
            let _ = writeln!(out, "frame,t_ms,kind,event,z,x,y,info");
            *SINK.lock().unwrap() = Some((out, Instant::now()));
            ENABLED.store(true, Ordering::Release);
            log::info!("Tile trace -> {TRACE_FILE}");
        }
        Err(e) => log::warn!("tile trace disabled: {TRACE_FILE}: {e}"),
    }
}

#[inline]
pub fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

/// `kind`: `tex`, `hgt`, `mesh`, `draw`, `frame`, `mark`.
pub fn event(kind: &str, event: &str, id: Option<TileId>, info: std::fmt::Arguments) {
    if !enabled() {
        return;
    }
    let frame = FRAME.load(Ordering::Relaxed);
    if let Some((out, start)) = SINK.lock().unwrap().as_mut() {
        let t = start.elapsed().as_secs_f64() * 1000.0;
        let _ = match id {
            Some(id) => writeln!(out, "{frame},{t:.2},{kind},{event},{},{},{},{info}", id.z, id.x, id.y),
            None => writeln!(out, "{frame},{t:.2},{kind},{event},,,,{info}"),
        };
    }
}

/// Advances the frame counter; flushes once a second's worth of frames.
pub fn next_frame() {
    if !enabled() {
        return;
    }
    let f = FRAME.fetch_add(1, Ordering::Relaxed) + 1;
    if f % 60 == 0 {
        if let Some((out, _)) = SINK.lock().unwrap().as_mut() {
            let _ = out.flush();
        }
    }
}

#[macro_export]
macro_rules! tile_event {
    ($kind:expr, $ev:expr, $id:expr) => {
        if $crate::globe::tiles::trace::enabled() {
            $crate::globe::tiles::trace::event($kind, $ev, $id, format_args!(""));
        }
    };
    ($kind:expr, $ev:expr, $id:expr, $($arg:tt)+) => {
        if $crate::globe::tiles::trace::enabled() {
            $crate::globe::tiles::trace::event($kind, $ev, $id, format_args!($($arg)+));
        }
    };
}
