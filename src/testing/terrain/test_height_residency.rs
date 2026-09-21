//! **F2's follow-up** — what the height cache being too small actually costs
//! (`docs/terrain-plan.md` §9 F2).
//!
//! F2 measured that at three of the ten real poses the tree wants more distinct height
//! sources than the 32 MiB slice can hold — `alps_inn_valley` 312 against 254 — and called
//! the consequence "churn". It did not measure the churn, and it did not look at the one
//! thing that makes churn more than a re-download: **whether a tile whose fetch is still in
//! flight can be evicted, and then fetched a second time over the network while the first
//! fetch is still running.**
//!
//! # Why that is a live question and not a hypothetical
//!
//! There are two dedup layers between `TileSystem::request_height_chain` and the wire, and
//! neither covers the in-flight window:
//!
//! 1. `HeightTileManager::request_tile` returns early when `cache.get_state(&src)` is
//!    `Some(..)`. The `Fetching` placeholder it writes is an ordinary `LruCache` entry, and
//!    `LruCache::put` evicts the least-recently-used entry without asking what state it is
//!    in. So the placeholder that *is* the dedup can be thrown away before the data lands.
//! 2. `TileFetcher::request_tile` keeps a `HashSet` of queued ids — but it removes an id
//!    from that set the moment the worker **pops** the request, i.e. when the download
//!    starts, not when it finishes (`tile_fetcher.rs`, `q.1.remove(&r.id)` inside the pop).
//!    So it dedups the *queued* window and not the *downloading* window.
//!
//! Cesium's counterpart is `GlobeSurfaceTile.eligibleForUnloading`
//! (`Scene/GlobeSurfaceTile.js`), which refuses to free a tile whose imagery is
//! `RECEIVING` or `TRANSFORMING` — the exact guard this cache does not have.
//!
//! # What is measured here, and how honestly
//!
//! [`double_fetches_at_the_binding_poses`] runs the **real** [`HeightTileManager`] — real
//! `TileCacheManager`, real `TileFetcher`, real tokio worker, real HTTP — against a local
//! [`CountingSource`] that serves one committed Terrarium fixture for every request and
//! counts the GETs. The request sequence is `TileSystem::request_height_chain` over the
//! arriving tree frame by frame and then over the settled visible set, which is what
//! production does as a camera reaches a pose and sits there.
//!
//! Two numbers come out, from opposite ends of the pipe and independent of each other:
//!
//! * **evicted in flight** — counted on the manager side: an id we watched go
//!   `None → Fetching` whose cache entry is gone again before any result arrived.
//! * **GETs vs distinct tiles** — counted on the wire by the server.
//!
//! # What it found: the churn is not there, and F2's 312 is not a production number
//!
//! Neither number is ever above zero. F2's 312 comes from `collect_sources`, which recurses
//! the **whole quadtree**; production asks for heights only for the visible set and its
//! ancestor chains, which is **220** tiles at the worst pose — inside 254. So the raise of
//! the slice that followed this measurement is headroom for the deeper tree of §9 F5, not a
//! repair of a fault. The two small tests at the bottom keep the *mechanism* written down,
//! so that if a future working set does overflow the capacity the finding is a state check
//! away rather than an investigation.
//!
//! # Why `culling` is not in this path
//!
//! `cargo test --release --lib culling::` must keep reporting 32/0/1 and libtest's filter
//! is a plain substring match on the full test path, so nothing under `testing::terrain::`
//! may contain the substring. Same reason as its three siblings.

use std::collections::{HashMap, HashSet};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cesium_engine::globe::quadtree::TileId;
use cesium_engine::globe::terrain::HeightTileManager;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, HEIGHT_TILE_BYTES};
use cesium_engine::globe::tiles::tile_cache::TileState;
use cesium_engine::globe::tiles::tile_fetcher::TilePriority;

use super::test_mesh_density::settled_shipped;
use super::test_terrain_occlusion::{real_poses, RealWorld};

/// How many frames of a motionless camera to replay. At 16 ms a frame this is a third of
/// a second — far less than the "settle" F2 described, and already enough.
const FRAMES: usize = 24;

/// Wall-clock between two replayed frames. The engine's own frame budget; it is what makes
/// "in flight" a real window rather than an artefact of replaying as fast as possible.
const FRAME_MS: u64 = 16;

/// Per-request latency the local source adds before answering, milliseconds.
///
/// A real Terrarium GET to `s3.amazonaws.com` from this machine is 40-200 ms. 60 ms is the
/// optimistic end of that, so every number below is a **lower** bound on what the shipped
/// source does.
const SOURCE_LATENCY_MS: u64 = 60;

/// The budgets compared: the 32 MiB slice B4 shipped, and the 48 MiB §9 F2 names.
const BUDGETS_MIB: [usize; 2] = [32, 48];

/// The largest set of distinct height tiles [`double_fetches_at_the_binding_poses`] saw
/// production ask for at any of the ten real poses, at either shipped imagery style:
/// `alps_inn_valley`, 220 — the visible set (103 tiles) plus every ancestor chain.
///
/// **Not F2's 312.** That number is `collect_sources` over the whole quadtree, interior and
/// culled nodes included, and production never requests heights for those: their
/// `HeightBounds` come from D1's inheritance margin instead. The gap between the two is the
/// whole of §9 F2's "the height slice is the one that binds".
const WORST_MEASURED_WORKING_SET: usize = 220;

// ── a local Terrarium source that counts ────────────────────────────────────────────

/// A one-thread HTTP/1.1 server that answers every GET with the same committed Terrarium
/// fixture, after [`SOURCE_LATENCY_MS`], and counts what it was asked for.
///
/// Hand-rolled on `std::net` rather than pulled in as a dependency: the root crate has no
/// HTTP server in its graph and a measurement is not a reason to add one — the same
/// argument `test_terrain_occlusion::fetch_missing` makes for shelling out to `curl`.
struct CountingSource {
    url_template: String,
    gets: Arc<AtomicUsize>,
    per_tile: Arc<Mutex<HashMap<String, usize>>>,
    stop: Arc<AtomicBool>,
}

impl CountingSource {
    fn new() -> Self {
        let png = Arc::new(fixture_png());
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind loopback");
        let port = listener.local_addr().expect("local addr").port();
        listener
            .set_nonblocking(true)
            .expect("non-blocking listener");

        let gets = Arc::new(AtomicUsize::new(0));
        let per_tile = Arc::new(Mutex::new(HashMap::new()));
        let stop = Arc::new(AtomicBool::new(false));

        let (g, p, s) = (gets.clone(), per_tile.clone(), stop.clone());
        std::thread::spawn(move || {
            while !s.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((sock, _)) => {
                        let (g, p, png) = (g.clone(), p.clone(), png.clone());
                        // One thread per connection: `TileFetcher` holds 8 in flight, so
                        // this stays a handful of threads and keeps the latency model
                        // honest (a single-threaded server would serialise them and
                        // manufacture the very eviction pressure being measured).
                        std::thread::spawn(move || serve_one(sock, &g, &p, &png));
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(1));
                    }
                    Err(_) => break,
                }
            }
        });

        Self {
            url_template: format!("http://127.0.0.1:{port}/{{z}}/{{x}}/{{y}}.png"),
            gets,
            per_tile,
            stop,
        }
    }

    fn reset(&self) {
        self.gets.store(0, Ordering::Relaxed);
        self.per_tile.lock().expect("per-tile counts").clear();
    }

    fn gets(&self) -> usize {
        self.gets.load(Ordering::Relaxed)
    }

    /// `(distinct paths served, GETs beyond the first for some path)`.
    fn distinct_and_repeats(&self) -> (usize, usize) {
        let m = self.per_tile.lock().expect("per-tile counts");
        let distinct = m.len();
        let repeats: usize = m.values().map(|n| n - 1).sum();
        (distinct, repeats)
    }
}

impl Drop for CountingSource {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

fn serve_one(
    mut sock: TcpStream,
    gets: &AtomicUsize,
    per_tile: &Mutex<HashMap<String, usize>>,
    png: &[u8],
) {
    let mut buf = [0u8; 2048];
    let n = match sock.read(&mut buf) {
        Ok(n) if n > 0 => n,
        _ => return,
    };
    let req = String::from_utf8_lossy(&buf[..n]);
    let path = req
        .split_whitespace()
        .nth(1)
        .unwrap_or("/unknown")
        .to_string();

    gets.fetch_add(1, Ordering::Relaxed);
    *per_tile
        .lock()
        .expect("per-tile counts")
        .entry(path)
        .or_insert(0) += 1;

    std::thread::sleep(Duration::from_millis(SOURCE_LATENCY_MS));

    let header = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: image/png\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        png.len()
    );
    let _ = sock.write_all(header.as_bytes());
    let _ = sock.write_all(png);
    let _ = sock.flush();
}

/// The bytes of one committed Terrarium fixture, served for every tile.
///
/// *Which* tile the bytes belong to is irrelevant here: nothing in this file reads a
/// height. What matters is that the response decodes, so the manager takes its `Ready`
/// branch and the entry it writes is the same size the shipped cache accounts for.
fn fixture_png() -> Vec<u8> {
    let path = format!(
        "{}/assets/terrain_fixtures/zugspitze_z12_2172_1433.png",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("reading {path}: {e}"))
}

// ── the replay ──────────────────────────────────────────────────────────────────────

/// `TileSystem::request_height_chain`, which is private to the engine and eight lines long.
///
/// Reproduced rather than exposed because production's copy is the thing under measurement
/// and widening its visibility to measure it would be the wrong trade. It is verbatim:
/// the tile at `High`, then its whole ancestor chain at `Low`, skipping ancestors the cache
/// already knows about.
fn request_height_chain(heights: &mut HeightTileManager, id: TileId) {
    heights.request_tile(id, TilePriority::High);
    let mut curr = heights.source_tile_for(id);
    while let Some(p) = curr.parent() {
        if heights.cache.get_state(&p).is_none() {
            heights.request_tile(p, TilePriority::Low);
        }
        curr = p;
    }
}

/// What one replay measured.
struct Churn {
    /// Distinct source tiles the replay ever asked the manager for.
    wanted: usize,
    /// Resident entries at the end, and the capacity they are held against.
    residency: (usize, usize),
    /// Ids that went `None → Fetching` and whose entry was gone again before any result
    /// arrived — the eviction `eligibleForUnloading` exists to forbid.
    evicted_in_flight: usize,
    /// GETs the source answered.
    gets: usize,
    /// Distinct paths the source answered.
    distinct_served: usize,
    /// GETs beyond the first for some path: re-downloads, of either kind.
    repeats: usize,
    /// `Ready` entries evicted and then asked for again — ordinary LRU churn, the
    /// comparison the in-flight number has to be read against.
    evicted_ready: usize,
}

/// Replays the arrival — `frames` in order, then the settled set until [`FRAMES`] frames
/// have gone by — against a real manager whose source is `src`, at `budget_mib`.
///
/// The arrival is included rather than only the settled set because it is the harder case
/// and the one production actually flies: the tree grows from the roots as heights land, so
/// the early frames request a coarse set and the later ones a deep one, and the union over
/// the arrival is what the cache has to hold.
fn replay(frames_in: &[Vec<TileId>], src: &CountingSource, budget_mib: usize) -> Churn {
    src.reset();
    let config = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            source_url: src.url_template.clone(),
            height_cache_budget_bytes: budget_mib * 1024 * 1024,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let mut heights = HeightTileManager::new(&config);

    let mut wanted: HashSet<TileId> = HashSet::new();
    let mut in_flight: HashSet<TileId> = HashSet::new();
    let mut was_ready: HashSet<TileId> = HashSet::new();
    let mut evicted_in_flight = 0usize;
    let mut evicted_ready = 0usize;

    let settled = frames_in.last().expect("at least one frame").clone();
    for f in 0..FRAMES.max(frames_in.len()) {
        let visible = frames_in.get(f).unwrap_or(&settled);
        for id in visible {
            let s = heights.source_tile_for(*id);
            // The chain too: `request_height_chain` asks for every ancestor of the source
            // as well, and those are height tiles the cache has to hold like any other.
            let mut c = s;
            wanted.insert(c);
            while let Some(p) = c.parent() {
                wanted.insert(p);
                c = p;
            }

            // The two eviction cases, distinguished *before* the request that would
            // re-issue the fetch. `peek_state` does not promote, so looking does not
            // perturb the order being measured.
            let absent = heights.cache.peek_state(&s).is_none();
            if absent && in_flight.remove(&s) {
                evicted_in_flight += 1;
            } else if absent && was_ready.remove(&s) {
                evicted_ready += 1;
            }

            request_height_chain(&mut heights, *id);

            if matches!(heights.cache.peek_state(&s), Some(TileState::Fetching)) {
                in_flight.insert(s);
            }
        }

        std::thread::sleep(Duration::from_millis(FRAME_MS));
        heights.update();

        // Anything that landed this frame is no longer in flight.
        in_flight.retain(|id| {
            let still = matches!(heights.cache.peek_state(id), Some(TileState::Fetching));
            if !still {
                was_ready.insert(*id);
            }
            still
        });
    }

    // Let the tail of the in-flight requests drain so the GET count is complete.
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline && !heights.is_loading_complete() {
        std::thread::sleep(Duration::from_millis(20));
        heights.update();
    }
    std::thread::sleep(Duration::from_millis(SOURCE_LATENCY_MS * 3));

    let (distinct_served, repeats) = src.distinct_and_repeats();
    Churn {
        wanted: wanted.len(),
        residency: heights.residency(),
        evicted_in_flight,
        gets: src.gets(),
        distinct_served,
        repeats,
        evicted_ready,
    }
}

/// **The measurement §9 F2 owed, and the one it did not know it owed.**
///
/// Three poses, two budgets, the real manager and a real socket. The first column pair is
/// F2's own question — does the working set fit — and the last three are the question F2
/// did not ask: what the overflow costs on the wire.
///
/// ```text
/// cargo test --release --lib terrain::test_height_residency -- --ignored --nocapture
/// ```
#[test]
#[ignore = "measurement, needs the network for real height tiles and binds a loopback port"]
fn double_fetches_at_the_binding_poses() {
    let mut world = RealWorld::new();
    let src = CountingSource::new();

    println!("\n  [F2b] height-cache churn at a motionless camera, {FRAMES} frames, real manager");
    println!(
        "    (source latency {SOURCE_LATENCY_MS} ms, frame {FRAME_MS} ms, \
         {} B per resident tile)",
        HEIGHT_TILE_BYTES
    );
    println!(
        "    {:<22} {:>7} {:>8} {:>11} {:>10} {:>7} {:>9} {:>9} {:>9}",
        "pose",
        "budget",
        "wanted",
        "resident",
        "evict/fly",
        "GETs",
        "distinct",
        "repeats",
        "evict/rdy"
    );

    // The three F2 found binding, plus one that fits, as the control: if the fitting pose
    // shows the same in-flight evictions then the finding is about the replay and not
    // about the budget.
    let interesting = [
        "alps_inn_valley",
        "alps_low",
        "salzach_to_alps",
        "rhone_valley",
    ];
    let mut any_in_flight_at_32 = false;

    // Both shipped imagery styles: the default Carto `@2x` (512², the one F2's first
    // table is measured at) and the Esri 256² style, whose `lod_factor` is twice as eager
    // and which therefore draws roughly seven times the tiles — the heavier case for the
    // height cache, and the one that decides whether the slice binds at all.
    for (style, texture_px) in [("carto @2x 512²", 512.0f32), ("esri 256²", 256.0)] {
        println!("\n    — {style} —");
        for (name, p) in real_poses() {
            if !interesting.contains(&name) {
                continue;
            }
            let (frames, asked) = settled_shipped(&p, texture_px, &mut world);
            println!(
                "    {name} — F2's whole-tree count was {asked}; production asks for the \
                 visible set and its chains, {} visible at the settled frame",
                frames.last().map(|f| f.len()).unwrap_or(0)
            );
            for mib in BUDGETS_MIB {
                let c = replay(&frames, &src, mib);
                println!(
                    "    {name:<22} {:>7} {:>8} {:>11} {:>10} {:>7} {:>9} {:>9} {:>9}",
                    format!("{mib} MiB"),
                    c.wanted,
                    format!("{}/{}", c.residency.0, c.residency.1),
                    c.evicted_in_flight,
                    c.gets,
                    c.distinct_served,
                    c.repeats,
                    c.evicted_ready,
                );
                if mib == 32 && c.evicted_in_flight > 0 {
                    any_in_flight_at_32 = true;
                }
            }
        }
    }

    println!(
        "    (evict/fly = an entry that went None -> Fetching and was gone before any \
         result arrived: the re-request that follows it is a second GET for a tile still \
         downloading. repeats = GETs beyond the first for the same path, counted on the \
         wire, which is the same event seen from the other end. evict/rdy = ordinary LRU \
         churn on entries that had already landed.)"
    );
    println!(
        "    in-flight evictions observed at 32 MiB on a binding pose: {}",
        if any_in_flight_at_32 { "yes" } else { "no" }
    );
}

/// The residency arithmetic the raised slice rests on, with no network at all.
///
/// [`WORST_MEASURED_WORKING_SET`] is what the measurement above found production actually
/// asks for at the worst of the ten real poses. The slice has to hold it with room for the
/// terrain refinement of §9 F5 to grow into, and it has to stay a *slice* — B4's claim —
/// leaving imagery well over the 103 MiB F2 measured it peaking at. All three are
/// arithmetic on shipped constants, so all three belong in the gate rather than behind
/// `--ignored`.
#[test]
fn the_raised_height_slice_covers_the_measured_working_set_and_still_leaves_imagery_room() {
    let config = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let entries = config.terrain.height_cache_budget_bytes / HEIGHT_TILE_BYTES;

    // Desktop: the measured worst case, with half again as much room for F5's deeper tree.
    #[cfg(not(target_os = "android"))]
    assert!(
        entries >= 3 * WORST_MEASURED_WORKING_SET / 2,
        "the desktop height slice holds {entries} tiles against a measured worst case of \
         {WORST_MEASURED_WORKING_SET}; §9 F5 refines the tree and needs the margin"
    );
    // Android is held to the unrun soak of §9 F3 and must still cover what is measured.
    #[cfg(target_os = "android")]
    assert!(entries >= WORST_MEASURED_WORKING_SET);

    // Still a slice, not an addition (B4).
    assert_eq!(
        config.imagery_cache_budget_bytes() + config.terrain.height_cache_budget_bytes,
        config.tile_cache_budget_bytes,
    );

    // And imagery keeps well over its measured peak: F2 measured 103 MiB at the worst
    // pose against the 480 MiB it had, so the raised slice must still leave it 4x that.
    assert!(
        config.imagery_cache_budget_bytes() >= 4 * 103 * 1024 * 1024,
        "imagery is left {} MiB against a measured peak of 103 MiB",
        config.imagery_cache_budget_bytes() / (1024 * 1024)
    );
}

/// The one structural fact the measurement above is about, stated without a socket:
/// `TileCacheManager` will evict a `Fetching` placeholder.
///
/// This is the difference from Cesium's `GlobeSurfaceTile.eligibleForUnloading`, which
/// refuses to free a tile in `RECEIVING`/`TRANSFORMING`. Whether it *matters* is the
/// measurement; that it is *possible* is this assertion, and it costs nothing to keep.
#[test]
fn the_cache_will_evict_a_tile_whose_fetch_is_still_in_flight() {
    use cesium_engine::globe::tiles::tile_cache::TileCacheManager;
    use std::num::NonZeroUsize;

    let mut cache: TileCacheManager<u8> =
        TileCacheManager::new(NonZeroUsize::new(2).unwrap(), Duration::from_secs(10));
    let id = |n: u32| TileId { z: 15, x: n, y: 0 };

    cache.mark_fetching(id(0));
    cache.mark_fetching(id(1));
    assert!(matches!(
        cache.peek_state(&id(0)),
        Some(TileState::Fetching)
    ));

    // One more entry than the cache holds, and the oldest goes — `Fetching` or not.
    cache.mark_fetching(id(2));
    assert!(
        cache.peek_state(&id(0)).is_none(),
        "a Fetching placeholder survived eviction; if this ever holds, the dedup in \
         HeightTileManager::request_tile is sound and the churn measurement is stale"
    );
}

/// The second dedup layer, and the window it does not cover.
///
/// `TileFetcher` keeps a `HashSet` of queued ids so a tile asked for twice while still
/// queued is fetched once. It removes the id when the worker **pops** the request — i.e.
/// when the download begins — so the set is empty of that id for the whole time the bytes
/// are on the wire. This test watches exactly that: request, wait for the queue to drain
/// into the worker, request again, and see a second request accepted.
#[test]
fn the_fetcher_dedups_the_queued_window_and_not_the_downloading_one() {
    let src = CountingSource::new();
    let config = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            source_url: src.url_template.clone(),
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let mut heights = HeightTileManager::new(&config);
    let id = TileId {
        z: 15,
        x: 17_000,
        y: 11_500,
    };

    heights.request_tile(id, TilePriority::High);

    // Wait until the worker has popped it: the request is now downloading, and
    // `SOURCE_LATENCY_MS` guarantees it has not finished.
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline && !heights.is_loading_complete() {
        std::thread::sleep(Duration::from_millis(2));
    }
    assert_eq!(
        src.gets(),
        1,
        "the first request must have reached the wire"
    );

    // Now do what an eviction does: drop the `Fetching` placeholder, then ask again.
    heights.cache.clear();
    heights.request_tile(id, TilePriority::High);

    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline && src.gets() < 2 {
        std::thread::sleep(Duration::from_millis(2));
    }
    assert_eq!(
        src.gets(),
        2,
        "with the placeholder gone the same tile was fetched again while the first \
         fetch was still on the wire — this is the window neither dedup layer covers"
    );
}
