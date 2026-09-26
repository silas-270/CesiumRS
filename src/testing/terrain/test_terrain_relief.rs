//! **The pre-check question**: can something a thousand times cheaper than the march
//! decide, before the march runs, whether the march is going to remove anything?
//!
//! Measuring terrain occlusion against the frame found the shape of the
//! answer: at six poses the stage removes twenty tiles at **one** of them and nothing at
//! four others, while charging 300–500 µs at all five that clear the altitude gate. The analysis
//! names the discriminator it could not build — "**relief in view, not height above
//! ground**" — and records that the quantity is the one the march itself computes, which
//! is why it stayed a note.
//!
//! This module is that note, measured. It does three things and nothing else:
//!
//! 1. **Extends the pose family**, because a threshold fitted to six poses is fitted to
//!    six poses. Twenty-nine poses in six families — valley-under-a-wall, on a ridge,
//!    terrain step, plain, coast, and the altitude band above the gate — each one placed
//!    over the **DEM's** ground rather than over a number, with its heading measured off
//!    the built camera ([`terrain_relief_poses_are_where_they_say`], avoiding earlier
//!    pose traps).
//!
//! 2. **Measures what terrain occlusion removes at each of them**, with both altitude gates opened, so
//!    the raw benefit is visible rather than hidden behind the gate that is already
//!    shipped. The instrument is `test_terrain_step::settle` — the production
//!    [`Fill::Visible`] height policy, the renderer's own LOD factors, no GPU needed.
//!
//! 3. **Computes four candidate statistics** off the settled tree, all of them cheap
//!    enough to run before the march, and prints them next to the removal so the
//!    threshold is read off data rather than argued:
//!
//!    | stat | what it is | the idea it tests |
//!    |---|---|---|
//!    | `hi_above_eye_m` | `max(node.hi) − eye`, metres | relief as a **height** — the obvious one |
//!    | `relief_hi_deg` | `max elev(near, node.hi)` | relief as an **angle**, i.e. height weighted by distance |
//!    | `relief_floor_deg` | `max elev(near, max(floor_grid))` | the same angle off what height-aware bounds can **prove** is there — the quantity the march would stamp |
//!    | `shadowed` | drawn tiles further out than that wall whose box top is under it | a **one-number march**: how many tiles could possibly fall in the shadow |
//!
//! Only visible leaves are walked, so the cost is the visible set (59–170 nodes) and not
//! the tree — that is the whole premise, and
//! [`terrain_relief_probe_is_cheap_against_the_march`] is what measures it rather than
//! assuming it.
//!
//! ```text
//! CESIUM_HEIGHT_CACHE=/tmp/dem \
//!   cargo test --release --lib terrain::test_terrain_relief -- --ignored --nocapture
//! ```

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::geometry::EARTH_RADIUS_A_F64;
use cesium_engine::globe::quadtree::terrain_occlusion::MIN_RANGE_M;
use cesium_engine::globe::quadtree::{
    tile_bounds, QuadtreeManager, QuadtreeNode, TerrainOcclusionConfig, TileId,
};
use cesium_engine::globe::terrain::Heightfield;
use glam::DVec3;

use crate::testing::culling::cameras::{build_camera, ViewParams};
use crate::testing::terrain::test_terrain_occlusion::{fetch_missing, RealWorld};
use crate::testing::terrain::test_terrain_step::{
    box_elevation_deg, dem_height_m, dest, ecef, elevation_deg, frustum_for, ground_range_m,
    mercator_xy, normal_at, settle, wrap_deg, Fill, DEM_Z,
};

/// Frames each arm is settled for. Six is what `test_terrain_step` uses and for its
/// reason: the height policy is `Fill::Visible`, so the tree and the cache chase each
/// other for a few frames before either is steady.
const FRAMES: usize = 6;

/// What kind of place a pose is, so the report can be read by family rather than by row.
///
/// The families are not decoration: the verdict this module has to reach is "the
/// pre-check is ≥ 0 at **every** family and positive where relief stands", and a
/// statistic that separates within one family and not across them is overfitted.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Family {
    /// A valley floor under a kilometre-scale wall — the one shape where terrain occlusion pays.
    Valley,
    /// The camera standing on the ridge instead of under it.
    Ridge,
    /// One step, then flat ground behind it — the terrain step shape.
    Step,
    /// Low relief in every direction.
    Plain,
    /// Water in front, land behind or beside.
    Coast,
    /// Relief below, but the camera is well above the ground.
    Above,
}

/// One measurement pose, placed over the DEM's ground rather than over an altitude.
pub(crate) struct ReliefPose {
    pub(crate) name: &'static str,
    pub(crate) lon: f64,
    pub(crate) lat: f64,
    /// Metres above the **local ground**, read out of the DEM when the pose is built.
    pub(crate) agl_m: f64,
    /// Compass bearing, degrees clockwise from north.
    pub(crate) bearing_deg: f64,
    /// Degrees below the horizontal.
    pub(crate) below_deg: f64,
    pub(crate) family: Family,
    pub(crate) what: &'static str,
}

impl ReliefPose {
    /// The `ViewParams` this pose becomes.
    ///
    /// `yaw_deg: -bearing` — `ViewParams::yaw_deg` is composed about the **nadir**-aligned
    /// view axis, so a rotation of `+yaw` about `−up` is `−yaw` about `+up`.
    /// [`terrain_relief_poses_are_where_they_say`] measures the
    /// heading off the built camera rather than trusting this line.
    fn view(&self, ground_m: f64) -> ViewParams {
        ViewParams {
            sweep: "relief",
            lat_deg: self.lat,
            lon_deg: self.lon,
            alt_m: ground_m + self.agl_m,
            pitch_deg: 90.0 - self.below_deg,
            yaw_deg: -self.bearing_deg,
            roll_deg: 0.0,
            width: 1280,
            height: 720,
            mode: CameraMode::Free,
        }
    }
}

/// **The family**, and it is deliberately wider than the question.
///
/// The earlier six poses are all in it (`inn_valley_300`, `alps_ridge_tuxer`,
/// `reutlingen_albtrauf`, `stuttgart_kessel`, `alps_above_600`, `alps_cruise_9km` are the
/// same places at the same AGLs), and twenty-three more are around them. The point of the
/// extra ones is not coverage for its own sake: a pre-check that only has to separate
/// *one* positive pose from *four* negative ones can be separated by almost any monotone
/// quantity, and the resulting threshold would mean nothing.
pub(crate) fn relief_poses() -> Vec<ReliefPose> {
    use Family::*;
    vec![
        // ── valley under a wall — where terrain occlusion pays ──────────────────
        ReliefPose {
            name: "inn_valley_300",
            lon: 11.40,
            lat: 47.26,
            agl_m: 300.0,
            bearing_deg: 0.0,
            below_deg: 1.5,
            family: Valley,
            what: "Innsbruck under the Nordkette — §7f's one paying pose",
        },
        ReliefPose {
            name: "inn_valley_60",
            lon: 11.40,
            lat: 47.26,
            agl_m: 60.0,
            bearing_deg: 0.0,
            below_deg: 1.0,
            family: Valley,
            what: "the same valley at rooftop height",
        },
        ReliefPose {
            name: "inn_valley_700",
            lon: 11.40,
            lat: 47.26,
            agl_m: 700.0,
            bearing_deg: 0.0,
            below_deg: 1.5,
            family: Valley,
            what:
                "§7f's ladder rung that still removed eleven tiles — the threshold has to keep it",
        },
        ReliefPose {
            name: "owens_valley_sierra",
            lon: -118.10,
            lat: 36.60,
            agl_m: 60.0,
            bearing_deg: 270.0,
            below_deg: 1.5,
            family: Valley,
            what:
                "Owens Valley west into the 3 km Sierra escarpment — D3's shape, outside the Alps",
        },
        ReliefPose {
            name: "rhine_gorge",
            lon: 7.79,
            lat: 50.13,
            agl_m: 30.0,
            bearing_deg: 0.0,
            below_deg: 1.0,
            family: Valley,
            what: "the Middle Rhine gorge — 200 m of relief, not 2 000",
        },
        ReliefPose {
            name: "death_valley_floor",
            lon: -116.87,
            lat: 36.45,
            agl_m: 50.0,
            bearing_deg: 270.0,
            below_deg: 1.5,
            family: Valley,
            what: "Badwater under the Panamint wall",
        },
        ReliefPose {
            name: "zillertal_mayrhofen",
            lon: 11.86,
            lat: 47.17,
            agl_m: 200.0,
            bearing_deg: 180.0,
            below_deg: 1.5,
            family: Valley,
            what: "Mayrhofen looking south into the Zillertal Alps",
        },
        ReliefPose {
            name: "oetztal_soelden",
            lon: 11.00,
            lat: 46.97,
            agl_m: 200.0,
            bearing_deg: 180.0,
            below_deg: 2.0,
            family: Valley,
            what: "the Ötztal looking south at the Weisskugel wall",
        },
        ReliefPose {
            name: "wipptal_steinach",
            lon: 11.47,
            lat: 47.09,
            agl_m: 200.0,
            bearing_deg: 90.0,
            below_deg: 1.5,
            family: Valley,
            what: "the Wipptal across to the Tuxer flank",
        },
        ReliefPose {
            name: "valais_sion",
            lon: 7.36,
            lat: 46.23,
            agl_m: 200.0,
            bearing_deg: 180.0,
            below_deg: 1.5,
            family: Valley,
            what: "the Rhône valley at Sion, south into the Valais Alps",
        },
        ReliefPose {
            name: "chamonix_montblanc",
            lon: 6.87,
            lat: 45.92,
            agl_m: 200.0,
            bearing_deg: 135.0,
            below_deg: 3.0,
            family: Valley,
            what: "Chamonix under the Mont Blanc massif",
        },
        ReliefPose {
            name: "lauterbrunnen",
            lon: 7.91,
            lat: 46.59,
            agl_m: 150.0,
            bearing_deg: 135.0,
            below_deg: 2.0,
            family: Valley,
            what: "the Lauterbrunnen trough under the Jungfrau",
        },
        ReliefPose {
            name: "adige_bolzano",
            lon: 11.35,
            lat: 46.50,
            agl_m: 200.0,
            bearing_deg: 0.0,
            below_deg: 1.5,
            family: Valley,
            what: "the Adige valley north into the Sarntal Alps",
        },
        ReliefPose {
            name: "khumbu_namche",
            lon: 86.71,
            lat: 27.80,
            agl_m: 300.0,
            bearing_deg: 30.0,
            below_deg: 3.0,
            family: Valley,
            what: "Namche Bazaar, north-east toward the Khumbu wall",
        },
        ReliefPose {
            name: "pokhara_annapurna",
            lon: 83.98,
            lat: 28.21,
            agl_m: 200.0,
            bearing_deg: 0.0,
            below_deg: 2.0,
            family: Valley,
            what: "Pokhara at 800 m under 8 000 m of Annapurna",
        },
        ReliefPose {
            name: "yosemite_valley",
            lon: -119.58,
            lat: 37.74,
            agl_m: 100.0,
            bearing_deg: 90.0,
            below_deg: 1.5,
            family: Valley,
            what: "the Yosemite trough east along the valley",
        },
        ReliefPose {
            name: "grand_canyon_floor",
            lon: -112.09,
            lat: 36.10,
            agl_m: 100.0,
            bearing_deg: 0.0,
            below_deg: 1.5,
            family: Valley,
            what: "inside the Grand Canyon, north at the rim",
        },
        // ── on the ridge instead of under it ──────────────────────────────────────
        ReliefPose {
            name: "alps_ridge_tuxer",
            lon: 11.20,
            lat: 47.05,
            agl_m: 2.0,
            bearing_deg: 0.0,
            below_deg: 3.0,
            family: Ridge,
            what: "§7f's `alps_cockpit`: standing on the Tuxer massif at 2 m AGL",
        },
        ReliefPose {
            name: "zugspitze_summit",
            lon: 10.985,
            lat: 47.42,
            agl_m: 20.0,
            bearing_deg: 0.0,
            below_deg: 5.0,
            family: Ridge,
            what: "on the Zugspitze, north over the foreland",
        },
        ReliefPose {
            name: "jungfraujoch",
            lon: 7.98,
            lat: 46.55,
            agl_m: 20.0,
            bearing_deg: 315.0,
            below_deg: 3.0,
            family: Ridge,
            what: "the Jungfraujoch, north-west down the Aletsch flank",
        },
        ReliefPose {
            name: "alb_plateau_top",
            lon: 9.30,
            lat: 48.42,
            agl_m: 20.0,
            bearing_deg: 315.0,
            below_deg: 1.0,
            family: Ridge,
            what: "on the Alb plateau looking back out over the Albtrauf",
        },
        // ── one step, flat behind it — §7d's shape ────────────────────────────────
        ReliefPose {
            name: "reutlingen_albtrauf",
            lon: 9.2043,
            lat: 48.4914,
            agl_m: 21.0,
            bearing_deg: 135.0,
            below_deg: 1.0,
            family: Step,
            what: "the Albtrauf at 8 km, 40 km of plateau behind it",
        },
        ReliefPose {
            name: "stuttgart_kessel",
            lon: 9.1829,
            lat: 48.7758,
            agl_m: 145.0,
            bearing_deg: 180.0,
            below_deg: 1.0,
            family: Step,
            what: "the basin rim at 2.5 km, the Filder plateau behind it",
        },
        ReliefPose {
            name: "boulder_front_range",
            lon: -105.27,
            lat: 40.01,
            agl_m: 30.0,
            bearing_deg: 270.0,
            below_deg: 1.0,
            family: Step,
            what: "Boulder, west into the Front Range step",
        },
        ReliefPose {
            name: "hegau_step",
            lon: 8.85,
            lat: 47.78,
            agl_m: 30.0,
            bearing_deg: 180.0,
            below_deg: 1.0,
            family: Step,
            what: "the Hegau cones over the Bodensee basin",
        },
        // ── low relief in every direction ─────────────────────────────────────────
        ReliefPose {
            name: "po_plain_mantova",
            lon: 10.79,
            lat: 45.16,
            agl_m: 100.0,
            bearing_deg: 0.0,
            below_deg: 1.0,
            family: Plain,
            what: "the Po plain, north toward the distant Alps",
        },
        ReliefPose {
            name: "north_german_plain",
            lon: 10.00,
            lat: 53.00,
            agl_m: 100.0,
            bearing_deg: 0.0,
            below_deg: 1.0,
            family: Plain,
            what: "the north German plain — no relief at all",
        },
        ReliefPose {
            name: "kansas_plain",
            lon: -98.00,
            lat: 38.50,
            agl_m: 100.0,
            bearing_deg: 0.0,
            below_deg: 1.0,
            family: Plain,
            what: "the High Plains",
        },
        ReliefPose {
            name: "munich_foreland",
            lon: 11.58,
            lat: 48.14,
            agl_m: 200.0,
            bearing_deg: 180.0,
            below_deg: 1.0,
            family: Plain,
            what: "the Alpine foreland, south at the Alps 80 km off",
        },
        // ── water ─────────────────────────────────────────────────────────────────
        ReliefPose {
            name: "ligurian_sea",
            lon: 8.00,
            lat: 43.50,
            agl_m: 100.0,
            bearing_deg: 0.0,
            below_deg: 1.0,
            family: Coast,
            what: "open water, coast 60 km north",
        },
        ReliefPose {
            name: "nice_coast",
            lon: 7.26,
            lat: 43.70,
            agl_m: 100.0,
            bearing_deg: 0.0,
            below_deg: 2.0,
            family: Coast,
            what: "the Riviera, north into the Maritime Alps",
        },
        // ── relief below, camera above it — §7f's freeloaders ─────────────────────
        ReliefPose {
            name: "alps_above_600",
            lon: 10.985,
            lat: 47.10,
            agl_m: 600.0,
            bearing_deg: 0.0,
            below_deg: 8.0,
            family: Above,
            what: "§7f's `alps_approach`: 584 m over a 2.4 km ridge",
        },
        ReliefPose {
            name: "alps_above_1200",
            lon: 11.40,
            lat: 47.26,
            agl_m: 1_200.0,
            bearing_deg: 0.0,
            below_deg: 1.5,
            family: Above,
            what: "the Inn valley pose one rung above the gate",
        },
        ReliefPose {
            name: "alps_cruise_9km",
            lon: 11.00,
            lat: 46.60,
            agl_m: 9_000.0,
            bearing_deg: 0.0,
            below_deg: 10.0,
            family: Above,
            what: "§7f's `alps_cruise_11km`: the Alps from cruise",
        },
    ]
}

// ── the statistics ───────────────────────────────────────────────────────────────

/// Everything one settled tree says about the pre-check's candidates.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Stats {
    /// Visible leaves.
    pub(crate) tiles: usize,
    /// `max(node.hi) − eye`, metres. Relief as a height — the obvious statistic, and the
    /// one §7f's prose suggests.
    pub(crate) hi_above_eye_m: f64,
    /// `max elev(near, node.hi)`, degrees — relief as an **angle**, which is the same
    /// height with the distance divided back out.
    pub(crate) relief_hi_deg: f64,
    /// The same, off `max(floor_grid)` instead of `hi`: what height-aware bounds can *prove* stands there,
    /// i.e. the altitude the march would actually stamp.
    pub(crate) relief_floor_deg: f64,
    /// Ground range of the node `relief_floor_deg` was taken at, metres.
    pub(crate) wall_range_m: f64,
    /// Drawn tiles beyond that wall whose box top is under its elevation angle — an upper
    /// bound on what the march could cull, computed with one wall instead of 4 608 cells.
    pub(crate) shadowed: usize,
    /// The same count taken **per azimuth sector** instead of globally: twelve 30° sectors,
    /// each with its own wall, each counting only the tiles behind *its* wall.
    ///
    /// This is the statistic that asks the question the global one cannot — "is the wall
    /// in the same direction as the tiles it is supposed to hide" — and it is still one
    /// pass over the visible leaves, not a march.
    pub(crate) shadowed_sec: usize,
    /// Wall-clock cost of computing all of the above, microseconds.
    pub(crate) probe_us: f64,
}

/// One visible leaf, reduced to the three numbers every statistic above is built from.
struct Leaf {
    range_m: f64,
    /// Compass bearing of the node's nearest ground point, degrees.
    bearing_deg: f64,
    /// Elevation angle to the node's own `hi`, at its nearest ground point, degrees.
    elev_hi_deg: f64,
    /// The same, to the highest of its sixteen provable sub-cell floors.
    elev_floor_deg: f64,
    /// Upper bound on the elevation angle of every point of the node's height-aware box — exactly
    /// what `TerrainHorizon::occludes` tests, degrees.
    theta_box_deg: f64,
    hi_m: f64,
}

fn walk_visible(
    node: &QuadtreeNode<Heightfield>,
    eye: DVec3,
    up: DVec3,
    cam_lon: f64,
    cam_lat: f64,
    out: &mut Vec<Leaf>,
) {
    if !node.visible {
        return;
    }
    if let Some(children) = &node.children {
        for c in children.iter() {
            walk_visible(c, eye, up, cam_lon, cam_lat, out);
        }
        return;
    }
    let b = tile_bounds(&node.id);
    // Nearest point of the node's ground rectangle: the camera's own ground position
    // clamped into it. Same construction `TerrainHorizon::extent_of` uses, and for the
    // same reason — a node's *centre* range says nothing about how near its edge comes.
    let nlat = cam_lat.clamp(b.lat_min, b.lat_max);
    let nlon = {
        let c = 0.5 * (b.lon_min + b.lon_max);
        let half = 0.5 * (b.lon_max - b.lon_min);
        c + wrap_deg(cam_lon - c).clamp(-half, half)
    };
    let range_m = ground_range_m(cam_lon, cam_lat, nlon, nlat);
    let hi_m = node.extra.hi * 1.0e6;
    let floor_max_m = node
        .extra
        .floor_grid
        .iter()
        .fold(f32::NEG_INFINITY, |a, b| a.max(*b)) as f64
        * 1.0e6;
    out.push(Leaf {
        range_m,
        bearing_deg: {
            let dla = (nlat - cam_lat).to_radians();
            let dlo =
                wrap_deg(nlon - cam_lon).to_radians() * (0.5 * (nlat + cam_lat)).to_radians().cos();
            dlo.atan2(dla).to_degrees()
        },
        elev_hi_deg: elevation_deg(eye, up, ecef(nlon, nlat, hi_m)),
        elev_floor_deg: elevation_deg(eye, up, ecef(nlon, nlat, floor_max_m)),
        theta_box_deg: box_elevation_deg(&node.obb, eye, up),
        hi_m,
    });
}

/// The four candidate statistics, off one settled tree.
///
/// The `MIN_RANGE_M` cut on the two angular ones is not a tuning knob: the march's first
/// ring starts there, so nothing nearer can be an occluder at all, and without the cut the
/// tile the camera is *standing on* reports +90° at every pose that has any relief under
/// it.
pub(crate) fn stats_of(
    qt: &QuadtreeManager<Heightfield>,
    eye: DVec3,
    up: DVec3,
    cam_lon: f64,
    cam_lat: f64,
    eye_m: f64,
) -> Stats {
    let t0 = std::time::Instant::now();
    let mut leaves = Vec::with_capacity(256);
    for root in qt.roots.iter() {
        walk_visible(root, eye, up, cam_lon, cam_lat, &mut leaves);
    }
    let mut s = Stats {
        tiles: leaves.len(),
        hi_above_eye_m: f64::NEG_INFINITY,
        relief_hi_deg: f64::NEG_INFINITY,
        relief_floor_deg: f64::NEG_INFINITY,
        ..Stats::default()
    };
    for l in &leaves {
        s.hi_above_eye_m = s.hi_above_eye_m.max(l.hi_m - eye_m);
        if l.range_m >= MIN_RANGE_M {
            if l.elev_hi_deg > s.relief_hi_deg {
                s.relief_hi_deg = l.elev_hi_deg;
            }
            if l.elev_floor_deg > s.relief_floor_deg {
                s.relief_floor_deg = l.elev_floor_deg;
                s.wall_range_m = l.range_m;
            }
        }
    }
    s.shadowed = leaves
        .iter()
        .filter(|l| l.range_m > s.wall_range_m && l.theta_box_deg < s.relief_floor_deg)
        .count();

    // **The sector-local count.** Twelve 30° sectors; each leaf is filed under the sector
    // of its nearest point's bearing, each sector keeps its own wall, and a wall is spread
    // one sector either way so a leaf sitting on a boundary still shadows its neighbours.
    const SEC: usize = 12;
    let sector_of =
        |b: f64| -> usize { (((b + 360.0) / (360.0 / SEC as f64)).floor() as usize) % SEC };
    let mut wall = [f64::NEG_INFINITY; SEC];
    let mut wall_r = [f64::INFINITY; SEC];
    for l in &leaves {
        if l.range_m < MIN_RANGE_M {
            continue;
        }
        let a = sector_of(l.bearing_deg);
        if l.elev_floor_deg > wall[a] {
            wall[a] = l.elev_floor_deg;
            wall_r[a] = l.range_m;
        }
    }
    let mut spread = wall;
    let mut spread_r = wall_r;
    for a in 0..SEC {
        for d in [SEC - 1, 1] {
            let n = (a + d) % SEC;
            if wall[n] > spread[a] {
                spread[a] = wall[n];
                spread_r[a] = wall_r[n];
            }
        }
    }
    s.shadowed_sec = leaves
        .iter()
        .filter(|l| {
            let a = sector_of(l.bearing_deg);
            l.range_m > spread_r[a] && l.theta_box_deg < spread[a]
        })
        .count();
    s.probe_us = t0.elapsed().as_secs_f64() * 1.0e6;
    s
}

fn gates_open() -> TerrainOcclusionConfig {
    TerrainOcclusionConfig {
        enabled: true,
        // **Both gates opened on purpose.** The question here is what terrain occlusion removes, not
        // what the shipped gate lets it try to remove; a pose the AGL gate already shuts
        // off would otherwise read a flat zero for a reason that has nothing to do with
        // the statistic under test.
        max_camera_altitude_m: f32::INFINITY,
        max_camera_agl_m: f32::INFINITY,
        ..TerrainOcclusionConfig::default()
    }
}

// ── the tests ────────────────────────────────────────────────────────────────────

/// **The pose check, and it comes first** — §7c lost three poses to coordinates that were
/// a mountainside, §7f found two more that were not at the altitude their names said.
///
/// For every pose: the DEM's ground height, the eye's height above it, the horizon
/// `√(2Rh)`, the sightline profile's maximum along the declared bearing, and the heading
/// **measured off the built camera**.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn terrain_relief_poses_are_where_they_say() {
    const R: f64 = EARTH_RADIUS_A_F64 * 1.0e6;
    let mut world = RealWorld::new();
    let poses = relief_poses();

    // One batched download for every z14 tile the profiles below will read. `RealWorld`
    // fetches one tile per `curl` otherwise, and 29 poses × 21 samples of that is minutes
    // of round trips.
    let mut ids = Vec::new();
    for p in &poses {
        for km in PROFILE_KM {
            let (lo, la) = dest(p.lon, p.lat, p.bearing_deg, km * 1_000.0);
            let (fx, fy) = mercator_xy(lo, la, DEM_Z);
            ids.push(TileId {
                z: DEM_Z,
                x: fx.floor().max(0.0) as u32,
                y: fy.floor().max(0.0) as u32,
            });
        }
        let (fx, fy) = mercator_xy(p.lon, p.lat, DEM_Z);
        ids.push(TileId {
            z: DEM_Z,
            x: fx.floor().max(0.0) as u32,
            y: fy.floor().max(0.0) as u32,
        });
    }
    ids.sort_unstable_by_key(|i| (i.z, i.x, i.y));
    ids.dedup();
    println!(
        "  warming {} z{DEM_Z} tiles for {} poses",
        ids.len(),
        poses.len()
    );
    fetch_missing(&world.dir, &ids);

    println!(
        "  {:<22} {:>8} {:>7} {:>9} {:>9} {:>8} {:>9}",
        "pose", "ground", "agl", "eye", "horizon", "crest", "heading"
    );
    for pose in &poses {
        let ground = dem_height_m(&mut world, pose.lon, pose.lat);
        let eye_m = ground + pose.agl_m;
        let horizon_km = (2.0 * R * eye_m.max(1.0)).sqrt() / 1_000.0;
        let eye = ecef(pose.lon, pose.lat, eye_m);
        let up = normal_at(eye);
        let mut best = (0.0_f64, f64::NEG_INFINITY, 0.0_f64);
        for km in PROFILE_KM {
            let (lo, la) = dest(pose.lon, pose.lat, pose.bearing_deg, km * 1_000.0);
            let h = dem_height_m(&mut world, lo, la);
            let e = elevation_deg(eye, up, ecef(lo, la, h));
            if e > best.1 {
                best = (km, e, h);
            }
        }

        // The heading, measured rather than trusted — `ViewParams::yaw_deg` is composed
        // about the nadir axis and points the other way.
        let p = pose.view(ground);
        let cam = build_camera(&p);
        let (cam_eye, ori) = cam.global_transform_f64();
        let fwd = ori * DVec3::NEG_Z;
        let u = normal_at(cam_eye);
        let h = (fwd - u * fwd.dot(u)).normalize();
        let east = {
            let m = (u.x * u.x + u.z * u.z).sqrt();
            DVec3::new(u.z / m, 0.0, -u.x / m)
        };
        let north = u.cross(east).normalize();
        let heading = wrap_deg(h.dot(east).atan2(h.dot(north)).to_degrees());

        println!(
            "  {:<22} {ground:>7.0}m {:>6.0}m {eye_m:>8.0}m {horizon_km:>8.1}km \
             {:>4.0}km/{:>+5.2}d {:>8.1}d   {:?}  {}",
            pose.name, pose.agl_m, best.0, best.1, heading, pose.family, pose.what
        );
        assert!(
            wrap_deg(heading - pose.bearing_deg).abs() < 1.0,
            "{}: the camera looks along {heading:.1} deg, the pose declares {:.1}",
            pose.name,
            pose.bearing_deg
        );
        // The eye is over the ground by construction; what has to be checked is that the
        // DEM agrees the place exists and the horizon reaches past the near field the
        // statistic is about.
        assert!(
            horizon_km > 10.0,
            "{}: the horizon is only {horizon_km:.1} km",
            pose.name
        );
    }
}

/// Ranges the sightline profile is sampled at, kilometres.
const PROFILE_KM: [f64; 18] = [
    0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 70.0,
    100.0,
];

/// **The separation table.** What terrain occlusion removes at each pose, against what each candidate
/// pre-check would have read before the march ran.
///
/// Both altitude gates are opened, so the removal column is the stage's raw answer. The
/// statistics are taken off the **occlusion-off** tree, because that is the tree a pre-check sees
/// on the frame it has to decide on — and off the occlusion-on tree as well, because in
/// production the previous frame's visible set is the culled one and a pre-check that
/// reads differently on the two would oscillate.
#[test]
#[ignore = "measurement: needs the network for real height tiles, takes minutes"]
fn terrain_relief_statistic_separates_the_family() {
    let mut world = RealWorld::new();
    let only = std::env::var("CESIUM_RELIEF_POSE").ok();
    println!(
        "  {:<22} {:>6} {:>4} {:>4} {:>8} {:>9} {:>9} {:>8} {:>7} {:>6} {:>6} {:>7}",
        "pose",
        "agl",
        "off",
        "on",
        "removed",
        "hi-eye",
        "rel_hi",
        "rel_flr",
        "wall_km",
        "shdw",
        "shdwS",
        "probe"
    );
    let mut csv = Vec::new();
    for pose in relief_poses()
        .into_iter()
        .filter(|p| only.as_deref().map(|o| o == p.name).unwrap_or(true))
    {
        let ground = dem_height_m(&mut world, pose.lon, pose.lat);
        let eye_m = ground + pose.agl_m;
        let p = pose.view(ground);
        let frustum = frustum_for(&p);
        let eye = ecef(pose.lon, pose.lat, eye_m);
        let up = normal_at(eye);

        let off = settle(&p, &frustum, None, Fill::Visible, FRAMES, &mut world);
        let on = settle(
            &p,
            &frustum,
            Some(gates_open()),
            Fill::Visible,
            FRAMES,
            &mut world,
        );
        let t_off = off.qt.get_visible_tiles().len();
        let t_on = on.qt.get_visible_tiles().len();
        let s_off = stats_of(&off.qt, eye, up, pose.lon, pose.lat, eye_m);
        let s_on = stats_of(&on.qt, eye, up, pose.lon, pose.lat, eye_m);
        println!(
            "  {:<22} {:>5.0}m {t_off:>4} {t_on:>4} {:>8} {:>8.0}m {:>8.2}d {:>7.2}d {:>6.1}km              {:>6} {:>6} {:>6.1}u  {:?}",
            pose.name,
            pose.agl_m,
            t_off as i64 - t_on as i64,
            s_off.hi_above_eye_m,
            s_off.relief_hi_deg,
            s_off.relief_floor_deg,
            s_off.wall_range_m / 1_000.0,
            s_off.shadowed,
            s_off.shadowed_sec,
            s_off.probe_us,
            pose.family,
        );
        println!(
            "      {:<18} D3-on tree ({} leaves) reads hi-eye {:.0} m, rel_hi {:.2} d, \
             rel_flr {:.2} d, shadowed {} / {} | trace off {:?} on {:?}",
            "",
            s_on.tiles,
            s_on.hi_above_eye_m,
            s_on.relief_hi_deg,
            s_on.relief_floor_deg,
            s_on.shadowed,
            s_on.shadowed_sec,
            off.trace.iter().map(|t| t.0).collect::<Vec<_>>(),
            on.trace.iter().map(|t| t.0).collect::<Vec<_>>(),
        );
        csv.push(format!(
            "{},{:?},{:.0},{t_off},{t_on},{},{:.0},{:.3},{:.3},{},{},{:.3},{}",
            pose.name,
            pose.family,
            pose.agl_m,
            t_off as i64 - t_on as i64,
            s_off.hi_above_eye_m,
            s_off.relief_hi_deg,
            s_off.relief_floor_deg,
            s_off.shadowed,
            s_off.shadowed_sec,
            s_on.relief_floor_deg,
            s_on.shadowed_sec,
        ));
    }
    println!(
        "\n  csv: pose,family,agl_m,tiles_off,tiles_on,removed,off_hi_above_eye_m,\
         off_relief_hi_deg,off_relief_floor_deg,off_shadowed,off_shadowed_sec,\
         on_relief_floor_deg,on_shadowed_sec"
    );
    for row in &csv {
        println!("  csv: {row}");
    }
}

/// **The pre-check can only ever remove culls, and here is the whole set it removed.**
///
/// The soundness argument is one line — not marching is not culling, and height-aware bounds plus the relief-aware horizon test is the arm
/// the culling gate proves independently — but the rule is that a bound is measured
/// rather than assumed, so this measures it.
///
/// One settled tree per pose, then **two marches on that same tree**: one with the
/// pre-check disabled (`min_relief_deg = −∞`) and one with it
/// at its shipped threshold. Every node in the tree is then offered to both horizons, and
/// the claim is set inclusion:
///
/// > every node the pre-check arm culls is culled by the arm without it.
///
/// That is strictly stronger than "FN = 0 at these poses", because it holds node by node
/// rather than pose by pose and it does not depend on the oracle being dense enough. The
/// report prints both counts so a pose where the pre-check costs culls is visible as a
/// number rather than hidden behind a passing assertion.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn terrain_relief_pre_check_only_ever_removes_culls() {
    use cesium_engine::globe::quadtree::TerrainHorizon;

    fn walk(
        node: &QuadtreeNode<Heightfield>,
        off: &TerrainHorizon,
        on: &TerrainHorizon,
        counts: &mut (usize, usize, usize, usize),
    ) {
        let b = tile_bounds(&node.id);
        let c_off = off.occludes(&node.obb, &b);
        let c_on = on.occludes(&node.obb, &b);
        counts.0 += 1;
        counts.1 += c_off as usize;
        counts.2 += c_on as usize;
        // The one direction that would be a regression: a node the pre-check arm culls
        // and the full arm does not.
        counts.3 += (c_on && !c_off) as usize;
        if let Some(children) = &node.children {
            for c in children.iter() {
                walk(c, off, on, counts);
            }
        }
    }

    let mut world = RealWorld::new();
    let mut total = (0usize, 0usize, 0usize, 0usize);
    println!(
        "  {:<22} {:>7} {:>9} {:>9} {:>8} {:>9}",
        "pose", "nodes", "cull -pre", "cull +pre", "gained", "relief"
    );
    for pose in relief_poses() {
        let ground = dem_height_m(&mut world, pose.lon, pose.lat);
        let p = pose.view(ground);
        let frustum = frustum_for(&p);
        let cam_alt = p.alt_m * 1.0e-6;

        let mut s = settle(
            &p,
            &frustum,
            Some(no_pre_check()),
            Fill::Visible,
            FRAMES,
            &mut world,
        );
        s.qt.refresh_terrain_horizon(&frustum, cam_alt, cam_alt, &no_pre_check());
        let off = s.qt.terrain_horizon().expect("march").clone();
        s.qt.refresh_terrain_horizon(&frustum, cam_alt, cam_alt, &with_pre_check());
        let on = s.qt.terrain_horizon().expect("march").clone();

        let mut c = (0usize, 0usize, 0usize, 0usize);
        for root in s.qt.roots.iter() {
            walk(root, &off, &on, &mut c);
        }
        println!(
            "  {:<22} {:>7} {:>9} {:>9} {:>8} {:>9}",
            pose.name,
            c.0,
            c.1,
            c.2,
            c.3,
            if on.is_active() { "march" } else { "skipped" }
        );
        assert_eq!(
            c.3, 0,
            "{}: the pre-check arm culled {} node(s) the full march does not — \
             the pre-check is supposed to be a switch, not a second occluder",
            pose.name, c.3
        );
        total.0 += c.0;
        total.1 += c.1;
        total.2 += c.2;
        total.3 += c.3;
    }
    println!(
        "\n  {} nodes over {} poses: {} culled without the pre-check, {} with it, \
         {} culled only with it",
        total.0,
        relief_poses().len(),
        total.1,
        total.2,
        total.3
    );
    assert_eq!(total.3, 0);
    // A test that scored nothing proves nothing: the arm without the pre-check has to
    // have found culls for the subset claim to have content.
    assert!(
        total.1 > 0,
        "the control arm culled nothing at all — this test is vacuous"
    );
}

/// The shipped configuration, with the pre-check switched off — §7f's engine exactly.
fn no_pre_check() -> TerrainOcclusionConfig {
    TerrainOcclusionConfig {
        max_camera_altitude_m: f32::INFINITY,
        max_camera_agl_m: f32::INFINITY,
        min_relief_deg: f32::NEG_INFINITY,
        ..TerrainOcclusionConfig::default()
    }
}

/// The same, with the pre-check at its shipped threshold. Only the altitude gates are
/// opened, so the pre-check is the only thing that differs between the two arms.
fn with_pre_check() -> TerrainOcclusionConfig {
    TerrainOcclusionConfig {
        max_camera_altitude_m: f32::INFINITY,
        max_camera_agl_m: f32::INFINITY,
        ..TerrainOcclusionConfig::default()
    }
}

/// **What the pre-check costs, against what it is deciding about.**
///
/// The premise of the whole thing is that reading the visible set is worth a fraction of
/// walking the tree, and a premise is worth measuring. Both halves are timed on the same
/// clock, at every pose: `refresh_terrain_horizon` with the pre-check disabled — which is
/// the probe's cost plus the full march — against the same call with it shipped, which at
/// a pose that fails is the probe alone.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn terrain_relief_probe_is_cheap_against_the_march() {
    /// Timed repetitions, so one scheduling hiccup does not become the number.
    const REPS: usize = 40;
    let mut world = RealWorld::new();
    println!(
        "  {:<22} {:>7} {:>11} {:>11} {:>8}",
        "pose", "tiles", "march (min)", "probe (min)", "ratio"
    );
    let (mut worst_probe, mut worst_name) = (0.0f64, "");
    for pose in relief_poses() {
        let ground = dem_height_m(&mut world, pose.lon, pose.lat);
        let p = pose.view(ground);
        let frustum = frustum_for(&p);
        let cam_alt = p.alt_m * 1.0e-6;
        let mut s = settle(
            &p,
            &frustum,
            Some(no_pre_check()),
            Fill::Visible,
            FRAMES,
            &mut world,
        );

        // The march, with the probe forced to pass so the two arms differ by the march
        // alone. `min` over repetitions for the same reason `terrain_balance` takes it:
        // the least-interfered run is the one that measured the code.
        let mut march = f64::INFINITY;
        let mut probe = f64::INFINITY;
        let pass = TerrainOcclusionConfig {
            min_relief_deg: f32::NEG_INFINITY,
            ..with_pre_check()
        };
        let fail = TerrainOcclusionConfig {
            min_relief_deg: 89.0,
            ..with_pre_check()
        };
        for _ in 0..REPS {
            let t = std::time::Instant::now();
            s.qt.refresh_terrain_horizon(&frustum, cam_alt, cam_alt, &pass);
            march = march.min(t.elapsed().as_secs_f64() * 1.0e6);
            let t = std::time::Instant::now();
            s.qt.refresh_terrain_horizon(&frustum, cam_alt, cam_alt, &fail);
            probe = probe.min(t.elapsed().as_secs_f64() * 1.0e6);
        }
        let tiles = s.qt.get_visible_tiles().len();
        println!(
            "  {:<22} {tiles:>7} {march:>10.1}u {probe:>10.1}u {:>7.0}x",
            pose.name,
            march / probe.max(1.0e-9)
        );
        if probe > worst_probe {
            worst_probe = probe;
            worst_name = pose.name;
        }
    }
    println!("\n  worst probe: {worst_probe:.1} us at {worst_name}");
}
