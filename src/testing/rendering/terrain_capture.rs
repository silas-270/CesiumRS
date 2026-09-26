//! Headless captures for Phase C of `docs/terrain-plan.md` §6 — relief, on and off —
//! and, since D3, for §3.3's occlusion march as well.
//!
//! AGENTS.md requires that anything touching rendering or geometry be checked with
//! the headless path and *looked at*. Phase C changes every vertex of every tile, so
//! it qualifies: the numbers in `terrain::test_heightfield` say the heights and the
//! normals are right, and only a picture says whether the globe looks like a globe.
//!
//! Each pose is captured **three times** — terrain off, terrain on with D1+D2 only, and
//! terrain on with D3 as well — with nothing else different, so the set is a controlled
//! comparison rather than three screenshots. The third shot is where D3 is *looked at*
//! rather than counted: a piece of ground that vanishes between the second and the third
//! is a false negative, and no tile-count table can see it.
//!
//! `#[ignore]`d because it needs the network (imagery *and* height tiles) and writes
//! PNGs:
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots \
//!   cargo test --release --lib rendering::terrain_capture -- --ignored --nocapture
//! ```

use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, satellite_imagery_url};

/// Where the PNGs go. Set `CESIUM_SHOT_DIR`; defaults to the system temp dir.
pub(crate) fn shot_dir() -> std::path::PathBuf {
    let dir = std::env::var_os("CESIUM_SHOT_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("cesium_terrain_shots"));
    let _ = std::fs::create_dir_all(&dir);
    dir
}

const A: f64 = 6.378137;
const B: f64 = 6.3567523142;

fn ecef(lon_deg: f64, lat_deg: f64, alt_m: f64) -> glam::Vec3 {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(lon_deg, lat_deg, alt_m);
    glam::Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

fn up_at(lon_deg: f64, lat_deg: f64) -> glam::DVec3 {
    let p = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(lon_deg, lat_deg);
    glam::DVec3::new(p[0] / (A * A), p[1] / (B * B), p[2] / (A * A)).normalize()
}

fn north_at(lon_deg: f64, up: glam::DVec3) -> glam::DVec3 {
    let east = glam::DVec3::new(
        -lon_deg.to_radians().sin(),
        0.0,
        -lon_deg.to_radians().cos(),
    );
    up.cross(east).normalize()
}

pub(crate) struct Pose {
    pub(crate) name: &'static str,
    pub(crate) eye: glam::Vec3,
    pub(crate) target: glam::Vec3,
    pub(crate) up: Option<glam::Vec3>,
    pub(crate) what: &'static str,
}

/// An oblique view: stand `alt_m` up at (`lon`, `lat`) and look along the local north
/// bearing, pitched `pitch_deg` below the horizontal.
pub(crate) fn oblique(
    name: &'static str,
    lon: f64,
    lat: f64,
    alt_m: f64,
    pitch_deg: f64,
    reach_mm: f64,
    what: &'static str,
) -> Pose {
    let eye = ecef(lon, lat, alt_m);
    let up = up_at(lon, lat);
    let north = north_at(lon, up);
    let pitch = pitch_deg.to_radians();
    let dir = (north * pitch.cos() - up * pitch.sin()).normalize();
    Pose {
        name,
        eye,
        target: eye + glam::Vec3::new(dir.x as f32, dir.y as f32, dir.z as f32) * reach_mm as f32,
        up: Some(glam::Vec3::new(up.x as f32, up.y as f32, up.z as f32)),
        what,
    }
}

fn poses() -> Vec<Pose> {
    vec![
        // **The D3 pose.** Down on the Inn valley floor at 900 m, looking north into the
        // Karwendel wall 15 km away — the regime `docs/terrain-plan.md` §3.3 says the
        // occlusion march is the difference between drawing a mountain range and drawing
        // everything behind it too. The other four are Phase C/D1 poses and are kept as
        // they are, because their numbers are quoted.
        oblique(
            "alps_inn_valley",
            11.40,
            47.26,
            900.0,
            1.5,
            0.20,
            "Innsbruck and the Nordkette wall from the Inn valley at 900 m — D3's own regime",
        ),
        // The Zugspitze fixture region: 47.42 N 10.99 E, 2 962 m. Stand 70 km south of
        // it at 9 km and look north into the main alpine ridge.
        oblique(
            "alps_zugspitze",
            10.985,
            46.79,
            9_000.0,
            12.0,
            0.30,
            "Northern Limestone Alps from 9 km, 70 km south of the Zugspitze",
        ),
        // Closer and lower: the regime where relief is the whole picture.
        oblique(
            "alps_low",
            10.985,
            47.10,
            4_500.0,
            8.0,
            0.12,
            "the same ridge from 4.5 km, 35 km out — valley relief at z13-15",
        ),
        // Everest: 27.99 N 86.93 E, 8 849 m, the highest relief the source has.
        oblique(
            "himalaya_everest",
            86.925,
            27.35,
            11_000.0,
            11.0,
            0.30,
            "the Everest massif from 11 km, 70 km south",
        ),
        // High and wide: where the limb enters the frame and Phase C's still-flat
        // culling has to be looked at rather than measured.
        oblique(
            "himalaya_limb_400km",
            86.925,
            23.0,
            400_000.0,
            18.0,
            3.0,
            "the Himalaya against the limb from 400 km — the unsound-culling check",
        ),
    ]
}

fn config(terrain: bool, occlusion: bool) -> TileEngineConfig {
    TileEngineConfig {
        // Satellite imagery: relief against a dark vector basemap is legible only in
        // silhouette, and half of what C2 changes is the shading.
        base_imagery_url: satellite_imagery_url(),
        offline_mode: false,
        transparent_background: true,
        target_texel_ratio: 1.0,
        // The shipping default. C4 measures 32 and 64; it does not adopt them, and a
        // capture at a density the engine does not ship would not show what ships.
        mesh_segments: 16,
        terrain: TerrainConfig {
            enabled: terrain,
            exaggeration: 1.0,
            occlusion: cesium_engine::globe::quadtree::TerrainOcclusionConfig {
                enabled: occlusion,
                ..Default::default()
            },
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// Renders one pose, with its own settle loop.
///
/// Deliberately **not** `headless::routes_headless_app::run_headless_render`. That one
/// stops as soon as `last_missing_tiles_count` hits zero, and that counter only covers
/// the quadtree's *visible* set — not the fallback parents `get_renderable_tiles` also
/// draws, and not the height fetches still in flight behind a deferred mesh. With
/// terrain on it therefore captured a frame whose whole near field had no geometry yet.
/// The fix belongs here rather than in the shared helper, which the Phase A capture
/// poses are pinned against.
///
/// Settles on three conditions at once — no missing meshes, nothing loading anywhere
/// (imagery, meshes *and* heights), and a few quiet frames after that, because a mesh
/// finished on a rayon worker only reaches the GPU on the next `update_logic`.
pub(crate) async fn render_settled(
    width: u32,
    height: u32,
    config: TileEngineConfig,
    pose: &Pose,
    out_path: &str,
) -> (usize, usize) {
    use cesium_engine::render::wgpu_state::WgpuState;

    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(width, height)),
        config,
        None,
    )
    .await;

    match pose.up {
        Some(up) => state.camera.set_eye_with_up(pose.eye, pose.target, up),
        None => state.camera.set_eye(pose.eye, pose.target),
    }

    let aspect = state.size.width as f32 / state.size.height as f32;
    let view_proj = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
    let mut quiet = 0;
    let mut visible = state.update_logic(aspect, view_proj).len();
    loop {
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &[])
            .await;
        visible = state.update_logic(aspect, view_proj).len();

        let settled =
            state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
        quiet = if settled { quiet + 1 } else { 0 };
        if quiet >= 8 || std::time::Instant::now() >= deadline {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    let heights_resident = state
        .tile_system
        .height_manager
        .as_ref()
        .map(|h| h.residency().0)
        .unwrap_or(0);

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(out_path), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(out_path), false);
    res.expect("headless render");

    (visible, heights_resident)
}

#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_terrain_poses() {
    let dir = shot_dir();
    println!("  writing captures to {}", dir.display());

    for p in poses() {
        println!("  {} — {}", p.name, p.what);
        for (suffix, terrain, occlusion) in [
            ("terrain_off", false, false),
            ("terrain_on_d1d2", true, false),
            ("terrain_on_d3", true, true),
        ] {
            let out = dir.join(format!("{}_{suffix}.png", p.name));
            let out_str = out.to_string_lossy().into_owned();

            let (visible, heights) = pollster::block_on(render_settled(
                1280,
                720,
                config(terrain, occlusion),
                &p,
                &out_str,
            ));

            println!(
                "    {suffix:<12} {visible:4} visible tiles, {heights:3} height tiles \
                 -> {}",
                out.display()
            );
        }
    }
}
