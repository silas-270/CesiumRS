//! **F5's picture** — the four levels below the source ceiling, photographed
//! (`docs/terrain-plan.md` §9 F5).
//!
//! F5's claim is one sentence: *below the z15 source ceiling the mesh is still a
//! decimation of data the engine already holds, and E1's `DETAIL_MAX_Z = 15` reported that
//! as zero error.* `testing::terrain::test_detail_below_ceiling` measures the reserve and
//! costs it. This is where it is seen — and where the honest size of it shows.
//!
//! # The poses are approaches, not the ten of §7b
//!
//! Every table before this one is measured on `real_poses`: forward views pitched 1-18°
//! below the horizontal from 300 m to 400 km, chosen for D3, which is a question about the
//! far field. There is no near field in them — the settled tree at `alps_inn_valley`
//! bottoms out at z15 and stays there whatever F5 does, which is exactly what the cost
//! table shows. z16-z19 live in front of an aircraft on final, a few hundred metres up and
//! pitched into the slope, so that is what these four poses are. Each sits over ground
//! whose DEM height was looked up first (`test_detail_below_ceiling::approach_poses`, and
//! the gate test beside it).
//!
//! # Two budgets, because the reserve and the budget are different questions
//!
//! Every pose is rendered **four** times: `detail_max_z` at E1's 15 and F5's 19, each at
//! `max_geometric_error_px` 12 (shipped) and 8.
//!
//! At 12 px the pair differs by **one level** and very little picture, and that is the
//! finding rather than a disappointment: by z16 the measured error has already fallen
//! under a 12 px budget, so the term stops asking. At 8 px the same change reaches z17 and
//! the near field visibly gains, while E1's ceiling — which forbids any error below z15 —
//! cannot spend the tighter budget on the ground in front of the aircraft at all. The pair
//! to read is therefore the **8 px** pair; the 12 px pair is the control that says how
//! little the shipped knob currently asks for.
//!
//! `#[ignore]`d for the same two reasons as its siblings: the network, and the PNGs.
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots \
//!   cargo test --release --lib rendering::terrain_f5_capture -- --ignored --nocapture
//! ```

use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, satellite_imagery_url};

use super::terrain_capture::{oblique, render_settled, shot_dir, Pose};

const WIDTH: u32 = 1280;
const HEIGHT: u32 = 720;

/// The four approaches of `testing::terrain::test_detail_below_ceiling::approach_poses`, as
/// `terrain_capture` poses.
///
/// Same longitude, latitude, altitude and bearing; `oblique` takes the pitch **below the
/// horizontal** directly, where `ViewParams::pitch_deg` measures from nadir, which is the
/// same 90° offset `test_terrain_occlusion::real_poses` documents. `reach_mm` only has to
/// be non-degenerate — it sets the look-at point along the ray, not the view.
fn poses() -> Vec<Pose> {
    vec![
        oblique(
            "f5_lowi_final",
            11.35,
            47.255,
            1_000.0,
            12.0,
            0.05,
            "Innsbruck on final, 421 m AGL, down the Inn — near field is valley floor \
             and the foot of the Nordkette",
        ),
        oblique(
            "f5_samedan_final",
            9.884,
            46.534,
            2_200.0,
            14.0,
            0.05,
            "Samedan, 500 m AGL over the Engadin floor, into Piz Nair",
        ),
        oblique(
            "f5_lukla_final",
            86.731,
            27.687,
            3_400.0,
            16.0,
            0.05,
            "Lukla, 557 m AGL, into the Dudh Kosi gorge — the steepest near field there is",
        ),
        oblique(
            "f5_aosta_final",
            7.368,
            45.738,
            1_100.0,
            14.0,
            0.05,
            "Aosta, 559 m AGL, towards the Grand Combin side",
        ),
    ]
}

/// `terrain_e1_capture::config` with F5's two knobs as the variables and everything else
/// held where that file holds it — D3 on, `mesh_segments` 16, Esri imagery, so the shots
/// sit beside §8's rather than beside them.
fn config(detail_max_z: u8, max_geometric_error_px: f32) -> TileEngineConfig {
    TileEngineConfig {
        base_imagery_url: satellite_imagery_url(),
        offline_mode: false,
        transparent_background: true,
        target_texel_ratio: 1.0,
        mesh_segments: 16,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            detail_max_z,
            max_geometric_error_px,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// Fraction of pixels that differ between two frames, sampled every second pixel in each
/// axis — `terrain_e2_capture::differing_fraction`'s rule, repeated so §9's percentages are
/// comparable with §8's.
fn differing_fraction(a: &[u8], b: &[u8]) -> f64 {
    let mut differ = 0usize;
    let mut total = 0usize;
    for y in (0..HEIGHT as usize).step_by(2) {
        for x in (0..WIDTH as usize).step_by(2) {
            let i = (y * WIDTH as usize + x) * 4;
            total += 1;
            if a[i..i + 4] != b[i..i + 4] {
                differ += 1;
            }
        }
    }
    differ as f64 / total.max(1) as f64
}

/// The same fraction per horizontal band, six bands top to bottom.
///
/// Band 0 is the sky and the far skyline; band 5 is the ground closest to the aircraft.
///
/// The bands are what turn the percentage into an argument, and the argument they make is
/// **not** "F5's pixels are at the bottom". What the measured runs show is that the pixels
/// land wherever the *shape* is, which on an approach is the valley walls rather than the
/// floor in front of the wheels: at `f5_lowi_final` the bottom half — runway, apron, the
/// flat Inn floor — is bit-identical in both ceilings, because flat ground has no error for
/// any budget to spend, while bands 1-2, the Nordkette slope, move by 54 % and 24 %. At
/// `f5_lukla_final`, where the gorge wall fills the frame from top to bottom, every band
/// moves. That is E1a's "the extra tiles land on the mountains", one ceiling deeper.
fn bands(a: &[u8], b: &[u8]) -> [f64; 6] {
    let mut out = [0.0f64; 6];
    let rows = HEIGHT as usize / 6;
    for (band, slot) in out.iter_mut().enumerate() {
        let (y0, y1) = (band * rows, ((band + 1) * rows).min(HEIGHT as usize));
        let mut differ = 0usize;
        let mut total = 0usize;
        for y in (y0..y1).step_by(2) {
            for x in (0..WIDTH as usize).step_by(2) {
                let i = (y * WIDTH as usize + x) * 4;
                total += 1;
                if a[i..i + 4] != b[i..i + 4] {
                    differ += 1;
                }
            }
        }
        *slot = differ as f64 / total.max(1) as f64;
    }
    out
}

fn read_png(path: &std::path::Path) -> Option<Vec<u8>> {
    let img = image::open(path).ok()?.to_rgba8();
    Some(img.into_raw())
}

#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_f5_below_the_source_ceiling() {
    let dir = shot_dir();
    println!("  writing F5 captures to {}", dir.display());

    for p in poses() {
        println!("\n  {} — {}", p.name, p.what);
        for px in [12.0f32, 8.0] {
            let mut frames: Vec<(u8, usize, Option<Vec<u8>>)> = Vec::new();
            for max_z in [15u8, 19] {
                let label = if max_z == 15 {
                    "e1_ceiling"
                } else {
                    "f5_ceiling"
                };
                let out = dir.join(format!("{}_{px:.0}px_{label}.png", p.name));
                let out_str = out.to_string_lossy().into_owned();
                let (visible, heights) = pollster::block_on(render_settled(
                    WIDTH,
                    HEIGHT,
                    config(max_z, px),
                    &p,
                    &out_str,
                ));
                println!(
                    "    {px:>2.0} px  detail_max_z {max_z:<3} {visible:4} visible tiles, \
                     {heights:3} height tiles -> {}",
                    out.display()
                );
                frames.push((max_z, visible, read_png(&out)));
            }

            if let [(_, n15, Some(a)), (_, n19, Some(b))] = &frames[..] {
                let d = differing_fraction(a, b);
                let bs = bands(a, b);
                println!(
                    "    {px:>2.0} px  tiles {n15} -> {n19} ({:+}), pixels differing {:.2} %",
                    *n19 as i64 - *n15 as i64,
                    100.0 * d
                );
                print!("    {px:>2.0} px  by band, sky to near ground:");
                for v in bs {
                    print!(" {:.2}%", 100.0 * v);
                }
                println!();
            }
        }
    }

    println!(
        "\n  Read each 8 px pair side by side, and read the band row rather than the total. \
         A band that reads 0.00 % is ground with no shape left to resolve — flat valley \
         floor, or a slope the tree already refines on imagery — and is the control, not a \
         failure. A pose whose every band reads 0.00 % never reached z16 at all: F5 removes \
         a ceiling, it does not add a demand."
    );
}
