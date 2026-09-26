//! **E1's picture** — the geometric LOD term, photographed (`docs/terrain-plan.md` §8).
//!
//! E1a's claim is one sentence: *at the same on-screen size, rugged ground refines and
//! flat ground does not.* `testing::terrain::test_terrain_lod` asserts it on synthetic
//! fields and counts it on real ones, and neither of those is a picture. This is the
//! picture.
//!
//! Three poses, each rendered **twice** with nothing different but
//! `TerrainConfig::max_geometric_error_px` — `0.0` is the pre-E1 engine, refining on
//! imagery sharpness alone; the shipped `12.0` adds the geometric half of the threshold:
//!
//! | pose | what it has to show |
//! |---|---|
//! | `e1_po_plain_to_alps` | **the controlled pair, in one frame.** The Po plain fills the foreground and the Alps stand on the horizon behind it, at the same altitude, in the same shot, under the same camera. If E1a works, the extra tiles land on the mountains and not on the plain. |
//! | `e1_alps_inn_valley` | rugged ground alone: the Karwendel wall, before and after. |
//! | `e1_bengal_flat` | the control: a flat delta coast at the same altitude and pitch, where the term must cost close to nothing. |
//!
//! The numeric twin of the middle row — the same pose, the same two budgets, with every
//! visible tile bucketed by its own measured error instead of drawn — is
//! `testing::terrain::test_terrain_lod::e1_the_extra_tiles_land_on_the_mountains`. It
//! needs no GPU, so it is where the claim is *counted*; this file is where it is *seen*.
//!
//! `#[ignore]`d for the same two reasons as `terrain_capture`: the network, and the PNGs.
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots \
//!   cargo test --release --lib rendering::terrain_e1_capture -- --ignored --nocapture
//! ```

use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, satellite_imagery_url};

use super::terrain_capture::{oblique, render_settled, shot_dir, Pose};

fn poses() -> Vec<Pose> {
    vec![
        // **The pose the whole section is about.** Over the Po plain at Vicenza, looking
        // due north: flat alluvium in the foreground, the Venetian Prealps and the
        // Dolomites standing on the horizon behind it. Both ground types, one camera, one
        // frame — so the comparison needs no argument about matched poses.
        //
        // **4 km, not the 300 m `test_terrain_lod`'s pose of the same name uses**, and the
        // reason is worth writing down because the first version of this pose got it
        // wrong: the horizon from 300 m is 62 km, the Prealps begin at 40 and the
        // Dolomites at 90, so the low pose photographs a plain with a hill line on the
        // skyline and no Alps in it at all. Whether a pose contains what its name says is
        // checkable before rendering — the horizon distance is `√(2Rh)` — and this one was
        // not checked until the picture came back.
        oblique(
            "e1_po_plain_to_alps",
            11.30,
            45.15,
            4_000.0,
            6.0,
            1.2,
            "flat plain in front, Alps behind — the extra tiles must land on the Alps",
        ),
        // Rugged ground on its own, at the pose §7b and §7c both quote.
        oblique(
            "e1_alps_inn_valley",
            11.40,
            47.26,
            900.0,
            1.5,
            0.3,
            "the Karwendel wall, with and without the geometric term",
        ),
        // The control. The Ganges delta at the same altitude and pitch as the Po pose:
        // `test_terrain_lod`'s level table measures this region at **0 m** of error at
        // every level from z7 down, so the term has nothing to ask for and the shot must
        // cost what the pre-E1 one cost.
        oblique(
            "e1_bengal_flat",
            88.50,
            21.80,
            300.0,
            1.0,
            0.6,
            "the control: flat delta, where the knob must be free",
        ),
    ]
}

/// `terrain_capture::config`'s terrain-on configuration with E1's knob as the variable.
///
/// D3 is left **on** throughout, unlike `terrain_capture`'s three-shot ladder: this
/// comparison is about `apply_lod`, and holding every culling stage fixed is what makes
/// the tile-count delta readable as refinement rather than as culling.
fn config(max_geometric_error_px: f32) -> TileEngineConfig {
    TileEngineConfig {
        base_imagery_url: satellite_imagery_url(),
        offline_mode: false,
        transparent_background: true,
        target_texel_ratio: 1.0,
        mesh_segments: 16,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            max_geometric_error_px,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_e1_geometric_term() {
    let dir = shot_dir();
    println!("  writing E1 captures to {}", dir.display());

    for p in poses() {
        println!("  {} — {}", p.name, p.what);
        for (suffix, budget) in [("geometric_off", 0.0f32), ("geometric_on", 12.0)] {
            let out = dir.join(format!("{}_{suffix}.png", p.name));
            let out_str = out.to_string_lossy().into_owned();
            let (visible, heights) =
                pollster::block_on(render_settled(1280, 720, config(budget), &p, &out_str));
            println!(
                "    {suffix:<14} {visible:4} visible tiles, {heights:3} height tiles -> {}",
                out.display()
            );
        }
    }
    println!(
        "  Read the pairs side by side: `geometric_on` must gain detail over relief and \
         none over the plain."
    );
}
