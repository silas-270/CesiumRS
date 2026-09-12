//! Guards on the culling **stage list** itself, rather than on what it decides.
//!
//! Two properties live here, and neither had a test before the stages became data:
//!
//! * **I-7, as a checkable statement.** `CullPipeline`'s final rule is "a cascade
//!   that runs out of stages keeps", which makes every stage a pure *subtraction*
//!   from the kept set. [`test_stage_prefix_only_grows_the_kept_set`] asserts the
//!   consequence: for every pipeline `P` and every prefix `P'` of it,
//!   `kept(P) ⊆ kept(P')`. Soundness then never depends on a stage being present —
//!   only on each present stage being right — which is what lets stages be switched
//!   off at all.
//! * **I-4, cheaply.** [`test_horizon_hot_structs_have_not_grown`] pins the sizes of
//!   the two f64 structs the limb test runs on. It cannot see a wrong number, but
//!   it does see the one edit most likely to produce one.

use std::collections::HashSet;

use cesium_engine::globe::quadtree::{
    CullPipeline, Frustum, HorizonCamera, QuadtreeManager, Stage, TileId, TilePatch,
};

use super::cameras::{build_camera, ViewParams};
use super::cells;

/// The fast cells: the nadir altitude ladder plus the zoom-cliff ladder, 204 poses
/// spanning every altitude decade and deepest zoom 11..20.
fn probe_cells() -> Vec<ViewParams> {
    let mut v = cells::nadir_ladder();
    v.extend(cells::zoom_cliff_cells());
    v
}

fn frustum_for(p: &ViewParams) -> Frustum {
    let cam = build_camera(p);
    let aspect = p.aspect() as f32;
    let planes = cam.calculate_frustum_planes(aspect);
    let (eye, _) = cam.global_transform_f64();
    Frustum::planes_only(planes, eye).with_corners(cam.frustum_corners_relative(aspect))
}

/// Every ordered stage list without repetition, length 0 through 3 — 16 of them,
/// `CullPipeline::DEFAULT` among them and the empty pipeline (which culls nothing)
/// as the common prefix of them all.
fn all_pipelines() -> Vec<Vec<Stage>> {
    const ALL: [Stage; 3] = [Stage::Horizon, Stage::NodeFrustum, Stage::SubPatchGrid];
    let mut out = vec![Vec::new()];
    let mut frontier: Vec<Vec<Stage>> = vec![Vec::new()];
    for _ in 0..ALL.len() {
        let mut next = Vec::new();
        for seq in &frontier {
            for s in ALL {
                if seq.contains(&s) {
                    continue;
                }
                let mut ext = seq.clone();
                ext.push(s);
                next.push(ext);
            }
        }
        out.extend(next.iter().cloned());
        frontier = next;
    }
    out
}

/// The visible set after **one** update of a fresh quadtree.
///
/// One update, and from a fresh tree, on purpose. It is the only state in which the
/// LOD hysteresis cannot confound the comparison: every node still has
/// `children == None` when its own subdivision threshold is read, so the
/// subdivision decision is a pure function of the camera distance and is therefore
/// identical under every pipeline. (Across *several* updates it is not: a node a
/// stronger pipeline culled has had its children dropped, so on a later frame it
/// re-enters at the tighter `subdivide_dist` rather than the `collapse_dist` its
/// weaker sibling sees, and the two trees can legitimately differ in depth.)
fn kept_tiles(frustum: &Frustum, stages: &[Stage]) -> HashSet<TileId> {
    let mut qt = QuadtreeManager::new();
    qt.pipeline = CullPipeline::of(stages);
    qt.update(frustum);
    qt.get_visible_tiles().into_iter().map(|(id, _, _)| id).collect()
}

fn name(stages: &[Stage]) -> String {
    if stages.is_empty() {
        return "[]".to_string();
    }
    let parts: Vec<String> = stages.iter().map(|s| format!("{s:?}")).collect();
    format!("[{}]", parts.join(", "))
}

/// Dropping stages off the end of a pipeline may only **grow** the kept set.
///
/// This is invariant I-7 made mechanical. Every stage proves invisibility and
/// nothing else, so removing one removes proofs and adds none; the final rule
/// ("out of stages ⇒ keep") is what turns that into a subset property instead of a
/// coin flip.
///
/// # It does go red
///
/// Verified by hand, not assumed: with `CullPipeline::keeps` returning `false`
/// instead of `true` after its loop, this test fails on the very first cell — the
/// empty pipeline then keeps nothing while every longer pipeline keeps something,
/// so every one of the 15 non-empty pipelines reports tiles its prefix lost. With
/// the final rule restored it passes.
#[test]
fn test_stage_prefix_only_grows_the_kept_set() {
    let pipelines = all_pipelines();
    assert_eq!(pipelines.len(), 16, "expected every ordered stage list");
    assert!(pipelines.contains(&vec![
        Stage::Horizon,
        Stage::NodeFrustum,
        Stage::SubPatchGrid
    ]));

    let cells = probe_cells();
    let mut checked_pairs = 0usize;
    let mut grew = 0usize;

    for p in &cells {
        let frustum = frustum_for(p);
        let kept: Vec<HashSet<TileId>> = pipelines.iter().map(|s| kept_tiles(&frustum, s)).collect();

        for (i, stages) in pipelines.iter().enumerate() {
            for cut in 0..stages.len() {
                let prefix = &stages[..cut];
                let j = pipelines
                    .iter()
                    .position(|q| q.as_slice() == prefix)
                    .expect("every prefix of a stage list is itself a stage list");

                let lost: Vec<&TileId> = kept[i].difference(&kept[j]).collect();
                assert!(
                    lost.is_empty(),
                    "pipeline {} kept {} tile(s) that its prefix {} dropped \
                     (lat={} alt={}m pitch={} mode={}); first: {:?}. \
                     A prefix must keep at least as much: stages only ever prove \
                     invisibility, so removing one cannot lose a tile (I-7).",
                    name(stages),
                    lost.len(),
                    name(prefix),
                    p.lat_deg,
                    p.alt_m,
                    p.pitch_deg,
                    p.mode_name(),
                    lost[0],
                );
                checked_pairs += 1;
                if kept[j].len() > kept[i].len() {
                    grew += 1;
                }
            }
        }
    }

    // The property is vacuous if no prefix ever actually keeps more, which is what
    // a pipeline that silently ignored its stage list would look like.
    assert!(
        grew * 4 > checked_pairs,
        "only {grew} of {checked_pairs} prefix pairs kept strictly more tiles — \
         the stage list is barely doing anything, so the subset property proves little"
    );
    println!(
        "  [I-7 prefix] {} cells x {} pipelines, {checked_pairs} prefix pairs, \
         {grew} strictly larger",
        cells.len(),
        pipelines.len()
    );
}

/// I-4: the limb test's two structs are f64 end to end, and still this size.
///
/// A weak guard and stated as such — it cannot see a wrong *number*, only a changed
/// *shape*. What it does catch is the single most likely way I-4 gets broken in
/// practice: someone demotes these fields to f32 to "save memory" in the hottest
/// struct in the culler. `TilePatch` is 8 × f64 with no padding; `HorizonCamera` is
/// `DVec3 + 3 × f64 + bool`, i.e. 48 + 8 payload bytes rounded up to the 8-byte
/// alignment of its f64 fields.
///
/// The horizon test's conditioning near the surface scales as `1/h`: in f32 the
/// error in `S` is ~1.2e-7, which at 3 m altitude is 0.23° of limb angle — 26 km of
/// ground. Both structs stay f64. See `test_horizon_closed_form_matches_brute_force`
/// and `test_limb_band_has_no_false_negatives` for the guards that check the value.
#[test]
fn test_horizon_hot_structs_have_not_grown() {
    assert_eq!(
        std::mem::size_of::<TilePatch>(),
        64,
        "TilePatch must stay 8 x f64 (I-4)"
    );
    assert_eq!(
        std::mem::size_of::<HorizonCamera>(),
        56,
        "HorizonCamera must stay DVec3 + 3 x f64 + bool, all f64 (I-4)"
    );
}
