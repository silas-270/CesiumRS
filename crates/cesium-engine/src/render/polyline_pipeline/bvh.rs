#![allow(clippy::unnecessary_unwrap)]
use crate::property::sampled::SampledPositionProperty;
use crate::property::Property;
use crate::render::polyline_pipeline::builder::ControlPoint;
use crate::time::SimulationTime;
use glam::DVec3;

// ── BVH node ──────────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct PolylineNode {
    pub t_start: f64,
    pub t_end: f64,
    pub p_start: DVec3,
    pub p_end: DVec3,
    pub center: DVec3,
    pub radius: f64,
    pub max_geometric_error: f64,
    pub children: Option<Box<[PolylineNode; 2]>>,
}

// ── BVH ───────────────────────────────────────────────────────────────────────

pub struct PolylineBVH {
    pub root: PolylineNode,
    pub global_start: f64,
    pub global_duration: f64,
}

impl PolylineBVH {
    pub fn build(property: &SampledPositionProperty) -> Option<Self> {
        let start_time = property.start_time()?;
        let stop_time = property.stop_time()?;

        if start_time.seconds >= stop_time.seconds {
            return None;
        }

        let p_start = property.evaluate(start_time)?;
        let p_end = property.evaluate(stop_time)?;

        let root = Self::build_node(
            property,
            start_time.seconds,
            stop_time.seconds,
            p_start,
            p_end,
            0,
            16,
        );

        Some(Self {
            root,
            global_start: start_time.seconds,
            global_duration: stop_time.seconds - start_time.seconds,
        })
    }

    fn build_node(
        property: &SampledPositionProperty,
        t_start: f64,
        t_end: f64,
        p_start: DVec3,
        p_end: DVec3,
        depth: u32,
        max_depth: u32,
    ) -> PolylineNode {
        let t_mid = (t_start + t_end) * 0.5;
        let p_mid = property
            .evaluate(SimulationTime::new(t_mid))
            .unwrap_or_else(|| p_start.lerp(p_end, 0.5));

        let line_vec = p_end - p_start;
        let length_sq = line_vec.length_squared();

        let num_samples = 10;
        let dt = (t_end - t_start) / num_samples as f64;

        let mut max_err = 0.0f64;
        let mut center = (p_start + p_end) * 0.5;
        let mut radius = (p_start - center).length();

        for i in 1..num_samples {
            let t = t_start + i as f64 * dt;
            if let Some(p) = property.evaluate(SimulationTime::new(t)) {
                let dist_to_center = (p - center).length();
                if dist_to_center > radius {
                    radius = dist_to_center;
                }
                let dist_to_line = if length_sq < 1e-8 {
                    (p - p_start).length()
                } else {
                    let proj_t = ((p - p_start).dot(line_vec) / length_sq).clamp(0.0, 1.0);
                    let projection = p_start + line_vec * proj_t;
                    (p - projection).length()
                };
                if dist_to_line > max_err {
                    max_err = dist_to_line;
                }
            }
        }

        let mut children = None;
        if depth < max_depth && (t_end - t_start) > 0.02 && max_err > 5e-8 {
            let left =
                Self::build_node(property, t_start, t_mid, p_start, p_mid, depth + 1, max_depth);
            let right =
                Self::build_node(property, t_mid, t_end, p_mid, p_end, depth + 1, max_depth);

            let dist = (left.center - right.center).length();
            center = (left.center + right.center) * 0.5;
            radius = (dist * 0.5) + left.radius.max(right.radius);

            children = Some(Box::new([left, right]));
        }

        radius += max_err + 0.01;

        PolylineNode {
            t_start,
            t_end,
            p_start,
            p_end,
            center,
            radius,
            max_geometric_error: max_err,
            children,
        }
    }

    /// Traverse the BVH and collect a flat list of `ControlPoint`s for all
    /// visible segments, ready for upload to the GPU.
    ///
    /// Strip breaks (where the path exits and re-enters the frustum) are
    /// represented by duplicate/degenerate points — the vertex shader will
    /// produce zero-area triangles for these, which the rasteriser discards.
    pub fn collect_visible_segments(
        &self,
        camera_pos: DVec3,
        frustum_planes: &[(DVec3, f64); 6],
        max_screen_error: f64,
        reference_point: DVec3,
    ) -> Vec<ControlPoint> {
        let mut out = Vec::new();
        let mut last_was_break = false;
        self.traverse(
            &self.root,
            camera_pos,
            frustum_planes,
            max_screen_error,
            reference_point,
            &mut out,
            &mut last_was_break,
        );
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn traverse(
        &self,
        node: &PolylineNode,
        camera_pos: DVec3,
        frustum_planes: &[(DVec3, f64); 6],
        max_screen_error: f64,
        reference_point: DVec3,
        out: &mut Vec<ControlPoint>,
        last_was_break: &mut bool,
    ) {
        // Frustum culling
        for (normal, d) in frustum_planes {
            let dist = normal.dot(node.center) + d;
            if dist < -node.radius {
                // Mark that we have a discontinuity in the ribbon.
                *last_was_break = true;
                return;
            }
        }

        // LOD check without sqrt
        let distance_sq = (node.center - camera_pos).length_squared().max(1e-12);
        let max_geom_err_sq = node.max_geometric_error * node.max_geometric_error;
        let screen_error_sq = (max_geom_err_sq * 2999824.0) / distance_sq; // 1732.0^2 = 2999824.0
        let max_screen_error_sq = max_screen_error * max_screen_error;

        if screen_error_sq <= max_screen_error_sq || node.children.is_none() {
            // Emit this leaf segment
            let prog_start = self.to_progress(node.t_start);
            let prog_end = self.to_progress(node.t_end);

            let p_start_rel = node.p_start - reference_point;
            let p_end_rel = node.p_end - reference_point;

            let cp_start = ControlPoint {
                position: [p_start_rel.x as f32, p_start_rel.y as f32, p_start_rel.z as f32],
                progress: prog_start,
            };
            let cp_end = ControlPoint {
                position: [p_end_rel.x as f32, p_end_rel.y as f32, p_end_rel.z as f32],
                progress: prog_end,
            };

            if *last_was_break && !out.is_empty() {
                // Insert degenerate bridge: duplicate last + new start.
                // The GPU rasteriser will produce zero-area triangles here.
                let last = *out.last().unwrap();
                out.push(last);
                out.push(cp_start);
            }
            *last_was_break = false;

            if out.is_empty() {
                out.push(cp_start);
            }

            out.push(cp_end);
        } else {
            // Recurse into children
            let children = node.children.as_ref().unwrap();
            self.traverse(
                &children[0],
                camera_pos,
                frustum_planes,
                max_screen_error,
                reference_point,
                out,
                last_was_break,
            );
            self.traverse(
                &children[1],
                camera_pos,
                frustum_planes,
                max_screen_error,
                reference_point,
                out,
                last_was_break,
            );
        }
    }

    fn to_progress(&self, t: f64) -> f32 {
        if self.global_duration > 0.0 {
            ((t - self.global_start) / self.global_duration).clamp(0.0, 1.0) as f32
        } else {
            0.0
        }
    }
}
