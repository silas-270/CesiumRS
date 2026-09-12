//! Exact convex-hull intersection ground truth for the analytic (Layer 1) tests.
//!
//! `Frustum::intersects_obb` is a *plane-by-plane* rejection test: it reports "no
//! intersection" only when the box is strictly outside one single plane. That is
//! conservative — it is the classic limitation of the separating-axis test
//! restricted to the face normals of one of the two hulls. A box can lie wholly
//! outside the frustum and yet not be outside any single plane (it pokes past a
//! frustum *corner*), and the plane test will call it visible.
//!
//! To measure that rather than assume it, this module implements a full, exact
//! separating-axis test between two convex hulls given as point sets:
//! candidate axes are both hulls' face normals plus every cross product of their
//! edge directions. For convex polytopes that axis set is complete, so a negative
//! answer here is genuine non-intersection and a positive answer is genuine
//! intersection.
//!
//! Everything is f64. The engine's own test runs in f32 — that difference is one
//! of the things Layer 1 measures.

use cesium_engine::globe::quadtree::OrientedBoundingBox;
use glam::{DMat4, DVec3, Vec3};

/// A convex hull described by its vertices, face normals and unique edge directions.
pub struct Hull {
    pub points: Vec<DVec3>,
    pub face_normals: Vec<DVec3>,
    pub edge_dirs: Vec<DVec3>,
}

impl Hull {
    /// Builds the hull of an oriented bounding box (its 8 corners).
    pub fn from_obb(center: DVec3, half_axes: [DVec3; 3]) -> Self {
        let mut points = Vec::with_capacity(8);
        for sx in [-1.0_f64, 1.0] {
            for sy in [-1.0_f64, 1.0] {
                for sz in [-1.0_f64, 1.0] {
                    points.push(
                        center + half_axes[0] * sx + half_axes[1] * sy + half_axes[2] * sz,
                    );
                }
            }
        }
        let dirs: Vec<DVec3> = half_axes
            .iter()
            .filter(|a| a.length_squared() > 0.0)
            .map(|a| a.normalize())
            .collect();
        Self {
            points,
            face_normals: dirs.clone(),
            edge_dirs: dirs,
        }
    }

    /// Builds the hull of a perspective frustum by unprojecting the eight NDC-cube
    /// corners with the inverse view-projection matrix.
    ///
    /// The engine uses **reverse-Z**, so the near plane is `ndc.z = 1` and the far
    /// plane is `ndc.z = 0`. A perspective frustum is the convex hull of these
    /// eight points, so SAT over them is exact.
    pub fn from_view_proj(view_proj: DMat4) -> Self {
        let inv = view_proj.inverse();
        let unproject = |x: f64, y: f64, z: f64| {
            let p = inv * glam::DVec4::new(x, y, z, 1.0);
            p.truncate() / p.w
        };

        // 0..3 = near quad (z = 1), 4..7 = far quad (z = 0), in matching order.
        let corner_ndc = [
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 1.0),
            (-1.0, 1.0),
        ];
        let mut points = Vec::with_capacity(8);
        for &(x, y) in &corner_ndc {
            points.push(unproject(x, y, 1.0));
        }
        for &(x, y) in &corner_ndc {
            points.push(unproject(x, y, 0.0));
        }

        // 12 edges: 4 on the near quad, 4 on the far quad, 4 lateral.
        let edges: [(usize, usize); 12] = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ];
        let mut edge_dirs = Vec::new();
        for (i, j) in edges {
            let d = points[j] - points[i];
            if d.length_squared() > 0.0 {
                push_unique_dir(&mut edge_dirs, d.normalize());
            }
        }

        // Face normals: near, far and the four side faces, from three corners each.
        let faces: [[usize; 3]; 6] = [
            [0, 1, 2], // near
            [4, 6, 5], // far
            [0, 4, 5], // bottom
            [2, 6, 7], // top
            [0, 3, 7], // left
            [1, 5, 6], // right
        ];
        let mut face_normals = Vec::new();
        for [a, b, c] in faces {
            let n = (points[b] - points[a]).cross(points[c] - points[a]);
            if n.length_squared() > 0.0 {
                push_unique_dir(&mut face_normals, n.normalize());
            }
        }

        Self {
            points,
            face_normals,
            edge_dirs,
        }
    }

    fn project(&self, axis: DVec3) -> (f64, f64) {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for p in &self.points {
            let v = p.dot(axis);
            lo = lo.min(v);
            hi = hi.max(v);
        }
        (lo, hi)
    }
}

fn push_unique_dir(v: &mut Vec<DVec3>, d: DVec3) {
    // Directions are unsigned for SAT purposes: d and -d give the same axis.
    if v.iter().any(|e| (e.dot(d)).abs() > 1.0 - 1.0e-12) {
        return;
    }
    v.push(d);
}

/// Exact convex/convex intersection in f64.
///
/// `tolerance` is applied to the overlap test: a gap narrower than `tolerance`
/// still counts as touching, so hulls that share a face are reported as
/// intersecting rather than flipping on rounding.
pub fn hulls_intersect(a: &Hull, b: &Hull, tolerance: f64) -> bool {
    let mut axes: Vec<DVec3> = Vec::with_capacity(
        a.face_normals.len() + b.face_normals.len() + a.edge_dirs.len() * b.edge_dirs.len(),
    );
    axes.extend_from_slice(&a.face_normals);
    axes.extend_from_slice(&b.face_normals);
    for ea in &a.edge_dirs {
        for eb in &b.edge_dirs {
            let c = ea.cross(*eb);
            if c.length_squared() > 1.0e-18 {
                axes.push(c.normalize());
            }
        }
    }

    for axis in axes {
        let (a_lo, a_hi) = a.project(axis);
        let (b_lo, b_hi) = b.project(axis);
        if a_hi < b_lo - tolerance || b_hi < a_lo - tolerance {
            return false; // separating axis found
        }
    }
    true
}

/// Convenience: promote an engine `OrientedBoundingBox` (f32) to an f64 hull.
pub fn obb_hull(obb: &OrientedBoundingBox) -> Hull {
    Hull::from_obb(
        DVec3::new(
            obb.center.x as f64,
            obb.center.y as f64,
            obb.center.z as f64,
        ),
        [
            to_d(obb.half_axes[0]),
            to_d(obb.half_axes[1]),
            to_d(obb.half_axes[2]),
        ],
    )
}

fn to_d(v: Vec3) -> DVec3 {
    DVec3::new(v.x as f64, v.y as f64, v.z as f64)
}
