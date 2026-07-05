use glam::DVec3;
use crate::engine::property::sampled::SampledPositionProperty;
use crate::engine::time::SimulationTime;
use crate::engine::property::Property;
use crate::engine::render::polyline::builder::PolylineVertex;

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

        // Recursive build with max depth 16 and min time step 0.02s
        let root = Self::build_node(property, start_time.seconds, stop_time.seconds, p_start, p_end, 0, 16);
        
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
        let p_mid = property.evaluate(SimulationTime::new(t_mid)).unwrap_or_else(|| p_start.lerp(p_end, 0.5));

        // Geometric error calculation
        let line_vec = p_end - p_start;
        let length_sq = line_vec.length_squared();
        
        // Sample points to find max error and bounding sphere radius
        let num_samples = 10;
        let dt = (t_end - t_start) / num_samples as f64;
        
        let mut max_err = 0.0f64;
        let mut center = (p_start + p_end) * 0.5;
        let mut radius = (p_start - center).length();

        for i in 1..num_samples {
            let t = t_start + i as f64 * dt;
            if let Some(p) = property.evaluate(SimulationTime::new(t)) {
                // Update bounds
                let dist_to_center = (p - center).length();
                if dist_to_center > radius {
                    radius = dist_to_center; // Simplified sphere expansion
                }

                // Update geometric error from line segment
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

        // Subdivide if needed
        // We subdivide if error > 0.05 meters (5e-8 Megameters) and we haven't hit max depth
        // or min step size of 0.02s
        let mut children = None;
        if depth < max_depth && (t_end - t_start) > 0.02 && max_err > 5e-8 {
            let left = Self::build_node(property, t_start, t_mid, p_start, p_mid, depth + 1, max_depth);
            let right = Self::build_node(property, t_mid, t_end, p_mid, p_end, depth + 1, max_depth);
            
            // Re-adjust parent bounding sphere to encompass children
            let dist = (left.center - right.center).length();
            center = (left.center + right.center) * 0.5;
            radius = (dist * 0.5) + left.radius.max(right.radius);

            children = Some(Box::new([left, right]));
        }

        // Pad radius slightly for safety (e.g. 10km = 0.01 Megameters)
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

    pub fn collect_visible_segments(
        &self,
        camera_pos: DVec3,
        frustum_planes: &[(DVec3, f64); 6],
        max_screen_error: f64,
    ) -> Vec<Vec<(DVec3, f32)>> {
        let mut strips = Vec::new();
        let mut current_strip = Vec::new();
        self.traverse(&self.root, camera_pos, frustum_planes, max_screen_error, &mut strips, &mut current_strip);
        if current_strip.len() > 1 {
            strips.push(current_strip);
        }
        strips
    }

    fn traverse(
        &self,
        node: &PolylineNode,
        camera_pos: DVec3,
        frustum_planes: &[(DVec3, f64); 6],
        max_screen_error: f64,
        strips: &mut Vec<Vec<(DVec3, f32)>>,
        current_strip: &mut Vec<(DVec3, f32)>,
    ) {
        // Frustum Culling
        for (normal, d) in frustum_planes {
            let dist = normal.dot(node.center) + d;
            if dist < -node.radius {
                // If we cull a node, the line is broken. Save the current strip if it has points.
                if current_strip.len() > 1 {
                    strips.push(std::mem::take(current_strip));
                } else {
                    current_strip.clear();
                }
                return; // Fully outside
            }
        }

        // LOD check
        // distance in megameters. max(0.000001) is 1 meter min distance.
        let distance = (node.center - camera_pos).length().max(0.000001);
        let screen_error_approx = (node.max_geometric_error * 1732.0) / distance; // tuning factor

        if screen_error_approx <= max_screen_error || node.children.is_none() {
            // Render this node as a single segment
            let progress_start = if self.global_duration > 0.0 { ((node.t_start - self.global_start) / self.global_duration) as f32 } else { 0.0 };
            let progress_end = if self.global_duration > 0.0 { ((node.t_end - self.global_start) / self.global_duration) as f32 } else { 0.0 };
            
            if current_strip.is_empty() {
                current_strip.push((node.p_start, progress_start));
            } else if current_strip.last().unwrap().0.distance(node.p_start) > 1e-5 {
                // Disconnected! Break the strip.
                if current_strip.len() > 1 {
                    strips.push(std::mem::take(current_strip));
                } else {
                    current_strip.clear();
                }
                current_strip.push((node.p_start, progress_start));
            }
            current_strip.push((node.p_end, progress_end));
        } else {
            // Recurse
            let children = node.children.as_ref().unwrap();
            self.traverse(&children[0], camera_pos, frustum_planes, max_screen_error, strips, current_strip);
            self.traverse(&children[1], camera_pos, frustum_planes, max_screen_error, strips, current_strip);
        }
    }
}

pub fn generate_vertices(points: &[(DVec3, f32)], camera_pos: DVec3) -> Vec<PolylineVertex> {
    if points.is_empty() {
        return Vec::new();
    }

    // Calculate LOD distance based on the center of the strip
    let center = points.iter().fold(DVec3::ZERO, |acc, p| acc + p.0) / points.len() as f64;
    let dist = center.distance(camera_pos);
    let is_3d = dist < 0.2; // 200km threshold

    let mut vertices = if is_3d {
        Vec::with_capacity(points.len() * 8 + 6)
    } else {
        Vec::with_capacity(points.len() * 2)
    };
    
    // Helper closure to emit a single face strip
    let emit_strip = |verts: &mut Vec<PolylineVertex>, side_a: f32, v_side_a: f32, side_b: f32, v_side_b: f32, face: f32| {
        // Start cap (if this is the absolute start of the flight path)
        if !points.is_empty() && points[0].1 < 1e-5 {
            let (curr, prog) = points[0];
            let next = if points.len() > 1 { points[1].0 } else { curr + DVec3::X };
            let prev = curr + (curr - next).normalize_or_zero() * 1.0;
            let curr_f32 = [curr.x as f32, curr.y as f32, curr.z as f32];
            let prev_f32 = [prev.x as f32, prev.y as f32, prev.z as f32];
            let next_f32 = [next.x as f32, next.y as f32, next.z as f32];

            verts.push(PolylineVertex {
                position: curr_f32, previous: prev_f32, next: next_f32,
                side: side_a, v_side: v_side_a, face, progress: prog, forward: -1.0,
            });
            verts.push(PolylineVertex {
                position: curr_f32, previous: prev_f32, next: next_f32,
                side: side_b, v_side: v_side_b, face, progress: prog, forward: -1.0,
            });
        }

        for i in 0..points.len() {
            let (curr, prog) = points[i];
            let prev = if i > 0 { points[i - 1].0 } else { curr + (curr - points[i + 1].0).normalize_or_zero() * 1.0 };
            let next = if i < points.len() - 1 { points[i + 1].0 } else { curr + (curr - prev).normalize_or_zero() * 1.0 };

            let curr_f32 = [curr.x as f32, curr.y as f32, curr.z as f32];
            let prev_f32 = [prev.x as f32, prev.y as f32, prev.z as f32];
            let next_f32 = [next.x as f32, next.y as f32, next.z as f32];

            verts.push(PolylineVertex {
                position: curr_f32, previous: prev_f32, next: next_f32,
                side: side_a, v_side: v_side_a, face, progress: prog, forward: 0.0,
            });

            verts.push(PolylineVertex {
                position: curr_f32, previous: prev_f32, next: next_f32,
                side: side_b, v_side: v_side_b, face, progress: prog, forward: 0.0,
            });
        }

        // End cap (if this is the absolute end of the flight path)
        if !points.is_empty() && points.last().unwrap().1 > 1.0 - 1e-5 {
            let (curr, prog) = *points.last().unwrap();
            let prev = if points.len() > 1 { points[points.len() - 2].0 } else { curr + DVec3::X };
            let next = curr + (curr - prev).normalize_or_zero() * 1.0;
            let curr_f32 = [curr.x as f32, curr.y as f32, curr.z as f32];
            let prev_f32 = [prev.x as f32, prev.y as f32, prev.z as f32];
            let next_f32 = [next.x as f32, next.y as f32, next.z as f32];

            verts.push(PolylineVertex {
                position: curr_f32, previous: prev_f32, next: next_f32,
                side: side_a, v_side: v_side_a, face, progress: prog, forward: 1.0,
            });
            verts.push(PolylineVertex {
                position: curr_f32, previous: prev_f32, next: next_f32,
                side: side_b, v_side: v_side_b, face, progress: prog, forward: 1.0,
            });
        }
    };

    if !is_3d {
        // LOD 2D Ribbon: just emit the top face with v_side = 0.0
        emit_strip(&mut vertices, -1.0, 0.0, 1.0, 0.0, 0.0);
        return vertices;
    }

    // Top face (face 0.0): Left to Right
    emit_strip(&mut vertices, -1.0, 1.0, 1.0, 1.0, 0.0);
    vertices.push(*vertices.last().unwrap()); // degenerate break

    // Bottom face (face 1.0): Right to Left
    let v2 = vertices.len();
    emit_strip(&mut vertices, 1.0, -1.0, -1.0, -1.0, 1.0);
    vertices.insert(v2, vertices[v2]); // degenerate front break
    vertices.push(*vertices.last().unwrap()); // degenerate back break

    // Left face (face 2.0): Bottom to Top
    let v3 = vertices.len();
    emit_strip(&mut vertices, -1.0, -1.0, -1.0, 1.0, 2.0);
    vertices.insert(v3, vertices[v3]); // degenerate front break
    vertices.push(*vertices.last().unwrap()); // degenerate back break

    // Right face (face 3.0): Top to Bottom
    let v4 = vertices.len();
    emit_strip(&mut vertices, 1.0, 1.0, 1.0, -1.0, 3.0);
    vertices.insert(v4, vertices[v4]); // degenerate front break

    vertices
}
