use cesium_engine::core::extension::GlobeExtension;
use cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use cesium_engine::render::polyline_pipeline::{
    builder::ControlPoint,
    pipeline::{DrawParams, PolylineConfig, PolylineRenderer},
};
use glam::DVec3;

pub struct Route {
    pub start: super::api::LatLon,
    pub end: super::api::LatLon,
}

pub struct RouteEntity {
    pub renderer: Option<PolylineRenderer>,
    pub control_points: Vec<ControlPoint>,
    pub reference_point: DVec3,
    pub config: PolylineConfig,
}

pub struct RoutesExtension {
    pub routes: Vec<RouteEntity>,
}

impl RoutesExtension {
    pub fn new(input_routes: &[super::api::HeadlessRoute]) -> Self {
        let mut routes = Vec::with_capacity(input_routes.len());

        for route in input_routes {
            let start = route.start;
            let end = route.end;

            let altitude_meters = 10000.0; // Constant altitude matching real flights

            let p1 = lon_lat_alt_to_ecef_f64(start.lon, start.lat, altitude_meters);
            let p2 = lon_lat_alt_to_ecef_f64(end.lon, end.lat, altitude_meters);
            let u1 = DVec3::from_array(p1).normalize();
            let u2 = DVec3::from_array(p2).normalize();
            
            let reference_point = DVec3::new(
                (p1[0] + p2[0]) / 2.0,
                (p1[1] + p2[1]) / 2.0,
                (p1[2] + p2[2]) / 2.0,
            );

            let dot = u1.dot(u2).clamp(-1.0, 1.0);
            let omega = dot.acos();
            let sin_omega = omega.sin();

            let num_segments = 64;
            let mut control_points = Vec::with_capacity(num_segments + 1);

            for i in 0..=num_segments {
                let t = i as f64 / num_segments as f64;
                let pos = if sin_omega < 1e-6 {
                    u1.lerp(u2, t).normalize() * DVec3::from_array(p1).length()
                } else {
                    let a = ((1.0 - t) * omega).sin() / sin_omega;
                    let b = (t * omega).sin() / sin_omega;
                    let dir = (u1 * a + u2 * b).normalize();
                    let r1 = DVec3::from_array(p1).length();
                    let r2 = DVec3::from_array(p2).length();
                    dir * (r1 + (r2 - r1) * t)
                };

                let rel = pos - reference_point;
                control_points.push(ControlPoint {
                    position: [rel.x as f32, rel.y as f32, rel.z as f32],
                    progress: t as f32,
                });
            }

            let config = PolylineConfig {
                physical_half_width: 2.98 / 1_000_000.0,
                color_start: [1.0, 0.4, 0.0, 1.0],
                color_end: [0.9, 0.9, 0.9, 1.0],
                split_progress: -1.0,
                ..Default::default()
            };

            routes.push(RouteEntity {
                renderer: None,
                control_points,
                reference_point,
                config,
            });
        }

        Self { routes }
    }
}

impl GlobeExtension for RoutesExtension {
    fn init(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) {
        for entity in &mut self.routes {
            let mut renderer = PolylineRenderer::new(device, config, camera_bind_group_layout);
            renderer.update_geometry(device, queue, &entity.control_points);
            entity.renderer = Some(renderer);
        }
    }

    fn update(
        &mut self,
        _device: &wgpu::Device,
        _queue: &wgpu::Queue,
        _camera_pos_dvec3: DVec3,
        _frustum: &[(DVec3, f64); 6],
        _camera: &mut cesium_engine::camera::camera::Camera,
        _aspect_ratio: f32,
    ) {
        // Nothing to update for static routes
    }

    fn render<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        camera_bind_group: &'a wgpu::BindGroup,
        viewport_size: [f32; 2],
        camera_pos_f64: [f64; 3],
    ) {
        for entity in &self.routes {
            if let Some(renderer) = &entity.renderer {
                renderer.draw(DrawParams {
                    render_pass,
                    camera_bind_group,
                    viewport_size,
                    camera_pos_f64,
                    reference_point: entity.reference_point.to_array(),
                    config: &entity.config,
                });
            }
        }
    }
}
