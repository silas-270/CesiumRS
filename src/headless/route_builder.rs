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

            // Use the new telemetry generator
            let points = cesium_flight::telemetry::generate(
                start.lon,
                start.lat,
                end.lon,
                end.lat,
                3600_000, // Dummy duration, headless doesn't care about time
                None,
                None,
            );

            // Compute the average reference point for precision
            let mut p_sum = DVec3::ZERO;
            let mut count = 0;
            for pt in &points {
                let ecef = lon_lat_alt_to_ecef_f64(pt.longitude, pt.latitude, pt.altitude);
                p_sum += DVec3::from_array(ecef);
                count += 1;
            }
            let reference_point = p_sum / count as f64;

            let mut control_points = Vec::with_capacity(points.len());
            for (i, pt) in points.iter().enumerate() {
                let ecef = lon_lat_alt_to_ecef_f64(pt.longitude, pt.latitude, pt.altitude);
                let pos = DVec3::from_array(ecef);
                let rel = pos - reference_point;
                let t = i as f32 / (points.len() - 1) as f32;
                control_points.push(ControlPoint {
                    position: [rel.x as f32, rel.y as f32, rel.z as f32],
                    progress: t,
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
