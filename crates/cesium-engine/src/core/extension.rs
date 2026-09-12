use glam::DVec3;

pub trait GlobeExtension {
    /// Called during engine initialization to load pipelines and resources
    fn init(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    );

    fn update(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        camera_pos_dvec3: DVec3,
        // The four side-plane inward unit normals, [Left, Right, Bottom, Top],
        // from `Camera::calculate_frustum_planes`. Camera-relative: all four pass
        // through the eye, so there are no offsets. See `docs/culling-math.md` §2.
        frustum: &[DVec3; 4],
        camera: &mut crate::camera::camera::Camera,
        aspect_ratio: f32,
    );

    /// Called every frame after the globe and engine debug models are drawn
    fn render<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        camera_bind_group: &'a wgpu::BindGroup,
        viewport_size: [f32; 2],
        camera_pos_f64: [f64; 3],
    );

    /// Called every frame during egui rendering to add custom UI elements
    #[cfg(feature = "debug_panel")]
    fn render_ui(&mut self, _ctx: &egui::Context, _ui: &mut egui::Ui) {}
}
