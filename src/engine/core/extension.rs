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
        frustum: &[(DVec3, f64); 6],
        camera: &mut crate::engine::camera::camera::Camera,
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
    fn render_ui(&mut self, _ctx: &egui::Context, _ui: &mut egui::Ui) {}
}
