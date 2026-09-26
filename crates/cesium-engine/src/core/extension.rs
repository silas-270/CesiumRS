use glam::DVec3;

pub trait GlobeExtension: Send + 'static {
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

    /// Called every frame right before [`Self::update`] with the engine's ground query:
    /// terrain height above the ellipsoid in megametres under an ECEF position, or `None`
    /// with terrain off or no data there yet. Default: ignored.
    fn sample_ground(&mut self, _ground: &dyn Fn(DVec3) -> Option<f64>) {}

    /// Called every frame after the globe and engine debug models are drawn
    fn render<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        camera_bind_group: &'a wgpu::BindGroup,
        viewport_size: [f32; 2],
        camera_pos_f64: [f64; 3],
    );

    /// Boxes of 3D content the screen-space label overlay must not be painted over,
    /// each as its eight corners relative to the camera (ECEF, megametres). The labels
    /// are drawn by egui after the scene, so without this they sit on top of the
    /// aircraft. Default: none.
    fn label_occluders(
        &self,
        _camera_pos_f64: [f64; 3],
        _viewport_size: [f32; 2],
    ) -> Vec<[glam::Vec3; 8]> {
        Vec::new()
    }

    /// Active runway corridors for terrain flattening, if any.
    fn runway_corridors(&self) -> &[crate::globe::terrain::RunwayCorridor] {
        &[]
    }

    /// Snapshot the extension's configuration and state for a high-res headless render.
    fn snapshot_for_headless(&self) -> Option<Box<dyn GlobeExtension>> {
        None
    }

    /// Called every frame during egui rendering to add custom UI elements
    #[cfg(feature = "debug_panel")]
    fn render_ui(&mut self, _ctx: &egui::Context, _ui: &mut egui::Ui) {}
}


