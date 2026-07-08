use glam::Mat4;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub(super) struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    inv_view_proj: [[f32; 4]; 4],
    camera_pos: [f32; 4],
    sun_params: [f32; 4],
}

impl CameraUniform {
    pub(super) fn new() -> Self {
        Self {
            view_proj: glam::Mat4::IDENTITY.to_cols_array_2d(),
            inv_view_proj: glam::Mat4::IDENTITY.to_cols_array_2d(),
            camera_pos: [0.0; 4],
            sun_params: [1.0, 0.0, 0.0, 0.0],
        }
    }

    pub(super) fn update_matrix(
        &mut self,
        view: Mat4,
        proj: Mat4,
        camera_pos_dvec: glam::DVec3,
        sun_intensity: f32,
        color_grading: [f32; 3],
    ) {
        let view_proj = proj * view;
        self.view_proj = view_proj.to_cols_array_2d();
        self.inv_view_proj = view_proj.inverse().to_cols_array_2d();
        self.camera_pos = [
            camera_pos_dvec.x as f32,
            camera_pos_dvec.y as f32,
            camera_pos_dvec.z as f32,
            1.0,
        ];
        self.sun_params = [
            sun_intensity,
            color_grading[0],
            color_grading[1],
            color_grading[2],
        ];
    }
}
