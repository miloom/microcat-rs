use libcamera;
pub struct Camera {
    camera: libcamera::camera::Camera,
}

impl Camera {
    pub fn new() -> Camera {
        let manager = libcamera::camera_manager::CameraManager::new().unwrap();
        Camera {}
    }
}
