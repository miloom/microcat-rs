use libcamera::{
    camera::CameraConfigurationStatus,
    camera_manager::CameraManager,
    framebuffer::AsFrameBuffer,
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    pixel_format::PixelFormat,
    properties,
    stream::StreamRole,
};
use tokio::time::Duration;
use tracing::{error, info, span, trace};
pub struct Camera {}

impl Camera {
    pub fn new() -> Camera {
        let manager = libcamera::camera_manager::CameraManager::new().unwrap();
        let cams = manager.cameras();
        let cam = cams.get(0).expect("No camera found");
        info!(
            "Using camera {}",
            *cam.properties().get::<properties::Model>().unwrap()
        );
        let mut cam = cam.acquire().expect("Unable to acquire camera");

        let mut cfgs = cam
            .generate_configuration(&[StreamRole::ViewFinder])
            .unwrap();
        for format in cfgs.get(0).unwrap().formats().pixel_formats().into_iter() {
            info!(
                "{:?} {} {}",
                format,
                format
                    .fourcc()
                    .to_le_bytes()
                    .map(|v| v as char)
                    .iter()
                    .collect::<String>(),
                format.modifier()
            );
        }
        const PIXEL_FORMAT_RGB: PixelFormat =
            PixelFormat::new(u32::from_le_bytes([b'R', b'G', b'2', b'4']), 0);

        cfgs.get_mut(0).unwrap().set_pixel_format(PIXEL_FORMAT_RGB);
        match cfgs.validate() {
            CameraConfigurationStatus::Valid => info!("Camera configuration valid!"),
            CameraConfigurationStatus::Adjusted => {
                info!("Camera configuration was adjusted: {:#?}", cfgs)
            }
            CameraConfigurationStatus::Invalid => panic!("Error validating camera configuration"),
        }
        assert_eq!(
            cfgs.get(0).unwrap().get_pixel_format(),
            PIXEL_FORMAT_RGB,
            "RGB is not supported by the camera"
        );
        cam.configure(&mut cfgs)
            .expect("Unable to configure camera");
        let mut alloc = FrameBufferAllocator::new(&cam);
        let cfg = cfgs.get(0).unwrap();
        let stream = cfg.stream().unwrap();
        let buffers = alloc.alloc(&stream).unwrap();
        println!("Allocated {} buffers", buffers.len());
        let buffers = buffers
            .into_iter()
            .map(|buf| MemoryMappedFrameBuffer::new(buf).unwrap())
            .collect::<Vec<_>>();

        let mut reqs = buffers
            .into_iter()
            .map(|buf| {
                let mut req = cam.create_request(None).unwrap();
                req.add_buffer(&stream, buf).unwrap();
                req
            })
            .collect::<Vec<_>>();
        let (tx, rx) = std::sync::mpsc::channel();
        cam.on_request_completed(move |req| {
            tx.send(req).unwrap();
        });
        cam.start(None).unwrap();
        cam.queue_request(reqs.pop().unwrap()).unwrap();
        let req = rx
            .recv_timeout(Duration::from_secs(2))
            .expect("Camera request failed");
        let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> = req.buffer(&stream).unwrap();
        let planes = framebuffer.data();
        use std::io::Write;
        let width = cfg.get_size().width as usize;
        let height = cfg.get_size().height as usize;
        let rgb_data = planes.first().unwrap();
        let rgb_len = framebuffer
            .metadata()
            .unwrap()
            .planes()
            .get(0)
            .unwrap()
            .bytes_used as usize;

        let filename = "picture.ppm".to_string();
        let mut file = std::fs::File::create(&filename).unwrap();
        write!(file, "P6\n{} {}\n255\n", width, height).unwrap(); // PPM header
        file.write_all(&rgb_data[..rgb_len]).unwrap();
        info!("Written PPM image to {}", &filename);
        Camera {}
    }
}
