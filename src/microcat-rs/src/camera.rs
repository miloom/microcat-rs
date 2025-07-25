use crate::Telemetry;
use libcamera::geometry::Size;
use libcamera::request::ReuseFlag;
use libcamera::{
    camera::CameraConfigurationStatus,
    framebuffer::AsFrameBuffer,
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    pixel_format::PixelFormat,
    properties,
    stream::StreamRole,
};
use std::thread::JoinHandle;
use tokio::sync::mpsc::Sender;
use tokio::sync::watch::Receiver;
use tracing::{debug, error, info};

pub fn run_camera(
    telemetry_tx: Sender<crate::Telemetry>,
    shutdown_rx: Receiver<bool>,
) -> JoinHandle<()> {
    info!("Starting camera task");
    std::thread::spawn(move || {
        let manager = libcamera::camera_manager::CameraManager::new().unwrap();
        let cams = manager.cameras();
        debug!("Got camera list");
        let cam = cams.get(0).expect("No camera found");
        info!(
            "Using camera {}",
            *cam.properties().get::<properties::Model>().unwrap()
        );
        let mut cam = cam.acquire().expect("Unable to acquire camera");

        let mut cfgs = cam
            .generate_configuration(&[StreamRole::VideoRecording])
            .unwrap();
        for format in cfgs.get(0).unwrap().formats().pixel_formats().into_iter() {
            debug!(
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
        cfgs.get(0)
            .unwrap()
            .formats()
            .sizes(PIXEL_FORMAT_RGB)
            .iter()
            .for_each(|size| {
                debug!("{:?}", size);
            });
        const PIXEL_FORMAT_RGB: PixelFormat =
            PixelFormat::new(u32::from_le_bytes([b'R', b'G', b'2', b'4']), 0);

        let size = Size {
            width: 480,
            height: 320,
        };
        cfgs.get_mut(0).unwrap().set_pixel_format(PIXEL_FORMAT_RGB);
        cfgs.get_mut(0).unwrap().set_size(size);
        cfgs.get_mut(0).unwrap().set_buffer_count(2);

        match cfgs.validate() {
            CameraConfigurationStatus::Valid => info!("Camera configuration valid!"),
            CameraConfigurationStatus::Adjusted => {
                info!("Camera configuration was adjusted: {:#?}", cfgs)
            }
            CameraConfigurationStatus::Invalid => {
                panic!("Error validating camera configuration")
            }
        }
        assert_eq!(
            cfgs.get(0).unwrap().get_pixel_format(),
            PIXEL_FORMAT_RGB,
            "RGB24 is not supported by the camera"
        );
        cam.configure(&mut cfgs)
            .expect("Unable to configure camera");
        let mut alloc = FrameBufferAllocator::new(&cam);
        let cfg = cfgs.get(0).unwrap();
        let stream = cfg.stream().unwrap();
        let buffers = alloc.alloc(&stream).unwrap();
        debug!("Allocated {} buffers", buffers.len());
        let buffers = buffers
            .into_iter()
            .map(|buf| MemoryMappedFrameBuffer::new(buf).unwrap())
            .collect::<Vec<_>>();

        let reqs = buffers
            .into_iter()
            .enumerate()
            .map(|(i, buf)| {
                let mut req = cam.create_request(Some(i as u64)).unwrap();
                req.add_buffer(&stream, buf).unwrap();
                req
            })
            .collect::<Vec<_>>();
        let (tx, mut rx) = tokio::sync::mpsc::channel(10);
        cam.on_request_completed(move |req| {
            tx.blocking_send(req).unwrap();
        });
        cam.start(None).unwrap();
        for req in reqs {
            cam.queue_request(req).unwrap();
        }
        let width = cfg.get_size().width;
        let height = cfg.get_size().height;
        loop {
            if let Ok(true) = shutdown_rx.has_changed() {
                println!("Shutting down Camera task...");
                info!("Shutting down Camera task...");
                break;
            } else if let Ok(mut req) = rx.try_recv() {
                // trace!("Camera request {:?} completed!", req);
                // trace!("Metadata: {:#?}", req.metadata());

                let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> =
                    req.buffer(&stream).unwrap();
                // trace!("FrameBuffer metadata: {:#?}", framebuffer.metadata());

                let planes = framebuffer.data();
                let rgb_data = planes.first().unwrap();
                // Actual encoded data will be smaller than framebuffer size, its length can be obtained from metadata.
                let bytes_used = framebuffer
                    .metadata()
                    .unwrap()
                    .planes()
                    .get(0)
                    .unwrap()
                    .bytes_used as usize;
                let now = tokio::time::Instant::now();
                let header = std_msgs::msg::Header {
                    stamp: builtin_interfaces::msg::Time {
                        sec: now.elapsed().as_secs() as i32,
                        nanosec: now.elapsed().subsec_nanos(),
                    },
                    frame_id: String::new(),
                };

                let mut data = Vec::with_capacity(bytes_used);
                data.extend_from_slice(&rgb_data[..bytes_used]);

                let image = sensor_msgs::msg::Image {
                    header,
                    height,
                    width,
                    encoding: "rgb8".to_string(),
                    is_bigendian: 0,
                    step: width * 3,
                    data,
                };
                if let Err(e) = telemetry_tx.blocking_send(Telemetry::CameraData(image)) {
                    error!("Error sending image: {}", e);
                }

                req.reuse(ReuseFlag::REUSE_BUFFERS);
                cam.queue_request(req).unwrap();
            }
        }
    })
}
