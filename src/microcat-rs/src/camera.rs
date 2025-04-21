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
use tokio::sync::mpsc::Sender;
use tokio::sync::watch::Receiver;
use tracing::{debug, info, trace};

pub fn run_camera(telemetry_tx: Sender<crate::Telemetry>, mut shutdown_rx: Receiver<bool>) {
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
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

            let size = Size {
                width: 800,
                height: 600,
            };
            cfgs.get_mut(0).unwrap().set_pixel_format(PIXEL_FORMAT_RGB);
            cfgs.get_mut(0).unwrap().set_size(size);

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
                "RGB is not supported by the camera"
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
                debug!("Request queued for execution: {req:#?}");
                cam.queue_request(req).unwrap();
            }
            let width = cfg.get_size().width as usize;
            let height = cfg.get_size().height as usize;
            loop {
                tokio::select! {
                    Some(mut req) = rx.recv() => {
                        trace!("Camera request {:?} completed!", req);
                        trace!("Metadata: {:#?}", req.metadata());

                        let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> =
                            req.buffer(&stream).unwrap();
                        trace!("FrameBuffer metadata: {:#?}", framebuffer.metadata());

                        let planes = framebuffer.data();
                        let rgb_data = planes.first().unwrap();
                        // Actual encoded data will be smalled than framebuffer size, its length can be obtained from metadata.
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
                        let image = sensor_msgs::msg::Image {
                            header,
                            height: height as u32,
                            width: width as u32,
                            encoding: "rgb8".to_string(),
                            is_bigendian: 0,
                            step: (width * 3) as u32,
                            data: rgb_data[..bytes_used].to_vec(),
                        };
                        let _ = telemetry_tx.send(Telemetry::CameraData(image)).await;

                        req.reuse(ReuseFlag::REUSE_BUFFERS);
                        cam.queue_request(req).unwrap();

                    }
                    _ = shutdown_rx.changed() => {
                        println!("Shutting down Camera task...");
                        info!("Shutting down Camera task...");
                        break;
                    }
                }
            }
        });
    });
}
