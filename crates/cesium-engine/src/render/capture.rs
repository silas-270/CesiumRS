pub fn capture_pixels(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    output_texture: &wgpu::Texture,
    config: &wgpu::SurfaceConfiguration,
) -> Vec<u8> {
    let u32_size = std::mem::size_of::<u32>() as u32;
    let unpadded_bytes_per_row = config.width * u32_size;
    let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let padded_bytes_per_row = unpadded_bytes_per_row.div_ceil(align) * align;

    let buffer_size = (padded_bytes_per_row * config.height) as wgpu::BufferAddress;

    let staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Screenshot Staging Buffer"),
        size: buffer_size,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut copy_encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("Screenshot Copy Encoder"),
    });

    copy_encoder.copy_texture_to_buffer(
        wgpu::ImageCopyTexture {
            texture: output_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::ImageCopyBuffer {
            buffer: &staging_buffer,
            layout: wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(padded_bytes_per_row),
                rows_per_image: Some(config.height),
            },
        },
        wgpu::Extent3d {
            width: config.width,
            height: config.height,
            depth_or_array_layers: 1,
        },
    );

    queue.submit(Some(copy_encoder.finish()));

    let buffer_slice = staging_buffer.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
        tx.send(result).unwrap();
    });

    device.poll(wgpu::Maintain::Wait);
    rx.recv().unwrap().unwrap();

    let data = buffer_slice.get_mapped_range();
    let mut rgba_data = Vec::with_capacity((config.width * config.height * 4) as usize);
    let is_bgra = matches!(
        config.format,
        wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb
    );

    for chunk in data.chunks(padded_bytes_per_row as usize) {
        for i in 0..config.width as usize {
            let c0 = chunk[i * 4];
            let c1 = chunk[i * 4 + 1];
            let c2 = chunk[i * 4 + 2];
            let c3 = chunk[i * 4 + 3];
            if is_bgra {
                rgba_data.push(c2);
                rgba_data.push(c1);
                rgba_data.push(c0);
                rgba_data.push(c3);
            } else {
                rgba_data.push(c0);
                rgba_data.push(c1);
                rgba_data.push(c2);
                rgba_data.push(c3);
            }
        }
    }
    drop(data);
    staging_buffer.unmap();

    rgba_data
}

pub fn capture_screenshot(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    output_texture: &wgpu::Texture,
    config: &wgpu::SurfaceConfiguration,
    out_path: &str,
) {
    let rgba_data = capture_pixels(device, queue, output_texture, config);
    let _ = image::save_buffer(
        out_path,
        &rgba_data,
        config.width,
        config.height,
        image::ColorType::Rgba8,
    );
}
