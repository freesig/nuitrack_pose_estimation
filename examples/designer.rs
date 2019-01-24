extern crate nannou;
extern crate vulkano;
extern crate vulkano_shaders;
extern crate nuitrack_rs as nuitrack;

use termion;
use nuitrack_pose_estimation;
use serde_json;
use clap;

mod controls;

use nuitrack_pose_estimation::{Joint2D, Joint2DType};
use self::nannou::prelude::*;
use self::vulkano::buffer::{BufferUsage, ImmutableBuffer};
use self::vulkano::command_buffer::DynamicState;
use self::vulkano::descriptor::descriptor_set::PersistentDescriptorSet;
use self::vulkano::device::DeviceOwned;
use self::vulkano::format::Format;
use self::vulkano::framebuffer::{RenderPassAbstract, Subpass};
use self::vulkano::image::{Dimensions, ImmutableImage};
use self::vulkano::pipeline::viewport::Viewport;
use self::vulkano::pipeline::{GraphicsPipeline, GraphicsPipelineAbstract};
use self::vulkano::sampler::{Filter, MipmapMode, Sampler, SamplerAddressMode};
use self::nannou::window::SwapchainFramebuffers;
use self::nuitrack::{Color3, Joint};
use std::cell::RefCell;
use std::sync::{mpsc, Arc};
use self::nuitrack::JointType;
use std::thread::JoinHandle;  
use std::time::{Duration, Instant};
use std::fs::OpenOptions;

use nannou::ui::{Ui, widget};

fn main() {
    nannou::app(model)
        .update(update)
        .exit(exit)
        .run();
}

struct Model {
    rx: Receivers,
    current: Current,
    graphics: Graphics,
    handles: Handles,
    capture: Capture,
    mode: DesignMode,
    controls: Controls,
    ui: Ui,
    ids: Ids,
}

struct Controls {
    threshold: f32,
    curve: i32,
    joint_cutoff: f32,
}

struct Capture {
    time: Timing,
    done: bool,
}

struct Timing {
    left: Duration,
    last: Instant,
}

struct Handles {
    nui: JoinHandle<()>,
}

struct Graphics {
    render_pass: Arc<RenderPassAbstract + Send + Sync>,
    pipeline: Arc<GraphicsPipelineAbstract + Send + Sync>,
    framebuffers: RefCell<SwapchainFramebuffers>,
    sampler: Arc<Sampler>,
}

#[derive(Debug, Clone)]
struct Vertex {
    position: [f32; 2],
    tex_coords: [f32; 2],
    mode: u32,
}

nannou::vulkano::impl_vertex!(Vertex, mode, position, tex_coords);

impl Vertex {
    const MODE_RGBA: u32 = 0;
    const MODE_DEPTH: u32 = 1;
    const MODE_SKELETON: u32 = 2;
    const MODE_DEBUG_POSE: u32 = 3;
    const MODE_DEBUG_SKELETON: u32 = 4;
}

// The most recently received skeletons, depth and color.
struct Current {
    skeletons: Option<Vec<Skeleton>>,
    debug_poses: Option<Vec<Vector2<f32>>>,
    debug_skeletons: Option<Vec<Vector2<f32>>>,
    depth: Option<DepthFrame>,
    color: Option<ColorFrame>,
    // The current color image on the GPU.
    rgba_image: Arc<ImmutableImage<Format>>,
    depth_image: Arc<ImmutableImage<Format>>,
}

struct Receivers {
    skeletons: mpsc::Receiver<Vec<Skeleton>>,
    detection: mpsc::Sender<DetectionMsg>,
    depth: mpsc::Receiver<DepthFrame>,
    color: mpsc::Receiver<ColorFrame>,
    nui: mpsc::Sender<bool>,
    skeleton_debug_rx: mpsc::Receiver<(Vec<Vector2>, Vec<Vector2>)>,
}

#[derive(Clone)]
enum DesignMode {
    Capture(u64),
    Test(String, f32),
    Play,
    Detect,
}

struct Skeleton {
    joints: Vec<Joint>,
}

struct DepthFrame {
    rows: u32,
    cols: u32,
    data: Vec<u16>,
}

struct ColorFrame {
    rows: u32,
    cols: u32,
    data: Vec<Color3>,
}

enum DetectionMsg {
    Threshold(f32),
    Curve(i32),
    JointCutOff(f32),
}

struct Ids {
    threshold: widget::Id,
    curve: widget::Id,
    joint_cutoff: widget::Id,
    background: widget::Id,
}

fn model(app: &App) -> Model {
    let arg_matches = clap::App::new("Pose Designer")
        .arg(clap::Arg::with_name("capture")
             .short("c")
             .long("capture")
             .help("Captures a poses after x seconds")
             .takes_value(true))
        .arg(clap::Arg::with_name("test")
             .short("t")
             .long("test")
             .help("Tests a pose. Pass it the pose name.")
             .takes_value(true))
        .arg(clap::Arg::with_name("threshold")
             .short("h")
             .long("threshold")
             .help("Threshold of pose detection.")
             .takes_value(true))
        .arg(clap::Arg::with_name("detect")
             .short("d")
             .long("detect")
             .help("Detects all shapes"))
        .get_matches();
    let design_mode = match (arg_matches.value_of("capture"), arg_matches.value_of("test"), arg_matches.value_of("threshold")) {
        (Some(s), None, None) => DesignMode::Capture(s.parse().unwrap()),
        (None, Some(p), Some(t)) => DesignMode::Test(p.to_string(), t.parse().unwrap()),
        (None, Some(p), None) => DesignMode::Test(p.to_string(), 10.0),
        _ => {
            if arg_matches.is_present("detect") {
                DesignMode::Detect
            } else {
                DesignMode::Play
            }
        }
    };

    let window_id = app.new_window()
        .with_dimensions(480 * 2, 640)
        .view(view)
        .build()
        .unwrap();

    app.window(window_id)
        .expect("No window")
        .set_position(600, 0);

    /////////////////////
    // Nuitrack Thread //
    /////////////////////

    let (skeletons_tx, skeletons) = mpsc::channel();
    let (skeleton_debug_tx, skeleton_debug_rx) = mpsc::channel();
    let (detection_tx, detection_rx) = mpsc::channel();
    let (depth_tx, depth) = mpsc::channel();
    let (color_tx, color) = mpsc::channel();
    let (nui_tx, nui_rx) = mpsc::channel();

    let detection_test = match design_mode {
        DesignMode::Test(ref pose_name, threshold) => nuitrack_pose_estimation::Tester::from_name(pose_name.clone(), threshold, 1, 1.0),
        _ => None,
    };

    let mut live_detection = match design_mode {
        DesignMode::Detect => Some(nuitrack_pose_estimation::Detector::new(1.0, 1, 1.0)),
        _ => None,
    };
    let nui_join_handle = std::thread::spawn(move || {
        let mut nui = nuitrack_rs::init()
            .expect("Couldn't create nui");
        
        nui.set_camera_rotation(90)
            .expect("Failed to rotate");

        nui.skeleton_data(move |data| {
            let mut debug_poses = Vec::new();
            let mut debug_skeletons = Vec::new();
            let skeletons = data.skeletons()
                .iter()
                .map(|skeleton| {
                    if let Some(ref tester) = detection_test {
                        let (pose, pose_verts, skeleton_verts) = tester.test(&skeleton);
                        debug_poses = pose_verts.iter()
                            .map(|v| vec2(v.x, v.y))
                            .collect();
                        debug_skeletons = skeleton_verts.iter()
                            .map(|v| vec2(v.x, v.y))
                            .collect();
                        //print!("{}{}", termion::cursor::Goto(1,40), termion::clear::CurrentLine);
                        println!("Pose: {:?}", pose);
                    }
                    if let Some(ref mut detector) = live_detection {
                        if let Ok(msg) = detection_rx.try_recv() {
                            match msg {
                                DetectionMsg::Threshold(t) => detector.set_threshold(t),
                                DetectionMsg::Curve(c) => detector.set_curve(c),
                                DetectionMsg::JointCutOff(co) => detector.set_joint_cutoff(co),
                            }
                        }
                        println!("Detecting {:?}", detector.detect(&skeleton));
                    }
                    Skeleton {
                        joints: skeleton.joints().to_vec()
                    }
                })
                .collect();
            skeletons_tx.send(skeletons).ok();
            skeleton_debug_tx.send((debug_poses, debug_skeletons)).ok();
        }).expect("Failed to add callback");

        nui.depth_data(move |data| {
            let depth = DepthFrame {
                rows: data.rows as _,
                cols: data.cols as _,
                data: data.frame().to_vec(),
            };
            depth_tx.send(depth).ok();
        }).expect("Failed to add callback");

        nui.color_data(move |data| {
            let color = ColorFrame {
                rows: data.rows as _,
                cols: data.cols as _,
                data: data.frame().to_vec(),
            };
            color_tx.send(color).ok();
        }).expect("Failed to add callback");


        let nui = nui.run().expect("failed to run nui");

        // Run at ~30fps forever.
        loop {
            match nui_rx.try_recv() {
                Ok(_) => break,
                Err(_) => {
                    nui.update().expect("failed to update nui player");
                },
            }
        }
    });

    let rx = Receivers { skeletons, depth, color, nui: nui_tx, skeleton_debug_rx, detection: detection_tx };

    let handles = Handles { nui: nui_join_handle };

    /////////////////////////////
    // Graphics Initialisation //
    /////////////////////////////

    let device = app.main_window().swapchain().device().clone();

    let vertex_shader = vs::Shader::load(device.clone()).unwrap();
    let fragment_shader = fs::Shader::load(device.clone()).unwrap();

    let render_pass = Arc::new(
        nannou::vulkano::single_pass_renderpass!(
            device.clone(),
            attachments: {
                color: {
                    load: Clear,
                    store: Store,
                    format: app.main_window().swapchain().format(),
                    samples: 1,
                    initial_layout: ImageLayout::PresentSrc,
                    final_layout: ImageLayout::PresentSrc,
                }
            },
            pass: {
                color: [color],
                depth_stencil: {}
            }
        )
        .unwrap(),
    );

    // Use dummy images for RGBA and depth for now until we get some data.
    let (width, height) = (640, 480);
    let (rgba_image, _future) = ImmutableImage::from_iter(
        (0..width * height).map(|_| [0u8, 0, 0, 1]),
        Dimensions::Dim2d { width, height },
        Format::R8G8B8A8Srgb,
        app.main_window().swapchain_queue().clone(),
    ).unwrap();
    let (depth_image, _future) = ImmutableImage::from_iter(
        (0..width * height).map(|_| std::u16::MAX),
        Dimensions::Dim2d { width, height },
        Format::R16Unorm,
        app.main_window().swapchain_queue().clone(),
    ).unwrap();

    // The sampler that will be used to sample the RGBA and depth textures.
    let sampler = Sampler::new(
        device.clone(),
        Filter::Linear,
        Filter::Linear,
        MipmapMode::Nearest,
        SamplerAddressMode::ClampToEdge,
        SamplerAddressMode::ClampToEdge,
        SamplerAddressMode::ClampToEdge,
        0.0,
        1.0,
        0.0,
        0.0,
    )
    .unwrap();

    let pipeline = Arc::new(
        GraphicsPipeline::start()
            .vertex_input_single_buffer::<Vertex>()
            .vertex_shader(vertex_shader.main_entry_point(), ())
            .triangle_list()
            .viewports_dynamic_scissors_irrelevant(1)
            .fragment_shader(fragment_shader.main_entry_point(), ())
            .blend_alpha_blending()
            .render_pass(Subpass::from(render_pass.clone(), 0).unwrap())
            .build(device.clone())
            .unwrap(),
    );

    let framebuffers = RefCell::new(SwapchainFramebuffers::default());

    let graphics = Graphics {
        render_pass,
        pipeline,
        framebuffers,
        sampler,
    };

    let current = Current {
        skeletons: None,
        debug_poses: None,
        debug_skeletons: None,
        depth: None,
        color: None,
        rgba_image,
        depth_image,
    };

    let time = match design_mode {
        DesignMode::Capture(s) => Timing {
            left: Duration::from_secs(s),
            last: Instant::now(),
        },
        _ => Timing {
            left: Duration::from_secs(0),
            last: Instant::now(),
        },
    };
    let capture = Capture { time, done: false };

    // GUI
    let gui_window = app.new_window()
        .with_dimensions(300, 500)
        .view(ui_view)
        .build()
        .expect("Failed to build second window");
    let mut ui = app.new_ui().window(gui_window).build().unwrap();

    let controls = Controls {
        threshold: 1.0,
        curve: 1,
        joint_cutoff: 1.0,

    };
    
    let ids = Ids {
        threshold: ui.generate_widget_id(),
        curve: ui.generate_widget_id(),
        joint_cutoff: ui.generate_widget_id(),
        background: ui.generate_widget_id(),
    };

    print!("{}", termion::clear::All);
    Model {
        rx,
        current,
        graphics,
        handles,
        capture,
        mode: design_mode,
        ui,
        controls,
        ids,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    controls::update(model);

    // Update the skeleton.
    if let Some(skeletons) = model.rx.skeletons.try_iter().last() {
        model.current.skeletons = Some(skeletons);
    }
    
    // Update the skeleton.
    if let Some((debug_pose, debug_skeleton)) = model.rx.skeleton_debug_rx.try_iter().last() {
        model.current.debug_poses = Some(debug_pose);
        model.current.debug_skeletons = Some(debug_skeleton);
    }
    
    if let DesignMode::Capture(_) = model.mode {
        if model.capture.time.left <= Duration::from_millis(500) && !model.capture.done {
            print!("{}{}", termion::cursor::Goto(1,20), termion::clear::CurrentLine);
            print!("CAPTURE");
            if let Some(ref skeletons) = model.current.skeletons {
                if let Some(sk) = skeletons.get(0) {
                    model.capture.done = true;
                    capture_skeleton(&sk);
                } else{
                    println!("No skeleton to capture");
                }
            } else {
                println!("No skeleton to capture");
            }
        } else {
            let left = model.capture.time.left.clone();
            let time_used = Instant::now() - model.capture.time.last;
            if let Some(left) = left.checked_sub(time_used) {
                model.capture.time.left = left;
            }
            print!("{}{}", termion::cursor::Goto(1,20), termion::clear::CurrentLine);
            print!("Capturing in: {}", model.capture.time.left.as_secs());
            model.capture.time.last = Instant::now();
        }
    }

    // Update the current depth.
    if let Some(depth) = model.rx.depth.try_iter().last() {
        let (width, height) = (depth.cols as _, depth.rows as _);
        let (depth_image, _future) = ImmutableImage::from_iter(
            depth.data.iter().cloned(),
            Dimensions::Dim2d { width, height },
            Format::R16Unorm,
            app.main_window().swapchain_queue().clone(),
        ).unwrap();
        model.current.depth_image = depth_image;
        model.current.depth = Some(depth);
    }

    // Update the current rgb.
    if let Some(color) = model.rx.color.try_iter().last() {
        let (width, height) = (color.cols as _, color.rows as _);
        let (rgba_image, _future) = ImmutableImage::from_iter(
            color.data.iter().map(|c| [c.red, c.green, c.blue, 255]),
            Dimensions::Dim2d { width, height },
            Format::R8G8B8A8Srgb,
            app.main_window().swapchain_queue().clone(),
        ).unwrap();
        model.current.rgba_image = rgba_image;
        model.current.color = Some(color);
    }
}

fn ui_view(app: &App, model: &Model, frame: Frame) -> Frame {
    // Draw the state of the `Ui` to the frame.
    model.ui.draw_to_frame(app, &frame).unwrap();
    frame
}

fn view(app: &App, model: &Model, frame: Frame) -> Frame {
    let [img_w, img_h] = frame.swapchain_image().dimensions();
    let graphics = &model.graphics;

    // Dynamic state.
    let viewport = Viewport {
        origin: [0.0, 0.0],
        dimensions: [img_w as _, img_h as _],
        depth_range: 0.0..1.0,
    };
    let dynamic_state = DynamicState {
        line_width: None,
        viewports: Some(vec![viewport]),
        scissors: None,
    };

    // The descriptor set for the rgb texture.
    let descriptor_set = Arc::new(
        PersistentDescriptorSet::start(graphics.pipeline.clone(), 0)
            .add_sampled_image(model.current.rgba_image.clone(), graphics.sampler.clone())
            .unwrap()
            .add_sampled_image(model.current.depth_image.clone(), graphics.sampler.clone())
            .unwrap()
            .build()
            .unwrap(),
    );

    // Update framebuffers so that count matches swapchain image count and dimensions match.
    graphics.framebuffers.borrow_mut()
        .update(&frame, graphics.render_pass.clone(), |builder, image| builder.add(image))
        .unwrap();

    // Left, right, top, bottom and middle values for vertex and texture coordinate systems.
    let (vl, vr, vt, vb, vm) = (-1.0, 1.0, -1.0, 1.0, 0.0);
    let (tl, tr, tt, tb) = (0.0, 1.0, 0.0, 1.0);

    // Rgba texture quad on the left.
    let rgba_vertices = [
        Vertex { position: [vl, vt], tex_coords: [tl, tt], mode: Vertex::MODE_RGBA },
        Vertex { position: [vl, vb], tex_coords: [tl, tb], mode: Vertex::MODE_RGBA },
        Vertex { position: [vm, vt], tex_coords: [tr, tt], mode: Vertex::MODE_RGBA },
        Vertex { position: [vl, vb], tex_coords: [tl, tb], mode: Vertex::MODE_RGBA },
        Vertex { position: [vm, vb], tex_coords: [tr, tb], mode: Vertex::MODE_RGBA },
        Vertex { position: [vm, vt], tex_coords: [tr, tt], mode: Vertex::MODE_RGBA },
    ];

    // Depth texture quad on the right.
    let depth_vertices = [
        Vertex { position: [vm, vt], tex_coords: [tl, tt], mode: Vertex::MODE_DEPTH },
        Vertex { position: [vm, vb], tex_coords: [tl, tb], mode: Vertex::MODE_DEPTH },
        Vertex { position: [vr, vt], tex_coords: [tr, tt], mode: Vertex::MODE_DEPTH },
        Vertex { position: [vm, vb], tex_coords: [tl, tb], mode: Vertex::MODE_DEPTH },
        Vertex { position: [vr, vb], tex_coords: [tr, tb], mode: Vertex::MODE_DEPTH },
        Vertex { position: [vr, vt], tex_coords: [tr, tt], mode: Vertex::MODE_DEPTH },
    ];


    // Skeleton vertices as squares overlaid on the Rgba texture half of the window.
    let mut skeleton_vertices = vec![];
    if let Some(ref skeletons) = model.current.skeletons {
        //print!("{}", termion::cursor::Goto(1,1));
        for skeleton in skeletons {
            for joint in &skeleton.joints {
                if let DesignMode::Play = model.mode {
                    println!("{}{:?} x: {}, y: {}", termion::clear::CurrentLine, JointType::from_u32(joint.type_), joint.proj.x, joint.proj.y);
                }
                let w = 8.0 / img_w as f32;
                let h = 8.0 / img_h as f32;
                let vs = Rect::from_w_h(w, h)
                    .shift([joint.proj.x - 1.0, joint.proj.y * 2.0 - 1.0].into())
                    //.shift_x(-1.0)
                    .triangles_iter()
                    .flat_map(|tri| tri.vertices())
                    .map(|v| {
                        Vertex {
                            position: [v.x, v.y],
                            tex_coords: [0.0, 0.0],
                            mode: Vertex::MODE_SKELETON,
                        }
                    });
                skeleton_vertices.extend(vs);
            }
        }
    }

    let mut debug_vertices = vec![];
    if let Some(ref debug_poses) = model.current.debug_poses {
        for debug_pose in debug_poses {
            let w = 16.0 / img_w as f32;
            let h = 16.0 / img_h as f32;
            let vs = Rect::from_w_h(w, h)
                .shift([debug_pose.x - 1.0, debug_pose.y * 2.0 - 1.0].into())
                .triangles_iter()
                .flat_map(|tri| tri.vertices())
                .map(|v| {
                    Vertex {
                        position: [v.x, v.y],
                        tex_coords: [0.0, 0.0],
                        mode: Vertex::MODE_DEBUG_POSE,
                    }
                });
            debug_vertices.extend(vs);
        }
    }

    if let Some(ref debug_skeletons) = model.current.debug_skeletons {
        for debug_skeleton in debug_skeletons {
            let w = 16.0 / img_w as f32;
            let h = 16.0 / img_h as f32;
            let vs = Rect::from_w_h(w, h)
                .shift([debug_skeleton.x - 1.0, debug_skeleton.y * 2.0 - 1.0].into())
                .triangles_iter()
                .flat_map(|tri| tri.vertices())
                .map(|v| {
                    Vertex {
                        position: [v.x, v.y],
                        tex_coords: [0.0, 0.0],
                        mode: Vertex::MODE_DEBUG_SKELETON,
                    }
                });
            debug_vertices.extend(vs);
        }
    }
    // Chain all the vertices together ready for vertex buffer creation.
    let vertices = rgba_vertices.iter().cloned()
        .chain(depth_vertices.iter().cloned())
        .chain(skeleton_vertices)
        .chain(debug_vertices)
        .collect::<Vec<_>>();

    // The vertex buffer that will be submitted to the GPU.
    let (vertex_buffer, _future) = ImmutableBuffer::from_iter(
        vertices.into_iter(),
        BufferUsage::all(),
        app.main_window().swapchain_queue().clone(),
    ).unwrap();

    let clear_values = vec![[0.0, 0.0, 0.0, 1.0].into()];

    frame
        .add_commands()
        .begin_render_pass(
            graphics.framebuffers.borrow()[frame.swapchain_image_index()].clone(),
            false,
            clear_values,
        )
        .unwrap()
        .draw(
            graphics.pipeline.clone(),
            &dynamic_state,
            vec![vertex_buffer.clone()],
            descriptor_set,
            (),
        )
        .unwrap()
        .end_render_pass()
        .expect("failed to add `end_render_pass` command");
    

    frame
}

fn capture_skeleton(skeleton: &Skeleton) {
    let mut skeleton_vertices: Vec<Joint2D> = Vec::new();
    for joint in &skeleton.joints {
        //println!("{}{:?} x: {2:.2}, y: {3:.2}", clear::CurrentLine, JointType::from_u32(joint.type_), joint.proj.x, joint.proj.y);
        if let Some(jt) = JointType::from_u32(joint.type_) {
            let jt: Joint2DType = jt.into();
            let pos = [joint.proj.x, joint.proj.y];
            skeleton_vertices.push(Joint2D::new(jt, pos));
        }
    }
    let mut input = String::new();

    println!("{}Enter a name for this pose", termion::clear::All);
    std::io::stdin().read_line(&mut input).expect("Couldnt read line");

    let name = input.trim().to_string();
    println!("Saving: {:?}", name);
    let pose_record = nuitrack_pose_estimation::PoseRecord {
        name,
        data: skeleton_vertices
    };
    let mut path = std::env::current_dir().expect("Couldn't find current directory");
    path.push("poses.json");
    let file = OpenOptions::new().append(true).create(true).open(path).expect("Failed to open file for writing poses");
    serde_json::to_writer(file, &pose_record).expect("Failed to write to file");
}


fn exit(_: &App, model: Model){
    let Model {
       rx,
       handles,
       ..
    } = model;
    rx.nui.send(true).unwrap();
    handles.nui.join().unwrap();
}

//////////////////
// GLSL Shaders //
//////////////////

mod vs {
    use vulkano;
    use vulkano_shaders;
    vulkano_shaders::shader! {
    ty: "vertex",
        src: "
#version 450

layout(location = 0) in vec2 position;
layout(location = 1) in vec2 tex_coords;
layout(location = 2) in uint mode;

layout(location = 0) out vec2 v_tex_coords;
layout(location = 1) flat out uint v_mode;

void main() {
    gl_Position = vec4(position, 0.0, 1.0);
    //v_tex_coords = position * 0.5 + 0.5;
    v_tex_coords = tex_coords;
    v_mode = mode;
}"
    }
}

mod fs {
    use vulkano;
    use vulkano_shaders;
    vulkano_shaders::shader! {
    ty: "fragment",
        src: "
#version 450

layout(location = 0) in vec2 tex_coords;
layout(location = 1) flat in uint mode;

layout(location = 0) out vec4 f_color;

layout(set = 0, binding = 0) uniform sampler2D rgba_sampler;
layout(set = 0, binding = 1) uniform sampler2D depth_sampler;

void main() {
    // RGBA.
    if (mode == uint(0)) {
        f_color = texture(rgba_sampler, tex_coords);
    // Depth.
    } else if (mode == uint(1)) {
        float d = texture(depth_sampler, tex_coords).x;
        f_color = vec4(d, d, d, 1.0);
    // Skeleton.
    } else if (mode == uint(2)) {
        f_color = vec4(1.0, 0.2, 0.6, 1.0);
    } else if (mode == uint(3)) {
        f_color = vec4(0.2, 1.0, 1.0, 1.0);
    } else {
        f_color = vec4(1.0, 0.0, 0.0, 1.0);
    }
}"
    }
}
