use nuitrack_rs;
use nuitrack_pose_estimation as npe;
use std::env;
use std::sync::mpsc;
use std::io;

enum Message {
    Joint(f32),
    Rotation(f32),
}

fn main() -> io::Result<()> {
    let mut args = env::args();
    args.next();
    let path = match args.next() {
        Some(p) => std::path::PathBuf::from(p),
        None => {
            println!("Please specify a path to a nuitrack recording");
            return Ok(());
        },
    };
    
    let mut detector = npe::Detector::new(npe::Settings::default());
    
    let mut nui = nuitrack_rs::playback(path, true).expect("Failed to make player");

    let (controls_tx, controls_rx) = mpsc::channel();
        
    nui.skeleton_data(move |data| {
        if let Ok(msg) = controls_rx.try_recv() {
            match msg {
                Message::Joint(v) => {
                    detector.settings.joint_cutoff = v;
                    println!("Updated joint cutoff to {}", v);
                },
                Message::Rotation(v) => {
                    detector.settings.rotation_cutoff = v;
                    println!("Updated rotation cutoff to {}", v);
                },
            }
        }
        for skeleton in data.skeletons() {
            println!("Detecting {:?}", detector.detect(skeleton.joints()));
        }
    }).expect("Failed to create skeleton callback");

    loop {
        nui.update().expect("failed to update nui player");
        
        println!("Update joint cutoff:");
        let mut joint_cutoff = String::new();
        io::stdin().read_line(&mut joint_cutoff)?;
        let joint_cutoff = joint_cutoff.trim();
        match joint_cutoff.parse() {
            Ok(jt) => {
                controls_tx.send(Message::Joint(jt)).ok();
            },
            Err(_) => (),
        }

        println!("Update rotation cutoff:");
        let mut rotation_cutoff = String::new();
        std::io::stdin().read_line(&mut rotation_cutoff)?;
        let rotation_cutoff = rotation_cutoff.trim();
        match rotation_cutoff.parse() {
            Ok(r) => {
                controls_tx.send(Message::Rotation(r)).ok();
            },
            Err(_) => (),
        }
    }
}

