use nuitrack_rs as nui;

use std::collections::HashMap;
use std::iter::FromIterator;
use self::nui::{Skeleton, SKELETON_BONES, JointType};

#[derive(Debug)]
pub enum Pose {
    Star,
}

#[derive(Clone, Debug)]
pub struct Vec2 {
    x: f32,
    y: f32,
}

pub type Bones = HashMap<(JointType, JointType), (Vec2, Vec2)>;

const THRESHOLD: f32 = 0.5;

const SPOT: Vec2 = Vec2{ x: 0.5, y: 0.5 };

const STAR: &'static [((JointType, JointType), (Vec2, Vec2))] = &[
    ((JointType::Head, JointType::Neck), (SPOT, SPOT)),
    ((JointType::Neck, JointType::Torso), (SPOT, SPOT)),
    ((JointType::LeftShoulder, JointType::RightShoulder), (SPOT, SPOT)),
    ((JointType::Waist, JointType::LeftHip), (SPOT, SPOT)),
    ((JointType::Waist, JointType::RightHip), (SPOT, SPOT)),
    ((JointType::Torso, JointType::Waist), (SPOT, SPOT)),
    ((JointType::LeftShoulder, JointType::LeftElbow), (SPOT, SPOT)),
    ((JointType::LeftElbow, JointType::LeftWrist), (SPOT, SPOT)),
    ((JointType::LeftHip, JointType::LeftKnee), (SPOT, SPOT)),
    ((JointType::LeftKnee, JointType::LeftAnkle), (SPOT, SPOT)),
    ((JointType::RightShoulder, JointType::RightElbow), (SPOT, SPOT)),
    ((JointType::RightElbow, JointType::RightWrist), (SPOT, SPOT)),
    ((JointType::RightHip, JointType::RightKnee), (SPOT, SPOT)),
    ((JointType::RightKnee, JointType::RightAnkle), (SPOT, SPOT)),
];

pub fn detect(skeleton: &Skeleton) -> Option<Pose> {
    // [(Joint, Joint)] -> Bones 
    let joints = skeleton
        .joints()
        .iter()
        .zip(skeleton.joints().iter());

    let mut bones: Bones = HashMap::new();

    for (a, b) in joints {
        match (JointType::from_u32(a.type_), JointType::from_u32(b.type_)) {
            (Some(t1), Some(t2)) => {
                bones.insert((t1, t2), (Vec2{ x: a.proj.x, y: a.proj.y }, Vec2{ x: b.proj.x, y: b.proj.y }));
            },
            _ => (),
        }
    }

    let star: Bones = HashMap::from_iter(STAR.iter().cloned());
    // Bones -> Pose -> [Distance]
    // [Distance] -> closeness: f32
    let closeness = bones.iter()
        .filter_map(|(k, (v1_s, v1_e))| {
            star.get(&k)
                .map(|(v2_s, v2_e)| distance(v1_s, v2_s) + distance(v1_e, v2_e))
        })
    .fold(0.0, |total, dist| total + dist);
    if closeness < THRESHOLD {
        Some(Pose::Star)
    } else {
        None
    }
}

fn distance(a: &Vec2, b: &Vec2) -> f32 {
    ((a.x - b.x) + (a.y - b.y)).sqrt()
}
