use nuitrack_rs as nui;

use std::collections::HashMap;
use std::iter::FromIterator;
use self::nui::{Skeleton, JointType};

#[derive(Debug)]
pub enum Pose {
    Star,
}

#[derive(Clone, Debug)]
pub struct Vec2 {
    x: f32,
    y: f32,
}

pub type JointPos = HashMap<JointType, Vec2>;

const THRESHOLD: f32 = 0.5;

const SPOT: Vec2 = Vec2{ x: 0.5, y: 0.5 };

const STAR: &'static [(JointType, Vec2)] = &[
    (JointType::Head, SPOT),
    (JointType::Neck, SPOT),
    (JointType::LeftShoulder, SPOT),
    (JointType::Waist, SPOT),
    (JointType::Torso, SPOT),
    (JointType::LeftShoulder, SPOT),
    (JointType::LeftElbow, SPOT),
    (JointType::LeftHip, SPOT),
    (JointType::LeftKnee, SPOT),
    (JointType::RightShoulder, SPOT),
    (JointType::RightElbow, SPOT),
    (JointType::RightHip, SPOT),
    (JointType::RightKnee, SPOT),
];

pub fn detect(skeleton: &Skeleton) -> Option<Pose> {
    let joints = skeleton
        .joints()
        .iter();

    let mut joint_pos: JointPos = HashMap::new();

    for j in joints {
        match JointType::from_u32(j.type_) {
            Some(t) => {
                joint_pos.insert(t, Vec2{ x: j.proj.x, y: j.proj.y });
            },
            _ => (),
        }
    }

    let star: JointPos = HashMap::from_iter(STAR.iter().cloned());
    // JointPos -> Pose -> [Distance]
    // [Distance] -> closeness: f32
    let closeness = joint_pos.iter()
        .filter_map(|(k, v1)| {
            star.get(&k)
                .map(|v2| distance(v1, v2))
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
