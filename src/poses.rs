use crate::{JointType, Joint2D, Vec2, PoseRecord};
use std::fs:: OpenOptions;
use std::io::BufReader;
use serde_json::Deserializer;
/*
Some(Head) x: 0.55, y: 0.08
Some(Neck) x: 0.56, y: 0.09
Some(Torso) x: 0.56, y: 0.28
Some(Waist) x: 0.57, y: 0.38
Some(LeftCollar) x: 0.56, y: 0.14
Some(LeftShoulder) x: 0.70, y: 0.14
Some(LeftElbow) x: 0.90, y: 0.16
Some(LeftWrist) x: 0.87, y: 0.09
Some(LeftHand) x: 0.86, y: 0.07
Some(LeftFingertip) x: 0.00, y: 0.00
Some(RightCollar) x: 0.56, y: 0.14
Some(RightShoulder) x: 0.42, y: 0.14
Some(RightElbow) x: 0.25, y: 0.19
Some(RightWrist) x: 0.07, y: 0.19
Some(RightHand) x: 0.04, y: 0.19
Some(RightFingertip) x: 0.00, y: 0.00
Some(LeftHip) x: 0.64, y: 0.41
Some(LeftKnee) x: 0.64, y: 0.64
Some(LeftAnkle) x: 0.64, y: 0.85
Some(LeftFoot) x: 0.00, y: 0.00
Some(RightHip) x: 0.49, y: 0.42
Some(RightKnee) x: 0.49, y: 0.64
Some(RightAnkle) x: 0.49, y: 0.85
Some(RightFoot) x: 0.00, y: 0.00
*/

pub const STAR: &'static [(JointType, Vec2)] = &[
    (JointType::Head, Vec2{ x:0.55, y: 0.08}),
    (JointType::Neck, Vec2{ x:0.56, y: 0.09}),
    (JointType::Waist, Vec2{ x:0.57, y: 0.38}),
    (JointType::LeftCollar, Vec2{ x:0.56, y: 0.14}),
    (JointType::LeftShoulder, Vec2{ x:0.70, y: 0.14}),
    (JointType::LeftElbow, Vec2{ x:0.90, y: 0.16}),
    (JointType::LeftWrist, Vec2{ x:0.87, y: 0.09}),
    (JointType::LeftHand, Vec2{ x:0.86, y: 0.07}),
    (JointType::RightCollar, Vec2{ x:0.56, y: 0.14}),
    (JointType::RightShoulder, Vec2{ x:0.42, y: 0.14}),
    (JointType::RightElbow, Vec2{ x:0.25, y: 0.19}),
    (JointType::RightWrist, Vec2{ x:0.07, y: 0.19}),
    (JointType::RightHand, Vec2{ x:0.04, y: 0.19}),
    (JointType::LeftHip, Vec2{ x:0.64, y: 0.41}),
    (JointType::LeftKnee, Vec2{ x:0.64, y: 0.64}),
    (JointType::LeftAnkle, Vec2{ x:0.64, y: 0.85}),
    (JointType::LeftFoot, Vec2{ x:0.00, y: 0.00}),
    (JointType::RightHip, Vec2{ x:0.49, y: 0.42}),
    (JointType::RightKnee, Vec2{ x:0.49, y: 0.64}),
    (JointType::RightAnkle, Vec2{ x:0.49, y: 0.85}),
    (JointType::RightFoot, Vec2{ x:0.00, y: 0.00}),
    (JointType::Torso, Vec2{ x:0.56, y: 0.28}),
];

pub fn read_poses() -> Vec<PoseRecord> {
    let mut path = std::env::current_dir().expect("Couldn't find current directory");
    path.push("poses.json");
    let file = OpenOptions::new().read(true).open(path).expect("Failed to open file for writing poses");
    let reader = BufReader::new(file);
    Deserializer::from_reader(reader)
        .into_iter::<PoseRecord>()
        .map(Result::unwrap)
        .collect()
}
