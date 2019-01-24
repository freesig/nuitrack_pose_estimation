use nuitrack_rs as nui;
use serde::{Serialize, Deserialize};

mod poses;
mod joints;

use std::collections::HashMap;
use std::iter::FromIterator;
use self::nui::{Skeleton, JointType};

pub use self::joints::Joint2DType;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseRecord {
    pub name: String,
    pub data: Vec<Joint2D>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Joint2D(Joint2DType, [f32; 2]);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Pose {
    DabR,
    DabL,
    HandsUp,
    Roof,
    FlyingR,
    FlyingL,
}

pub type JointPos = HashMap<JointType, Vec2>;

pub struct Detector {
    threshold: f32,
    curve: i32,
    poses: HashMap<Pose, JointPos>,
}

pub struct Tester {
    detector: Detector,
    name: Pose,
}

#[derive(Copy, Clone, Debug)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Joint2D {
    pub fn new(jt: Joint2DType, pos: [f32; 2]) -> Self {
        Joint2D(jt, pos)
    }
}

impl From<[f32; 2]> for Vec2 {
    fn from(item: [f32; 2]) -> Self {
        Vec2{ x: item[0], y: item[1] }
    }
}

impl Vec2 {
    fn xy(x: f32, y: f32) -> Self {
        Vec2{ x, y }
    }
}

impl Pose {
    fn from_name(name: &str) -> Option<Self> {
        match name {
            "DabR" => Some(Pose::DabR),
            "DabL" => Some(Pose::DabL),
            "HandsUp" => Some(Pose::HandsUp),
            "Roof" => Some(Pose::Roof),
            "FlyingR" => Some(Pose::FlyingR),
            "FlyingL" => Some(Pose::FlyingL),
            _ => None,
        }
    }
}

impl std::ops::Sub for &Vec2 {
    type Output = Vec2;
    
    fn sub(self, rhs: &Vec2) -> Self::Output {
        Vec2{ 
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl std::ops::Add for &Vec2 {
    type Output = Vec2;
    
    fn add(self, rhs: &Vec2) -> Self::Output {
        Vec2{ 
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}


impl Detector {
    pub fn new(threshold: f32, curve: i32) -> Self {
        let mut poses = HashMap::new();
        for pose_record in poses::read_poses().into_iter() {
            let PoseRecord {
                name,
                data,
            } = pose_record;
            if let Some(name) = Pose::from_name(&name) {
                poses.insert(name,
                             data.into_iter().map(|j2d| (joints::j2d_to_j(j2d.0), Vec2::xy(j2d.1[0], j2d.1[1]))).collect());
            }
        }
        Detector{ threshold, poses, curve }
    }
    
    pub fn detect(&self, skeleton: &Skeleton) -> Option<Pose> {
        let joints = joints_map(skeleton);
        self.check_poses(joints)
    }

    fn detect_pose(&self, name: Pose, skeleton: &Skeleton) -> (Option<()>, Vec<Vec2>, Vec<Vec2>) {
        let mut joints = joints_map(skeleton);
        let pose = self.poses.get(&name).expect(&format!("Pose {:?} doesn't exist", name));
        (self.check_pose(pose, &mut joints),
        pose.iter()
        .map(|(_, &v)| v)
        .collect(),
        joints.iter()
        .map(|(_, &v)| v)
        .collect())
    }

    fn check_poses(&self, mut joints: JointPos) -> Option<Pose> {
        for (name, pose) in &self.poses {
            if let Some(_) = self.check_pose(pose, &mut joints) {
                return Some(*name);
            }
        }
        None
    }

    fn check_pose(&self, pose: &JointPos, joints: &mut JointPos ) -> Option<()> {
        let torso = joints.get(&JointType::Torso).map(|&t| t);
        if let (Some(torso), Some(pose_torso)) =  (torso, pose.get(&JointType::Torso)){
            allign(joints, &torso, pose_torso);
            let closeness = joints.iter()
                .filter_map(|(k, v1)| {
                    pose.get(&k)
                        .map(|v2| distance(v1, v2).powi(self.curve))
                })
            .fold(0.0, |total, dist| total + dist);
            if closeness < self.threshold {
                Some(())
            } else {
                None
            }
        } else {
            None
        }
    }

}

impl Tester {
    pub fn from_name(name: String, threshold: f32, curve: i32) -> Option<Self> {
        let detector = Detector::new(threshold, curve);
        Pose::from_name(&name)
            .map(|name| Tester{ detector, name })
    }
    
    pub fn test(&self, skeleton: &Skeleton) -> (Option<Pose>, Vec<Vec2>, Vec<Vec2>) {
        let (found, debug_pose, debug_skeleton) =  self.detector.detect_pose(self.name, skeleton);
        (found.map(|_| self.name), debug_pose, debug_skeleton)
    }
}

fn joints_map(skeleton: &Skeleton) -> JointPos {
    let joints = skeleton
        .joints()
        .iter()
        .map(|j| (JointType::from_u32(j.type_), j))
        .filter(|(t, _)| t.is_some())
        .filter(|(_, j)| j.proj.x >= 0.0 && j.proj.x <= 1.0 && j.proj.y >= 0.0 && j.proj.y <= 1.0)
        .map(|(t, j)| (t.unwrap(), Vec2{ x: j.proj.x, y: j.proj.y }));
    HashMap::from_iter(joints)
}

fn allign(skeleton: &mut JointPos, torso: &Vec2, pose_torso: &Vec2){
    let difference = pose_torso - torso;
    for (_, v) in skeleton.iter_mut() {
        let temp = *v;
        *v = &temp + &difference;
    }
}

fn distance(a: &Vec2, b: &Vec2) -> f32 {
    let temp = (a.x - b.x) + (a.y - b.y);
    if temp >= 0.0 {
        temp
    } else {
        0.0
    }
}
