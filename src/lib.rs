use nuitrack_rs as nui;
use serde::{Serialize, Deserialize};

mod poses;
mod joints;

use std::collections::HashMap;
use std::iter::FromIterator;
use self::nui::{Skeleton, JointType};

pub use self::joints::Joint2DType;

const THRESHOLD: f32 = 3.0;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseRecord {
    pub name: String,
    pub data: Vec<Joint2D>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Joint2D(Joint2DType, [f32; 2]);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Pose {
    Star,
}

pub type JointPos = HashMap<JointType, Vec2>;


pub struct Detector {
    poses: HashMap<Pose, Vec<(JointType, Vec2)>>,
}

pub struct Tester {
    detector: Detector,
    name: String,
}

#[derive(Clone, Debug)]
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
            "Star" => Some(Pose::Star),
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
    pub fn new() -> Self {
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
        Detector{ poses }
    }

    pub fn detect(&self, skeleton: &Skeleton) -> Option<Pose> {
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

        for (name, p) in &self.poses {

            let pose_pos: JointPos = HashMap::from_iter(p.iter().cloned());
            let pose_pos = if let (Some(torso), Some(pose_torso)) = (joint_pos.get(&JointType::Torso), pose_pos.get(&JointType::Torso)) {
                allign(&p, torso, pose_torso)
            } else {
                pose_pos
            };

            // JointPos -> Pose -> [Distance]
            // [Distance] -> closeness: f32
            let closeness = joint_pos.iter()
                .filter_map(|(k, v1)| {
                    pose_pos.get(&k)
                        .map(|v2| distance(v1, v2))
                })
            .fold(0.0, |total, dist| total + dist);
            if closeness < THRESHOLD {
                return Some(Pose::Star)
            }
        }
        None
    }

}

impl Tester {
    pub fn new(name: String) -> Self {
        let detector = Detector::new();
        //let pose = detector.get_pose(name);
        Tester{ detector, name }
    }
    
    pub fn test(&self, skeleton: &Skeleton) -> Option<(String, Vec<Vec2>)> {
        /*
        let mut debug_star = Vec::new();
            debug_star = p.iter()
                    .map(|(_, v)| v.clone())
                    .collect();
                    */
        unimplemented!()
    }
}

fn allign(pose: &[(JointType, Vec2)], torso: &Vec2, pose_torso: &Vec2) -> HashMap<JointType, Vec2> {
    let difference = torso - pose_torso;
    let dp = translate(pose, &difference);
    HashMap::from_iter(dp.into_iter())
}

fn distance(a: &Vec2, b: &Vec2) -> f32 {
    ((a.x - b.x) + (a.y - b.y)).sqrt()
}

fn translate(pose: &[(JointType, Vec2)], difference: &Vec2) -> Vec<(JointType, Vec2)> {
    pose.iter()
        .map(|(j, v)| (j.clone(), v + difference))
        .collect()
}
