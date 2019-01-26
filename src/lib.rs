use nuitrack_rs as nui;
use nalgebra as na;

use na::{Point2, distance};
use std::collections::HashMap;
use std::iter::FromIterator;
use self::nui::{Skeleton, JointType};
use std::cmp::Ordering::Equal;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Pose {
    DabR,
    DabL,
    HandsUp,
    Roof,
    FlyingR,
    FlyingL,
}

pub type PoseData = HashMap<Pose, JointPos>;

pub type JointPos = HashMap<JointType, Point2<f32>>;

pub struct Detector {
    pub settings: Settings,
    poses: PoseData,
}

pub struct Settings {
    pub joint_cutoff: f32,
}

pub struct Tester {
    detector: Detector,
    name: Pose,
}

impl Pose {
    pub fn from_name(name: &str) -> Option<Self> {
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

impl Detector {
    pub fn with_poses(settings: Settings, poses: PoseData) -> Self {
        Detector{ poses, settings }
    }
    
    pub fn detect(&self, skeleton: &Skeleton) -> Option<Pose> {
        let joints = joints_map(skeleton);
        self.check_poses(joints)
    }

    fn detect_pose(&self, name: Pose, skeleton: &Skeleton) -> (Option<f32>, Vec<Point2<f32>>, Vec<Point2<f32>>) {
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
        let mut estimation = None;
        let mut max_closeness = std::f32::MAX;
        for (name, pose) in &self.poses {
            if let Some(closeness) = self.check_pose(pose, &mut joints) {
                if closeness < max_closeness {
                    estimation = Some(*name);
                    max_closeness = closeness;
                }

            }
        }
        estimation
    }

    fn check_pose(&self, pose: &JointPos, joints: &mut JointPos ) -> Option<f32> {
        let torso = joints.get(&JointType::Torso).map(|&t| t);
        if let (Some(torso), Some(pose_torso)) =  (torso, pose.get(&JointType::Torso)){
            allign(joints, &torso, pose_torso);
            let furthest_point = joints.iter()
                .filter_map(|(k, v1)| {
                    pose.get(&k)
                        .map(|v2| distance(v1, v2))
                })
            .inspect(|d| println!("d: {}", d))
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(Equal));
            match furthest_point {
                Some(fp) if fp < self.settings.joint_cutoff => Some(fp),
                _ => None,
            }
        } else {
            None
        }
    }

}

impl Tester {
    pub fn set_joint_cutoff(&mut self, joint_cutoff: f32) {
        self.detector.settings.joint_cutoff = joint_cutoff;
    }
    pub fn test_pose(settings: Settings, name: Pose, pose: JointPos) -> Self {
        let mut poses = HashMap::new();
        poses.insert(name, pose);
        Tester {
            name,
            detector: Detector::with_poses(settings, poses),
        }
    }
    
    pub fn test(&self, skeleton: &Skeleton) -> (Option<Pose>, Vec<Point2<f32>>, Vec<Point2<f32>>) {
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
        .map(|(t, j)| (t.unwrap(), Point2::new(j.proj.x, j.proj.y)))
        .filter(|(t, _)| available_joints(t));
    HashMap::from_iter(joints)
}

fn available_joints(joint_type: &JointType) -> bool {
    match joint_type {
            //JointType::Head |
            //JointType::Neck |
            JointType::Torso |
            JointType::LeftShoulder |
            JointType::LeftElbow |
            JointType::LeftWrist |
            JointType::LeftHand |
            JointType::RightShoulder |
            JointType::RightElbow |
            JointType::RightWrist |
            JointType::RightHand => true,
            _ => false,
    }
}

fn allign(skeleton: &mut JointPos, torso: &Point2<f32>, pose_torso: &Point2<f32>){
    let difference = pose_torso - torso;
    for (_, v) in skeleton.iter_mut() {
        let temp = *v;
        *v = &temp + &difference;
    }
}
