use nuitrack_rs as nui;
use nalgebra_glm as glm;

use glm::{Mat2x4, Vec2};
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

pub type JointPos = HashMap<JointType, Vec2>;

pub struct Detector {
    pub settings: Settings,
    poses: PoseData,
}

pub struct Settings {
    pub joint_cutoff: f32,
    pub rotation_cutoff: f32,
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

    fn detect_pose(&self, name: Pose, skeleton: &Skeleton) -> (Option<f32>, Vec<Vec2>, Vec<Vec2>) {
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

    fn check_pose(&self, pose: &JointPos, joints: &JointPos ) -> Option<f32> {
        if let (Ok(mut pose_arms), Ok(joints_arms)) =  (arms(pose), arms(joints)){
            let rotations: Vec<f32> = pose_arms.iter_mut()
                .zip(joints_arms.iter())
                .map(|(pose_arm, joints_arm)| kabsch(pose_arm, joints_arm))
                .filter(Option::is_some)
                .map(Option::unwrap)
                .filter(|r| r >= &self.settings.rotation_cutoff)
                .collect();
            if rotations.len() <= 0 {
                return None;
            }

            let furthest_point = pose_arms.into_iter()
                .zip(joints_arms.into_iter())
                .flat_map(|(a, b)| {
                    a.into_iter()
                        .zip(b.into_iter())
                        .map(|(v1, v2)| glm::distance(&v1, &v2))
                })
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
        .map(|(t, j)| (t.unwrap(), glm::vec2(j.proj.x, j.proj.y)))
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

struct JointMissing;

/// Arms in order
fn arms(joints: &JointPos) -> Result<Vec<Vec<Vec2>>, JointMissing> {
    let right = vec![joints.get(&JointType::RightShoulder).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::RightElbow).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::RightWrist).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::RightHand).map(|&j|j).ok_or(JointMissing)?];
    let left = vec![joints.get(&JointType::LeftShoulder).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::LeftElbow).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::LeftWrist).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::LeftHand).map(|&j|j).ok_or(JointMissing)?];
    
    Ok(vec![right, left])
}

fn kabsch(a: &mut [Vec2], b: &[Vec2]) -> Option<f32> {
    let (mut total_a, mut total_b) = (0.0, 0.0);
    for i in 0..(a.len() - 1) {
        total_a += glm::l2_norm(&(a[i+1] - a[i]));
        total_b += glm::l2_norm(&(b[i+1] - b[i]));
    }

    if total_a <= 0.0 || total_b <= 0.0 {
        return None;
    }

    let scale = total_a / total_b;
    let ma: Vec<f32> = a.iter()
        .flat_map(|n| n.into_iter().map(|&i|i))
        .collect();
    let mut ma = glm::make_mat2x4(&ma[..]);
    ma /= scale;
    let mb: Vec<f32> = b.iter()
        .flat_map(|n| n.into_iter().map(|&i|i))
        .collect();
    let mut mb = glm::make_mat2x4(&mb[..]);
    mb /= scale;


    let mut a_ctr = glm::vec2(0.0, 0.0);
    let mut b_ctr = glm::vec2(0.0, 0.0);

    for i in 0..4 {
        a_ctr += ma.column(i);
        b_ctr += mb.column(i);
    }
    a_ctr /= 4.0;
    b_ctr /= 4.0;
    for i in 0..4 {
        let mut col_a = glm::column(&ma, i);
        col_a -= a_ctr;
        ma = glm::set_column(&ma, i, &col_a);
        let mut col_b = glm::column(&mb, i);
        col_b -= b_ctr;
        mb = glm::set_column(&mb, i, &col_b);
    }

    let cov = mb * ma.transpose();
    let svd = cov.svd(true, true);

    if let (Some(ref v), Some(ref u)) = (svd.v_t, svd.u) {
        let d = (v * u.transpose()).determinant();
        let d = if d > 0.0 { 1.0 } else { -1.0 };
        let i = glm::identity::<f32, glm::U2>();
        let mut col_i = glm::column(&i, 1);
        col_i[1] = d;
        let i = glm::set_column(&i, 1, &col_i);

        let r = v * i * u.transpose();
        let mad: () = glm::column(&ma, 0);

    } else {
        return None
    };

    unimplemented!()
}

/*
fn allign(skeleton: &mut JointPos, torso: &Vec2, pose_torso: &Vec2){
    let difference = pose_torso - torso;
    for (_, v) in skeleton.iter_mut() {
        let temp = *v;
        *v = &temp + &difference;
    }
}
*/

