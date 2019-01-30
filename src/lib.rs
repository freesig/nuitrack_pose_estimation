use nuitrack_rs as nui;
use nalgebra_glm as glm;
use nalgebra as na;

use glm::{Vec2, Mat2x2};
use na::MatrixMN;
use std::collections::HashMap;
use std::iter::FromIterator;
use self::nui::{Skeleton, JointType};
use std::cmp::Ordering::Equal;
use std::cell::RefCell;

type Mat2x8 = MatrixMN<f32, na::U2, na::U8>;

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
    pub lasts: Lasts,
}

pub struct Lasts {
    pub capture: bool,
    pub skeleton: RefCell<Option<Vec<Vec2>>>,
    pub pose: RefCell<Option<Vec<Vec2>>>,
    pub max_dist: RefCell<Option<f32>>,
    pub rotations: RefCell<Option<f32>>,
}

pub struct Settings {
    pub joint_cutoff: f32,
    pub rotation_cutoff: f32,
}

pub struct Tester {
    pub detector: Detector,
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
        let lasts = Lasts {
            capture: false,
            skeleton: RefCell::new(None),
            pose: RefCell::new(None),
            rotations: RefCell::new(None),
            max_dist: RefCell::new(None),
        };
        Detector{ poses, settings, lasts }
    }
    
    pub fn detect(&self, skeleton: &Skeleton) -> Option<Pose> {
        let joints = joints_map(skeleton);
        self.check_poses(joints)
    }

    fn detect_pose(&self, name: Pose, skeleton: &Skeleton) -> Option<f32> {
        let mut joints = joints_map(skeleton);
        let pose = self.poses.get(&name).expect(&format!("Pose {:?} doesn't exist", name));
        self.check_pose(pose, &mut joints)
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
        if let (Ok(mut pose_arms), Ok(mut joints_arms)) =  (arms(pose), arms(joints)){
            let rotation = if let Some(rotation) = kabsch(&mut pose_arms, &mut joints_arms) {
                if rotation.abs() <= self.settings.rotation_cutoff {
                    rotation
                } else {
                    return None;
                }
            } else {
                return None;
            };

            if self.lasts.capture {
                self.debug(&joints_arms, &pose_arms);
            }

            let furthest_point = pose_arms.into_iter()
                .zip(joints_arms.into_iter())
                .map(|(v1, v2)| glm::distance(&v1, &v2))
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(Equal));
            if self.lasts.capture {
                self.debug_info(rotation, &furthest_point);
            }
            match furthest_point {
                Some(fp) if fp < self.settings.joint_cutoff => Some(fp),
                _ => None,
            }
        } else {
            None
        }
    }

    fn debug(&self, joints_arms: &Vec<Vec2>, pose_arms: &Vec<Vec2>) {
        self.lasts.skeleton.replace(Some(joints_arms.clone()));
        self.lasts.pose.replace(Some(pose_arms.clone()));
    }

    fn debug_info(&self, rotation: f32, furthest_point: &Option<f32>) {
        self.lasts.max_dist.replace(furthest_point.clone());
        self.lasts.rotations.replace(Some(rotation));
    }

}

impl Tester {
    pub fn test_pose(settings: Settings, name: Pose, pose: JointPos) -> Self {
        let mut poses = HashMap::new();
        poses.insert(name, pose);
        let mut detector = Detector::with_poses(settings, poses);
        detector.lasts.capture = true;
        Tester {
            name,
            detector,
        }
    }
    
    pub fn test(&self, skeleton: &Skeleton) -> Option<Pose> {
        let found  =  self.detector.detect_pose(self.name, skeleton);
        found.map(|_| self.name) 
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
fn arms(joints: &JointPos) -> Result<Vec<Vec2>, JointMissing> {
    let arms = vec![joints.get(&JointType::RightShoulder).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::RightElbow).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::RightWrist).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::RightHand).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::LeftShoulder).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::LeftElbow).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::LeftWrist).map(|&j|j).ok_or(JointMissing)?,
        joints.get(&JointType::LeftHand).map(|&j|j).ok_or(JointMissing)?];
    
    Ok(arms)
}

fn kabsch(a: &mut [Vec2], b: &mut [Vec2]) -> Option<f32> {
    let scale = get_scale(a, b);
    let scale = if scale.is_none() { return scale; } else { scale.unwrap() };

    let mut ma = make_mat(a);
    ma /= scale;

    let mut mb = make_mat(b);

    center(&mut ma, &mut mb);

    write_joints(b, &mb);

    rotation_matrix(&ma, &mb)
        .map(|(r, angle)| {
            let mad = r * ma;
            write_joints(a, &mad);
            angle
        })
}

fn get_scale(a: &[Vec2], b: &[Vec2]) -> Option<f32> {
    let (mut total_a, mut total_b) = (0.0, 0.0);
    for i in 0..(a.len() - 1) {
        total_a += glm::l2_norm(&(a[i+1] - a[i]));
        total_b += glm::l2_norm(&(b[i+1] - b[i]));
    }
    if total_a <= 0.0 || total_b <= 0.0 {
        None
    } else {
        Some(total_a / total_b)
    }
}

fn make_mat(a: &[Vec2]) -> Mat2x8 {
    Mat2x8::from_columns(&a[..])
}

fn center(ma: &mut Mat2x8, mb: &mut Mat2x8) {
    let mut a_ctr = glm::vec2(0.0, 0.0);
    let mut b_ctr = glm::vec2(0.0, 0.0);

    for i in 0..8 {
        a_ctr += ma.column(i);
        b_ctr += mb.column(i);
    }
    a_ctr /= 8.0;
    b_ctr /= 8.0;
    for i in 0..8 {
        let mut col_a = glm::column(&ma, i);
        col_a -= a_ctr;
        *ma = glm::set_column(&ma, i, &col_a);
        let mut col_b = glm::column(&mb, i);
        col_b -= b_ctr;
        *mb = glm::set_column(&mb, i, &col_b);
    }
}

fn write_joints(a: &mut [Vec2], m: &Mat2x8) {
    for (n, v) in a.iter_mut().enumerate() {
        *v = Vec2::from(m.column(n));
    }
}

fn rotation_matrix(ma: &Mat2x8, mb: &Mat2x8) -> Option<(Mat2x2, f32)> {
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
        let rot = na::Rotation2::from_matrix_unchecked(r);
        Some((r, rot.angle()))
    } else { None }
}

