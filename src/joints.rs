use crate::JointType;
use serde::{Serialize, Deserialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u32)]
pub enum Joint2DType {
    None = 0,
    Head,
    Neck,
    Torso,
    Waist,
    LeftCollar,
    LeftShoulder,
    LeftElbow,
    LeftWrist,
    LeftHand,
    LeftFingertip, // Possibly unused in current version.
    RightCollar,
    RightShoulder,
    RightElbow,
    RightWrist,
    RightHand,
    RightFingertip, // Possibly unused in current version.
    LeftHip,
    LeftKnee,
    LeftAnkle,
    LeftFoot, // Possibly unused in current version.
    RightHip,
    RightKnee,
    RightAnkle,
    RightFoot, // Possibly unused in current version.
}

impl From<JointType> for Joint2DType {
    fn from(item: JointType) -> Self {
        match item {
            JointType::None => Joint2DType::None,
            JointType::Head => Joint2DType::Head,
            JointType::Neck => Joint2DType::Neck,
            JointType::Torso => Joint2DType::Torso,
            JointType::Waist => Joint2DType::Waist,
            JointType::LeftCollar => Joint2DType::LeftCollar,
            JointType::LeftShoulder => Joint2DType::LeftShoulder,
            JointType::LeftElbow => Joint2DType::LeftElbow,
            JointType::LeftWrist => Joint2DType::LeftWrist,
            JointType::LeftHand => Joint2DType::LeftHand,
            JointType::LeftFingertip => Joint2DType::LeftFingertip,
            JointType::RightCollar => Joint2DType::RightCollar,
            JointType::RightShoulder => Joint2DType::RightShoulder,
            JointType::RightElbow => Joint2DType::RightElbow,
            JointType::RightWrist => Joint2DType::RightWrist,
            JointType::RightHand => Joint2DType::RightHand,
            JointType::RightFingertip => Joint2DType::RightFingertip,
            JointType::LeftHip => Joint2DType::LeftHip,
            JointType::LeftKnee => Joint2DType::LeftKnee,
            JointType::LeftAnkle => Joint2DType::LeftAnkle,
            JointType::LeftFoot => Joint2DType::LeftFoot,
            JointType::RightHip => Joint2DType::RightHip,
            JointType::RightKnee => Joint2DType::RightKnee,
            JointType::RightAnkle => Joint2DType::RightAnkle,
            JointType::RightFoot => Joint2DType::RightFoot,
        }
    }
}

pub fn j2d_to_j(item: Joint2DType) -> JointType {
    match item {
        Joint2DType::None => JointType::None,
        Joint2DType::Head => JointType::Head,
        Joint2DType::Neck => JointType::Neck,
        Joint2DType::Torso => JointType::Torso,
        Joint2DType::Waist => JointType::Waist,
        Joint2DType::LeftCollar => JointType::LeftCollar,
        Joint2DType::LeftShoulder => JointType::LeftShoulder,
        Joint2DType::LeftElbow => JointType::LeftElbow,
        Joint2DType::LeftWrist => JointType::LeftWrist,
        Joint2DType::LeftHand => JointType::LeftHand,
        Joint2DType::LeftFingertip => JointType::LeftFingertip,
        Joint2DType::RightCollar => JointType::RightCollar,
        Joint2DType::RightShoulder => JointType::RightShoulder,
        Joint2DType::RightElbow => JointType::RightElbow,
        Joint2DType::RightWrist => JointType::RightWrist,
        Joint2DType::RightHand => JointType::RightHand,
        Joint2DType::RightFingertip => JointType::RightFingertip,
        Joint2DType::LeftHip => JointType::LeftHip,
        Joint2DType::LeftKnee => JointType::LeftKnee,
        Joint2DType::LeftAnkle => JointType::LeftAnkle,
        Joint2DType::LeftFoot => JointType::LeftFoot,
        Joint2DType::RightHip => JointType::RightHip,
        Joint2DType::RightKnee => JointType::RightKnee,
        Joint2DType::RightAnkle => JointType::RightAnkle,
        Joint2DType::RightFoot => JointType::RightFoot,
    }
}
