use nalgebra_glm as glm;
use nuitrack_pose_estimation as pe;
use nuitrack_rs;

use glm::Vec2;
use nuitrack_rs::{Joint, JointType, Orientation, SkeletonFeed, Vector3};
use pe::{Detector, Pose, PoseData, Settings};
use std::collections::HashMap;
use std::iter::FromIterator;

const DAB_R: (&'static str, [(JointType, (f32, f32)); 8]) = (
    "DabR",
    [
        (JointType::LeftShoulder, (0.68835175, 0.49775392)),
        (JointType::LeftElbow, (0.5864927, 0.5303629)),
        (JointType::LeftWrist, (0.45776764, 0.40316057)),
        (JointType::LeftHand, (0.43292272, 0.37860954)),
        (JointType::RightShoulder, (0.51725876, 0.48693466)),
        (JointType::RightElbow, (0.3500515, 0.41818976)),
        (JointType::RightWrist, (0.20907341, 0.3226182)),
        (JointType::RightHand, (0.1777911, 0.30141133)),
    ],
);
/*
{"name":"DabR","data":[["Head",[0.5808275,0.42642814]],["Neck",[0.58105177,0.45120373]],["Torso",[0.5645727,0.5913842]],["Waist",[0.5587295,0.68986714]],["LeftCollar",[0.5765486,0.4895107]],["LeftShoulder",[0.68835175,0.49775392]],["LeftElbow",[0.5864927,0.5303629]],["LeftWrist",[0.45776764,0.40316057]],["LeftHand",[0.43292272,0.37860954]],["LeftFingertip",[0.0,0.0]],["RightCollar",[0.5765486,0.4895107]],["RightShoulder",[0.51725876,0.48693466]],["RightElbow",[0.3500515,0.41818976]],["RightWrist",[0.20907341,0.3226182]],["RightHand",[0.1777911,0.30141133]],["RightFingertip",[0.0,0.0]],["LeftHip",[0.6397388,0.70486915]],["LeftKnee",[0.6397388,0.91291016]],["LeftAnkle",[0.6397388,1.1086042]],["LeftFoot",[0.0,0.0]],["RightHip",[0.4802191,0.7070243]],["RightKnee",[0.4802191,0.90910274]],["RightAnkle",[0.4802191,1.099188]],["RightFoot",[0.0,0.0]]]}
*/

fn dab_r() -> PoseData {
    let name = Pose::DabR;
    let pose = HashMap::from_iter(DAB_R.1.iter().map(|&(ty, (x, y))| (ty, glm::vec2(x, y))));
    let mut poses = HashMap::new();
    poses.insert(name, pose);
    poses
}

fn skeleton(joints: &[(u32, Vec2)]) -> SkeletonFeed {
    let orient = Orientation { matrix: [1.0; 9] };
    let joints = joints
        .iter()
        .map(|&(type_, v)| Joint {
            type_,
            confidence: 1.0,
            orient,
            proj: Vector3 {
                x: v.x,
                y: v.y,
                z: 0.0,
            },
            real: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        })
        .collect();
    SkeletonFeed { id: 1, joints }
}

fn identity_mock() -> Vec<(u32, Vec2)> {
    vec![
        (6, glm::vec2(0.68835175, 0.49775392)),
        (7, glm::vec2(0.5864927, 0.5303629)),
        (8, glm::vec2(0.45776764, 0.40316057)),
        (9, glm::vec2(0.43292272, 0.37860954)),
        (12, glm::vec2(0.51725876, 0.48693466)),
        (13, glm::vec2(0.3500515, 0.41818976)),
        (14, glm::vec2(0.20907341, 0.3226182)),
        (15, glm::vec2(0.1777911, 0.30141133)),
    ]
}

#[test]
fn match_identity() {
    let mock_skeleton = identity_mock();
    let settings = Settings {
        rotation_cutoff: 0.34,
        joint_cutoff: 0.01,
    };
    let skeleton = skeleton(&mock_skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton.joints);
    assert_eq!(result, Some(Pose::DabR));
}

#[test]
fn match_close() {
    let mut mock_skeleton = identity_mock();
    mock_skeleton[5].1 = glm::vec2(0.3501515, 0.41818976);
    mock_skeleton[6].1 = glm::vec2(0.20917341, 0.3226182);
    mock_skeleton[7].1 = glm::vec2(0.1778911, 0.30141133);
    let settings = Settings {
        rotation_cutoff: 0.34,
        joint_cutoff: 0.01,
    };
    let skeleton = skeleton(&mock_skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton.joints);
    assert_eq!(result, Some(Pose::DabR));
}

#[test]
fn nomatch_joint_cutoff() {
    let mut mock_skeleton = identity_mock();
    mock_skeleton[5].1 = glm::vec2(0.4500515, 0.41818976);
    mock_skeleton[6].1 = glm::vec2(0.30907341, 0.3226182);
    mock_skeleton[7].1 = glm::vec2(0.2777911, 0.30141133);
    let settings = Settings {
        rotation_cutoff: 0.34,
        joint_cutoff: 0.01,
    };
    let skeleton = skeleton(&mock_skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton.joints);
    assert_eq!(result, None);
}

#[test]
fn nomatch_rotation_cutoff() {
    let mut mock_skeleton = identity_mock();
    mock_skeleton[5].1 = glm::vec2(0.264197101006, 0.663852311753);
    mock_skeleton[5].1 = glm::vec2(0.139708055532, 0.528565662165);
    mock_skeleton[6].1 = glm::vec2(0.0531405166554, 0.38074962801);
    mock_skeleton[7].1 = glm::vec2(0.0337515260423, 0.348309201115);
    let settings = Settings {
        rotation_cutoff: 0.34,
        joint_cutoff: 0.01,
    };
    let skeleton = skeleton(&mock_skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton.joints);
    assert_eq!(result, None);
}

#[test]
fn match_rotation_close() {
    // 15deg rotation
    let mut mock_skeleton = identity_mock();
    mock_skeleton[0].1 = glm::vec2(0.536068538626, 0.658951909094);
    mock_skeleton[1].1 = glm::vec2(0.429240426524, 0.664086702989);
    mock_skeleton[2].1 = glm::vec2(0.337823952165, 0.507902190168);
    mock_skeleton[3].1 = glm::vec2(0.320179876426, 0.477757377759);
    let settings = Settings {
        rotation_cutoff: 0.34,
        joint_cutoff: 0.08,
    };
    let skeleton = skeleton(&mock_skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton.joints);
    assert_eq!(result, Some(Pose::DabR));
}

#[test]
fn match_allign() {
    let mut mock_skeleton = identity_mock();
    for m in mock_skeleton.iter_mut() {
        m.1.x += 0.2;
        m.1.y += 0.2;
    }
    let settings = Settings {
        rotation_cutoff: 0.34,
        joint_cutoff: 0.01,
    };
    let skeleton = skeleton(&mock_skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton.joints);
    assert_eq!(result, Some(Pose::DabR));
}

#[test]
fn match_scale() {
    let mut mock_skeleton = identity_mock();
    for m in mock_skeleton.iter_mut() {
        m.1 *= 1.2;
    }
    let settings = Settings {
        rotation_cutoff: 0.34,
        joint_cutoff: 0.01,
    };
    let skeleton = skeleton(&mock_skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton.joints);
    assert_eq!(result, Some(Pose::DabR));
}
