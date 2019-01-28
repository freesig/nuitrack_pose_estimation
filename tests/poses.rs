use nuitrack_pose_estimation as pe;
use nuitrack_rs;
use nalgebra_glm as glm;

use nuitrack_rs::{JointType, Joint, Orientation, Vector3, SkeletonFeed, feed_to_ptr};
use pe::{Pose, Settings, PoseData, Detector};
use std::collections::HashMap;
use std::iter::FromIterator;
use glm::Vec2;


const DAB_R: (&'static str, [(JointType, (f32, f32)); 2]) = ("DabR", 
                                                       [
                                                       (JointType::Torso, (0.5645727, 0.5913842)),
                                                       (JointType::LeftShoulder, (0.68835175, 0.49775392))
                                                       ]);
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
    let orient = Orientation {
        matrix: [1.0; 9],
    };
    let joints = joints.iter()
        .map(|&(type_, v)| Joint {
            type_,
            confidence: 1.0,
            orient, 
            proj: Vector3{ x: v.x, y: v.y, z: 0.0 },
            real: Vector3{ x: 0.0, y: 0.0, z: 0.0 },
        })
    .collect();
    SkeletonFeed {
        id: 1,
        joints,
    }
}

#[test]
fn match_identity() {
    let mock_skeleton = vec![
        (3, glm::vec2(0.5645727, 0.5913842)),
        (6, glm::vec2(0.68835175, 0.49775392))
    ];
    let settings = Settings {
        joint_cutoff: 0.01,
    };
    let skeleton = vec![skeleton(&mock_skeleton)];
    let skeleton_ptr = feed_to_ptr(&skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton_ptr[0]);
    assert_eq!(result, Some(Pose::DabR));
}

#[test]
fn match_close() {
    let mock_skeleton = vec![
        (3, glm::vec2(0.5645727, 0.5913842)),
        (6, glm::vec2(0.68845175, 0.49775392))
    ];
    let settings = Settings {
        joint_cutoff: 0.01,
    };
    let skeleton = vec![skeleton(&mock_skeleton)];
    let skeleton_ptr = feed_to_ptr(&skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton_ptr[0]);
    assert_eq!(result, Some(Pose::DabR));
}

#[test]
fn nomatch_joint_cutoff() {
    let mock_skeleton = vec![
        (3, glm::vec2(0.5645727, 0.5913842)),
        (6, glm::vec2(0.70845175, 0.51775392))
    ];
    let settings = Settings {
        joint_cutoff: 0.01,
    };
    let skeleton = vec![skeleton(&mock_skeleton)];
    let skeleton_ptr = feed_to_ptr(&skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton_ptr[0]);
    assert_eq!(result, None);
}

#[test]
fn match_allign() {
    let mock_skeleton = vec![
        (3, glm::vec2(0.6645727, 0.6913842)),
        (6, glm::vec2(0.78835175, 0.59775392))
    ];
    let settings = Settings {
        joint_cutoff: 0.01,
    };
    let skeleton = vec![skeleton(&mock_skeleton)];
    let skeleton_ptr = feed_to_ptr(&skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton_ptr[0]);
    assert_eq!(result, Some(Pose::DabR));
}

#[test]
fn match_scale() {
    let mock_skeleton = vec![
        (3, glm::vec2(0.5645727, 0.5913842)),
        (6, glm::vec2(0.78835175, 0.59775392))
    ];
    let settings = Settings {
        joint_cutoff: 0.01,
    };
    let skeleton = vec![skeleton(&mock_skeleton)];
    let skeleton_ptr = feed_to_ptr(&skeleton);
    let tester = Detector::with_poses(settings, dab_r());
    let result = tester.detect(&skeleton_ptr[0]);
    assert_eq!(result, Some(Pose::DabR));
}
