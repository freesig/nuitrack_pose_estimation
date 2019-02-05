use crate::{glm, JointType, Pose, PoseData, Vec2};
use std::collections::HashMap;

const POSE_DATA: &'static [(Pose, [(JointType, [f32; 2]); 8]); 6] = &[
    (
        Pose::DabR,
        [
            (JointType::LeftShoulder, [0.68835175, 0.49775392]),
            (JointType::LeftElbow, [0.5864927, 0.5303629]),
            (JointType::LeftWrist, [0.45776764, 0.40316057]),
            (JointType::LeftHand, [0.43292272, 0.37860954]),
            (JointType::RightShoulder, [0.51725876, 0.48693466]),
            (JointType::RightElbow, [0.3500515, 0.41818976]),
            (JointType::RightWrist, [0.20907341, 0.3226182]),
            (JointType::RightHand, [0.1777911, 0.30141133]),
        ],
    ),
    (
        Pose::DabL,
        [
            (JointType::LeftShoulder, [0.54779404, 0.47681838]),
            (JointType::LeftElbow, [0.69728184, 0.38596952]),
            (JointType::LeftWrist, [0.8303246, 0.27515584]),
            (JointType::LeftHand, [0.85951024, 0.25084662]),
            (JointType::RightShoulder, [0.3391317, 0.4949254]),
            (JointType::RightElbow, [0.3619585, 0.45994073]),
            (JointType::RightWrist, [0.5184539, 0.35749125]),
            (JointType::RightHand, [0.55025893, 0.33667013]),
        ],
    ),
    (
        Pose::HandsUp,
        [
            (JointType::LeftShoulder, [0.622165, 0.4894577]),
            (JointType::LeftElbow, [0.8113499, 0.52536154]),
            (JointType::LeftWrist, [0.8465704, 0.40550312]),
            (JointType::LeftHand, [0.8546598, 0.3779743]),
            (JointType::RightShoulder, [0.38535437, 0.49258333]),
            (JointType::RightElbow, [0.20178324, 0.5451793]),
            (JointType::RightWrist, [0.16273132, 0.4116555]),
            (JointType::RightHand, [0.15390228, 0.38146776]),
        ],
    ),
    (
        Pose::FlyingR,
        [
            (JointType::LeftShoulder, [0.59306926, 0.46183428]),
            (JointType::LeftElbow, [0.7451703, 0.386608]),
            (JointType::LeftWrist, [0.8828938, 0.28804398]),
            (JointType::LeftHand, [0.9125005, 0.26685545]),
            (JointType::RightShoulder, [0.37028378, 0.49336982]),
            (JointType::RightElbow, [0.2942572, 0.6027725]),
            (JointType::RightWrist, [0.16613771, 0.6731666]),
            (JointType::RightHand, [0.14067307, 0.68715787]),
        ],
    ),
    (
        Pose::FlyingL,
        [
            (JointType::LeftShoulder, [0.6795717, 0.508412]),
            (JointType::LeftElbow, [0.7892335, 0.62164533]),
            (JointType::LeftWrist, [0.89885813, 0.6966205]),
            (JointType::LeftHand, [0.91965455, 0.7108437]),
            (JointType::RightShoulder, [0.4441179, 0.4797545]),
            (JointType::RightElbow, [0.3059799, 0.40138206]),
            (JointType::RightWrist, [0.1715625, 0.2713209]),
            (JointType::RightHand, [0.14034835, 0.24111839]),
        ],
    ),
    (
        Pose::Roof,
        [
            (JointType::LeftShoulder, [0.56592184, 0.4550135]),
            (JointType::LeftElbow, [0.63900703, 0.33216816]),
            (JointType::LeftWrist, [0.6491533, 0.17541593]),
            (JointType::LeftHand, [0.6513979, 0.14073782]),
            (JointType::RightShoulder, [0.3340116, 0.45822495]),
            (JointType::RightElbow, [0.23958142, 0.35273913]),
            (JointType::RightWrist, [0.21619011, 0.18642789]),
            (JointType::RightHand, [0.21106102, 0.14996022]),
        ],
    ),
];

pub fn load() -> PoseData {
    POSE_DATA
        .iter()
        .map(|&(pose, data)| {
            let data: HashMap<JointType, Vec2> = data
                .into_iter()
                .map(|&(jt, pt)| (jt, glm::vec2(pt[0], pt[1])))
                .collect();
            (pose, data)
        })
        .collect()
}
