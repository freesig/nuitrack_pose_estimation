use nuitrack_rs;
use nuitrack_pose_estimation as npe;
use nuitrack_rs::{Vector3, Orientation, Joint, Skeleton};
use self::npe::Pose;

fn main() {
    let (_joints, skeleton) = mock_data();
    //Skeleton -> Pose
    let pose = npe::detect(&skeleton);
    println!("Pose {:?}", pose);
}

fn mock_data() -> ([Joint; 24], Skeleton) {
    let joints = [ Joint { type_: 1, confidence: 0.75, real: Vector3 { x: 21.612122, y: 440.7368, z: 1077.2002 }, proj: Vector3 { x: 0.51787436, y: 0.013984492, z: 1077.2002 }, orient: Orientation { matrix: [0.99727744, -0.07308053, 0.009848095, 0.071373835, 0.99018276, 0.120182484, -0.018534413, -0.11915238, 0.99270296] } }
    , Joint { type_: 2, confidence: 0.75, real: Vector3 { x: 114.944756, y: 395.84195, z: 1085.5349 }, proj: Vector3 { x: 0.5943352, y: 0.06684306, z: 1085.5349 }, orient: Orientation { matrix: [0.99727744, -0.07308053, 0.009848095, 0.071373835, 0.99018276, 0.120182484, -0.018534413, -0.11915238, 0.99270296] } }
    , Joint { type_: 3, confidence: 0.75, real: Vector3 { x: 122.86625, y: 119.94968, z: 1120.819 }, proj: Vector3 { x: 0.59766203, y: 0.37287503, z: 1120.819 }, orient: Orientation { matrix: [0.99727744, -0.07308053, 0.009848095, 0.071373835, 0.99018276, 0.120182484, -0.018534413, -0.11915238, 0.99270296] } }
    , Joint { type_: 4, confidence: 0.75, real: Vector3 { x: 116.58102, y: -51.862366, z: 1106.9888 }, proj: Vector3 { x: 0.5938238, y: 0.5556514, z: 1106.9888 }, orient: Orientation { matrix: [0.9965209, 0.033623803, -0.076259606, -0.043253943, 0.9907824, -0.12837182, 0.07124033, 0.13122372, 0.98878974] } }
    , Joint { type_: 5, confidence: 0.75, real: Vector3 { x: 116.97262, y: 325.2149, z: 1094.5675 }, proj: Vector3 { x: 0.5952073, y: 0.14706467, z: 1094.5675 }, orient: Orientation { matrix: [0.99727744, -0.07308053, 0.009848095, 0.071373835, 0.99018276, 0.120182484, -0.018534413, -0.11915238, 0.99270296] } }
    , Joint { type_: 6, confidence: 0.75, real: Vector3 { x: 270.7859, y: 357.06186, z: 1094.3905 }, proj: Vector3 { x: 0.72043586, y: 0.11244049, z: 1094.3905 }, orient: Orientation { matrix: [0.45845157, 0.77111214, 0.44182375, -0.8865545, 0.36213443, 0.28788862, 0.061994795, -0.5236838, 0.849654] } }
    , Joint { type_: 7, confidence: 0.75, real: Vector3 { x: 366.2371, y: 137.08788, z: 1099.3538 }, proj: Vector3 { x: 0.7967928, y: 0.35187483, z: 1099.3538 }, orient: Orientation { matrix: [0.027525604, 0.77111214, 0.6361041, -0.84068954, 0.36213443, -0.40261614, -0.5408174, -0.5236838, 0.6582339] } }
    , Joint { type_: 8, confidence: 0.75, real: Vector3 { x: 380.3081, y: -53.102383, z: 995.9166 }, proj: Vector3 { x: 0.8402053, y: 0.56333715, z: 995.9166 }, orient: Orientation { matrix: [0.12117857, 0.77111214, 0.62506145, -0.89093643, 0.36213443, -0.27402723, -0.437662, -0.5236838, 0.7308948] } }
    , Joint { type_: 9, confidence: 0.75, real: Vector3 { x: 383.1223, y: -91.140434, z: 975.2292 }, proj: Vector3 { x: 0.84999293, y: 0.61101246, z: 975.2292 }, orient: Orientation { matrix: [0.12117857, 0.77111214, 0.62506145, -0.89093643, 0.36213443, -0.27402723, -0.437662, -0.5236838, 0.7308948] } }
    , Joint { type_: 10, confidence: 0.0, real: Vector3 { x: 0.0, y: 0.0, z: 0.0 }, proj: Vector3 { x: 0.0, y: 0.0, z: 0.0 }, orient: Orientation { matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] } }
    , Joint { type_: 11, confidence: 0.75, real: Vector3 { x: 116.97262, y: 325.2149, z: 1094.5675 }, proj: Vector3 { x: 0.5952073, y: 0.14706467, z: 1094.5675 }, orient: Orientation { matrix: [0.99727744, -0.07308053, 0.009848095, 0.071373835, 0.99018276, 0.120182484, -0.018534413, -0.11915238, 0.99270296] } }
    , Joint { type_: 12, confidence: 0.75, real: Vector3 { x: -42.060646, y: 360.95663, z: 1086.5847 }, proj: Vector3 { x: 0.46551412, y: 0.10539854, z: 1086.5847 }, orient: Orientation { matrix: [0.36326367, -0.8136611, -0.45386684, 0.92606324, 0.36877173, 0.08008937, 0.10220765, -0.44940293, 0.88746303] } }
    , Joint { type_: 13, confidence: 0.75, real: Vector3 { x: -111.50204, y: 133.86156, z: 1052.9719 }, proj: Vector3 { x: 0.4056603, y: 0.34898978, z: 1052.9719 }, orient: Orientation { matrix: [-0.11394401, -0.8136611, -0.5700635, 0.6568872, 0.36877173, -0.6576524, 0.74532944, -0.44940293, 0.49246424] } }
    , Joint { type_: 14, confidence: 0.75, real: Vector3 { x: -120.78376, y: -42.15624, z: 926.47626 }, proj: Vector3 { x: 0.3838544, y: 0.55404985, z: 926.47626 }, orient: Orientation { matrix: [0.11412638, -0.8136611, -0.570027, 0.8593319, 0.36877173, -0.354339, 0.4985217, -0.44940293, 0.7412915] } }
    , Joint { type_: 15, confidence: 0.75, real: Vector3 { x: -122.64011, y: -77.3598, z: 901.1771 }, proj: Vector3 { x: 0.3787586, y: 0.60197, z: 901.1771 }, orient: Orientation { matrix: [0.11412638, -0.8136611, -0.570027, 0.8593319, 0.36877173, -0.354339, 0.4985217, -0.44940293, 0.7412915] } }
    , Joint { type_: 16, confidence: 0.0, real: Vector3 { x: 0.0, y: 0.0, z: 0.0 }, proj: Vector3 { x: 0.0, y: 0.0, z: 0.0 }, orient: Orientation { matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] } }
    , Joint { type_: 17, confidence: 0.75, real: Vector3 { x: 211.95457, y: -66.45011, z: 1098.2517 }, proj: Vector3 { x: 0.67193705, y: 0.5718722, z: 1098.2517 }, orient: Orientation { matrix: [0.9999646, 0.0077592675, -0.0032636276, -0.008402431, 0.943411, -0.33151937, 0.0005065947, 0.33153504, 0.94344276] } }
    , Joint { type_: 18, confidence: 0.75, real: Vector3 { x: 209.23915, y: -401.82053, z: 986.91406 }, proj: Vector3 { x: 0.6888827, y: 0.9836376, z: 986.91406 }, orient: Orientation { matrix: [0.9999646, 0.00820811, -0.001866663, -0.008402431, 0.9866462, -0.16266124, 0.0005065947, 0.16267115, 0.9866802] } }
    , Joint { type_: 19, confidence: 0.0, real: Vector3 { x: 209.23915, y: -734.2269, z: 986.91406 }, proj: Vector3 { x: 0.6888827, y: 1.3837272, z: 986.91406 }, orient: Orientation { matrix: [0.9999646, 0.00820811, -0.001866663, -0.008402431, 0.9866462, -0.16266124, 0.0005065947, 0.16267115, 0.9866802] } }
    , Joint { type_: 20, confidence: 0.0, real: Vector3 { x: 0.0, y: 0.0, z: 0.0 }, proj: Vector3 { x: 0.0, y: 0.0, z: 0.0 }, orient: Orientation { matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] } }
    , Joint { type_: 21, confidence: 0.75, real: Vector3 { x: 22.58023, y: -68.280396, z: 1085.1774 }, proj: Vector3 { x: 0.5185377, y: 0.5747416, z: 1085.1774 }, orient: Orientation { matrix: [0.9929866, -0.011785427, 0.11763766, 0.067626506, 0.8727833, -0.48340043, -0.0969751, 0.4879656, 0.8674592] } }
    , Joint { type_: 22, confidence: 0.75, real: Vector3 { x: 26.493052, y: -378.45874, z: 915.9117 }, proj: Vector3 { x: 0.5257696, y: 0.99083126, z: 915.9117 }, orient: Orientation { matrix: [0.9929866, -0.032033946, 0.11380396, 0.067626506, 0.94346535, -0.32449928, -0.0969751, 0.3299196, 0.93901485] } }
    , Joint { type_: 23, confidence: 0.0, real: Vector3 { x: 26.493052, y: -710.8651, z: 915.9117 }, proj: Vector3 { x: 0.5257696, y: 1.4219362, z: 915.9117 }, orient: Orientation { matrix: [0.9929866, -0.032033946, 0.11380396, 0.067626506, 0.94346535, -0.32449928, -0.0969751, 0.3299196, 0.93901485] } }
    , Joint { type_: 24, confidence: 0.0, real: Vector3 { x: 0.0, y: 0.0, z: 0.0 }, proj: Vector3 { x: 0.0, y: 0.0, z: 0.0 }, orient: Orientation { matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] } } ];

    (joints, Skeleton {
        id: 0,
        num_joints: 24,
        joints: &joints[0],
    })
}
