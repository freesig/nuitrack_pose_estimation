use crate::PoseRecord;
use std::fs:: OpenOptions;
use std::io::BufReader;
use serde_json::Deserializer;

pub fn read_poses() -> Vec<PoseRecord> {
    let mut path = std::env::current_dir().expect("Couldn't find current directory");
    path.push("poses.json");
    let file = OpenOptions::new().read(true).open(path).expect("Failed to open file for writing poses");
    let reader = BufReader::new(file);
    Deserializer::from_reader(reader)
        .into_iter::<PoseRecord>()
        .map(Result::unwrap)
        .collect()
}
