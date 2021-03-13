//! Contains methods and types which are useful for logging
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::Path;

use crate::types::combined_position_orientation_array_to_isometry;
use crate::{BodyId, Error, Velocity};
use nalgebra::{DVector, Isometry3, Vector6};
use std::convert::TryInto;
use std::fmt;
use std::fmt::{Display, Formatter};
use std::time::Duration;

/// Represents the State of a Body which was recorded with
/// [`start_state_logging`](`crate::PhysicsClient::start_state_logging`) with the
/// [`GenericRobot`](`crate::types::LoggingType::GenericRobot`)
/// [`LoggingType`](`crate::types::LoggingType`).
/// Use [`read_generic_robot_log`](`read_generic_robot_log`) to read such a log file.
#[derive(Debug)]
pub struct GenericRobotLog {
    /// ongoing counter of logs from the log file.
    pub chunk_number: usize,
    /// ongoing counter of the steps in the simulation
    pub step_count: usize,
    /// simulation time stamp
    pub time_stamp: Duration,
    /// BodyId of the log entry
    pub body: BodyId,
    /// Base pose of the robot. Not the end-effector pose!
    pub base_pose: Isometry3<f64>,
    /// Base velocity of the robot. Not the end-effector velocity!
    pub base_velocity: Velocity,
    /// total number of joints of the robot.
    pub num_joints: usize,
    /// joint positions with length of num_joints
    pub joint_positions: DVector<f64>,
    /// joint velocities with length of num_joints
    pub joint_velocities: DVector<f64>,
}
impl Display for GenericRobotLog {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        writeln!(f, "chunk #  {}", self.chunk_number)?;
        writeln!(f, "step count = {}", self.step_count)?;
        writeln!(f, "body = {:?}", self.body)?;
        writeln!(f, "Base Pose:")?;
        writeln!(f, "{}", self.base_pose.to_homogeneous())?;
        writeln!(f, "Base Velocity:")?;
        writeln!(f, "{}", self.base_velocity.to_vector())?;
        writeln!(f, "num joints = {}", self.num_joints)?;
        writeln!(f, "joint positions:")?;
        writeln!(f, "{:?}", self.joint_positions.as_slice())?;
        writeln!(f, "joint velocities:")?;
        writeln!(f, "{:?}", self.joint_velocities.as_slice())
    }
}
impl Default for GenericRobotLog {
    fn default() -> Self {
        GenericRobotLog {
            chunk_number: 0,
            step_count: 0,
            time_stamp: Default::default(),
            body: BodyId(-1),
            base_pose: Isometry3::identity(),
            base_velocity: Vector6::zeros().into(),
            num_joints: 0,
            joint_positions: DVector::identity(1),
            joint_velocities: DVector::identity(1),
        }
    }
}
fn calc_size(fmt: &str) -> usize {
    let mut size = 0;
    for c in fmt.chars() {
        size += match c {
            'I' | 'i' | 'f' => 4,
            'B' => 1,
            _ => {
                panic!("can not determine data type")
            }
        };
    }
    size
}

/// reads log files which were generated with [`GenericRobot`](`crate::types::LoggingType::GenericRobot`)
/// [`LoggingType`](`crate::types::LoggingType`).
/// it returns a list of all entries in the log or an error if the file could not be openend.
/// # Arguments
/// * `filename` - location of the log file.
/// # Example
/// ```no_run
///# use rubullet::logging_utils::read_generic_robot_log;
/// let logs = read_generic_robot_log("LOG0001.txt").unwrap();
/// ```
pub fn read_generic_robot_log<P: AsRef<Path>>(filename: P) -> Result<Vec<GenericRobotLog>, Error> {
    let file = File::open(filename).map_err(|_| Error::new("could not open file"))?;
    let mut reader = BufReader::new(file);
    let mut key_buf = String::new();
    reader
        .read_line(&mut key_buf)
        .expect("error while reading file");
    let mut fmt_buf = String::new();
    reader
        .read_line(&mut fmt_buf)
        .expect("error while reading file");
    let fmt = fmt_buf.strip_suffix("\n").unwrap();
    assert_eq!("IfifffffffffffffI", &fmt[0..17]);
    let sz = calc_size(fmt);
    let mut chunk_index = 0;
    let mut logs = Vec::<GenericRobotLog>::new();
    loop {
        let mut check_buf = [0_u8; 2];
        match reader.read_exact(&mut check_buf) {
            Ok(_) => {}
            Err(_) => {
                return Ok(logs);
            }
        }
        assert_eq!(
            &check_buf,
            &[170_u8, 187_u8],
            "Error, expected aabb terminal"
        );

        let mut buf = vec![0_u8; sz];
        reader.read_exact(&mut buf).unwrap();
        let mut log = GenericRobotLog {
            chunk_number: chunk_index,
            ..Default::default()
        };
        chunk_index += 1;
        log.step_count = u32::from_le_bytes(buf[0..4].try_into().unwrap()) as usize;
        log.time_stamp = Duration::from_secs_f32(f32::from_le_bytes(buf[4..8].try_into().unwrap()));
        log.body = BodyId(i32::from_le_bytes(buf[8..12].try_into().unwrap()));
        assert!(log.body.0 >= 0);
        let mut pose_elements = [0.; 7];
        for i in 0..7 {
            pose_elements[i] =
                f32::from_le_bytes(buf[12 + 4 * i..16 + 4 * i].try_into().unwrap()) as f64;
        }
        log.base_pose = combined_position_orientation_array_to_isometry(pose_elements);
        let mut velocity_elements = [0.; 6];
        for i in 0..6 {
            velocity_elements[i] =
                f32::from_le_bytes(buf[40 + 4 * i..44 + 4 * i].try_into().unwrap()) as f64;
        }
        let vel_vec: Vector6<f64> = velocity_elements.into();
        log.base_velocity = vel_vec.into();
        log.num_joints = u32::from_le_bytes(buf[64..68].try_into().unwrap()) as usize;
        let remaining = sz - 68;
        assert_eq!(remaining % 8, 0);
        let mut joint_positions = DVector::<f64>::from(vec![0.; log.num_joints]);
        for i in 0..log.num_joints {
            joint_positions[i] =
                f32::from_le_bytes(buf[68 + 4 * i..72 + 4 * i].try_into().unwrap()) as f64;
        }
        let mut joint_velocities = DVector::<f64>::from(vec![0.; log.num_joints]);
        let start_byte = 68 + remaining / 2;
        for i in 0..log.num_joints {
            joint_velocities[i] = f32::from_le_bytes(
                buf[start_byte + 4 * i..start_byte + 4 + 4 * i]
                    .try_into()
                    .unwrap(),
            ) as f64;
        }
        log.joint_positions = joint_positions;
        log.joint_velocities = joint_velocities;
        logs.push(log);
    }
}
