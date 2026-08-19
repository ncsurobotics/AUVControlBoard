//! Tools for describing the vehicle to the control board

/// Used to set relative speeds of motion in each Degree of Freedom (DoF).
///
/// There are two groups: linear (x, y, z) and angular (xrot, yrot, zrot). Expected in the format `[x, y, z, xrot, yrot, zrot]`.
///
/// Within each group, use 1.0 for the fastest DoF. Other DoFs in the group are percentages of the fastest speed (from 0.0 to 1.0).
use serde::{Deserialize, Serialize};
use serde_with::serde_as;
pub type DofSpeeds = [f32; 6];

/// Contains all parameters needed to describe a vehicle to the control board.
#[derive(Debug, Serialize, Deserialize)]
pub struct Definition<const MATRIX_ROWS: usize> {
    pub(crate) motor_matrix: MotorMatrix<MATRIX_ROWS>,
    pub(crate) dof_speeds: DofSpeeds,
    pub(crate) pid_axes: PidAxes,
}

impl<const MATRIX_ROWS: usize> Definition<MATRIX_ROWS> {
    pub const fn new(
        motor_matrix: MotorMatrix<MATRIX_ROWS>,
        dof_speeds: DofSpeeds,
        pid_axes: PidAxes,
    ) -> Self {
        Self {
            motor_matrix,
            dof_speeds,
            pid_axes,
        }
    }
}

#[serde_as]
#[derive(Debug, Serialize, Deserialize)]
#[serde(transparent)]
pub struct MotorMatrix<const ROWS: usize> {
    #[serde_as(as = "[_; ROWS]")]
    pub(crate) rows: [MotorMatrixRow; ROWS],
}

impl<const ROWS: usize> MotorMatrix<ROWS> {
    pub const fn from_rows(rows: [MotorMatrixRow; ROWS]) -> Self {
        const { assert!(ROWS <= 8, "AUVControlBoard supports at most 8 motors") };
        Self { rows }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct MotorMatrixRow {
    pub(crate) x: f32,
    pub(crate) y: f32,
    pub(crate) z: f32,
    pub(crate) pitch: f32,
    pub(crate) roll: f32,
    pub(crate) yaw: f32,
    pub(crate) inverted: bool,
}

impl MotorMatrixRow {
    pub const fn new(
        x: f32,
        y: f32,
        z: f32,
        pitch: f32,
        roll: f32,
        yaw: f32,
        inverted: bool,
    ) -> Self {
        Self {
            x,
            y,
            z,
            pitch,
            roll,
            yaw,
            inverted,
        }
    }
}

#[macro_export]
macro_rules! motor_matrix {
    ( $( $x:expr, $y:expr, $z:expr, $pitch:expr, $roll:expr, $yaw:expr, $inverted:expr );+ $(;)? ) => {
        $crate::vehicle::MotorMatrix::from_rows([
            $( $crate::vehicle::MotorMatrixRow::new($x, $y, $z, $pitch, $roll, $yaw, $inverted) ),+
        ])
    };
}

pub type PidAxes = [PidAxis; 4];

#[derive(Debug, Serialize, Deserialize)]
pub struct PidAxis {
    pub(crate) which: char,
    pub(crate) kp: f32,
    pub(crate) ki: f32,
    pub(crate) kd: f32,
    pub(crate) limit: f32,
    pub(crate) invert: bool,
}

impl PidAxis {
    pub const fn new(which: char, kp: f32, ki: f32, kd: f32, limit: f32, invert: bool) -> Self {
        Self {
            which,
            kp,
            ki,
            kd,
            limit,
            invert,
        }
    }
}

#[macro_export]
macro_rules! pid_axes {
    ( $( $which:ident, $kp:expr, $ki:expr, $kd:expr, $limit:expr, $invert:expr );* $(;)? ) => {
        [
            $(
                $crate::vehicle::PidAxis::new(
                    stringify!($which).as_bytes()[0] as char,
                    $kp,
                    $ki,
                    $kd,
                    $limit,
                    $invert,
                )
            ),*
        ]
    };
}
