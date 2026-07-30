//! Tools for describing the vehicle to the control board

use super::util::{ControlBoardError, Result};

/// Thruster inversions used to invert the positive and negative direction of thrusters.
///
/// If `true`, the thruster of the corresponding index will be inverted
pub type ThrusterInversions = Vec<bool>;

/// Used to set relative speeds of motion in each Degree of Freedom (DoF).
///
/// There are two groups: linear (x, y, z) and angular (xrot, yrot, zrot). Expected in the format `[x, y, z, xrot, yrot, zrot]`.
///
/// Within each group, use 1.0 for the fastest DoF. Other DoFs in the group are percentages of the fastest speed (from 0.0 to 1.0).
pub type DofSpeeds = [f32; 6];

/// Contains all parameters needed to describe a vehicle to the control board.
pub struct VehicleDefinition {
    pub(crate) motor_matrix: MotorMatrix,
    pub(crate) thruster_inversions: ThrusterInversions,
    pub(crate) dof_speeds: DofSpeeds,
    pub(crate) pid_axes: PidAxes,
}

impl VehicleDefinition {
    /// Contructs a new populated `VehicleDefinition`
    pub fn new(
        motor_matrix: MotorMatrix,
        thruster_inversions: ThrusterInversions,
        dof_speeds: DofSpeeds,
        pid_axes: PidAxes,
    ) -> Result<Self> {
        let rows = motor_matrix.0.iter().map(|r| r.is_some() as u8).sum();
        let inversions = thruster_inversions.len();

        if rows != inversions as u8 {
            Err(ControlBoardError::ThrusterMismatch { rows, inversions })
        } else {
            Ok(Self {
                motor_matrix,
                thruster_inversions,
                dof_speeds,
                pid_axes,
            })
        }
    }
}

/// Contains a *populated* motor matrix.
/// Can only be created using [`MotorMatrixBuilder`].
#[derive(Debug)]
pub struct MotorMatrix(pub(crate) [Option<MotorMatrixRowParams>; 8]);

impl MotorMatrix {
    /// Creates a new [`MotorMatrixBuilder`]
    pub fn builder(thrusters_in_use: u8) -> MotorMatrixBuilder {
        MotorMatrixBuilder::new(thrusters_in_use)
    }
}

/// A builder for [`MotorMatrix`]. This struct makes it cleaner to define thruster parameters,
/// as well as validate that the correct amount of thrusters have a definition.
#[derive(Debug)]
pub struct MotorMatrixBuilder {
    pub(crate) params: [Option<MotorMatrixRowParams>; 8],
    pub(crate) thrusters_in_use: u8,
}

impl MotorMatrixBuilder {
    /// Creates an unpopulated builder.
    pub fn new(thrusters_in_use: u8) -> Self {
        MotorMatrixBuilder {
            params: [None, None, None, None, None, None, None, None],
            thrusters_in_use,
        }
    }

    /// Sets the parameters for a specific thruster.
    pub fn set_row(mut self, thruster: u8, parameters: MotorMatrixRowParams) -> Self {
        let thruster_index = thruster - 1;
        self.params[thruster_index as usize] = Some(parameters);
        self
    }

    /// Build the populated [`MotorMatrix`]
    pub fn build(self) -> MotorMatrix {
        // Make sure that we currently have parameters for `thrusters_in_use` thrusters.
        let thrusters_defined = self.params.iter().filter(|x| x.is_some()).count();

        assert_eq!(thrusters_defined, self.thrusters_in_use as usize);

        // Make sure that all of the arrays contain `Some()`
        MotorMatrix(self.params)
    }
}

/// Each row corresponds to a line of self.motor_matrix_set(...);
/// Can either be created manually, or the parameters can be dumped into [`Self::new`]
/// for shorter code.
#[derive(Debug)]
pub struct MotorMatrixRowParams {
    pub(crate) x: f32,
    pub(crate) y: f32,
    pub(crate) z: f32,
    pub(crate) pitch: f32,
    pub(crate) roll: f32,
    pub(crate) yaw: f32,
}

impl MotorMatrixRowParams {
    pub fn new(x: f32, y: f32, z: f32, pitch: f32, roll: f32, yaw: f32) -> Self {
        Self {
            x,
            y,
            z,
            pitch,
            roll,
            yaw,
        }
    }
}

impl From<[f32; 6]> for MotorMatrixRowParams {
    fn from(vals: [f32; 6]) -> Self {
        Self {
            x: vals[0],
            y: vals[1],
            z: vals[2],
            pitch: vals[3],
            roll: vals[4],
            yaw: vals[5],
        }
    }
}

pub type PidAxes = [PidAxis; 4];

pub struct PidAxis {
    pub(crate) which: char,
    pub(crate) kp: f32,
    pub(crate) ki: f32,
    pub(crate) kd: f32,
    pub(crate) limit: f32,
    pub(crate) invert: bool,
}

impl From<(char, f32, f32, f32, f32, bool)> for PidAxis {
    fn from(value: (char, f32, f32, f32, f32, bool)) -> Self {
        Self {
            which: value.0,
            kp: value.1,
            ki: value.2,
            kd: value.3,
            limit: value.4,
            invert: value.5,
        }
    }
}
