//! Communicate with AUVControlBoard hardware
//!
//! # Example Usage:
//!
//! ```
//! // TODO: Macro this
//! // motor_matrix! [
//! //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//! //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//! //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//! //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//! //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//! //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//! //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//! //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//! // ]
//! let motor_matrix = MotorMatrix::builder(8)
//!   .set_row(1, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].into())
//!   .set_row(2, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].into())
//!   .set_row(3, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].into())
//!   .set_row(4, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].into())
//!   .set_row(5, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].into())
//!   .set_row(6, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].into())
//!   .set_row(7, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].into())
//!   .set_row(8, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].into())
//!   .build();
//!
//! let vehicle_def = VehicleDefinition::new(
//!   motor_matrix,
//!   [true, true, false, false, true, false, false, true].into(),
//!   [0.7071, 0.7071, 1.0, 0.4413, 1.0, 0.8139],
//!   [
//!     ('X', 0.8, 0.0, 0.0, 0.6, false).into(),
//!     ('Y', 2.0, 0.0, 0.0, 0.1, false).into(),
//!     ('Z', 4.0, 0.0, 0.0, 1.0, false).into(),
//!     ('D', 1.5, 0.0, 0.0, 1.0, false).into(),
//!   ],
//! )?;
//!
//! let board = ControlBoard::serial(config.control_board_path.as_str(), &vehicle_def).await;
//! ```

mod interface;
mod protocol;

pub use interface::*;

macro_rules! write_stream_mutexed {
    ( $stream_mutex:expr, $string:expr ) => {{
        $stream_mutex
            .lock()
            .await
            .write_all($string.as_bytes())
            .await
            .unwrap()
    }};
}
pub(crate) use write_stream_mutexed;
