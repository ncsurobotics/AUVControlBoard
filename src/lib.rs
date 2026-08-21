//! Communicate with AUVControlBoard hardware
//!
//! # Example Usage:
//!
//! ```
//! use auv_control_board::{ControlBoard, motor_matrix, pid_axes, vehicle::Definition};
//! #[tokio::main]
//! async fn main() {
//!     const VEHICLE_DEFINITION: Definition<8> = Definition::new(
//!         motor_matrix! [
//!             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
//!             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
//!             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
//!             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
//!             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
//!             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
//!             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
//!             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
//!         ],
//!         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
//!         pid_axes! [
//!             X, 0.0, 0.0, 0.0, 0.0, false;
//!             X, 0.0, 0.0, 0.0, 0.0, false;
//!             X, 0.0, 0.0, 0.0, 0.0, false;
//!             X, 0.0, 0.0, 0.0, 0.0, false;
//!         ],
//!     );
//!
//!     let board = ControlBoard::serial("/dev/ttyACM0", &VEHICLE_DEFINITION).await;
//! }
//! ```

mod interface;
mod protocol;
#[cfg(test)]
mod tests;

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
