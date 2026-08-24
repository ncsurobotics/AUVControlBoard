//! Communicate with AUVControlBoard hardware
//!
//! # Example Usage:
//!
//! ```
//! use auv_control_board::{SerialControlBoard, motor_matrix, pid_axes, vehicle::Definition};
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
//!     let board = SerialControlBoard::new("/dev/serial/by-id/usb-STMicroelectronics_Control_Board_v2__Virtual_COM_Port__36313632303251010061003C-if00", VEHICLE_DEFINITION).await;
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
