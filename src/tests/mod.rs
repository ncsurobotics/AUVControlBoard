use super::*;
use crate::{vehicle::Definition, SerialControlBoard};
use serial_test::serial;

use std::time::Duration;
use tokio::time::sleep;

const VEHICLE_DEFINITION: Definition<8> = Definition::new(
    motor_matrix! [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false;
    ],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    pid_axes! [
        X, 0.0, 0.0, 0.0, 0.0, false;
        X, 0.0, 0.0, 0.0, 0.0, false;
        X, 0.0, 0.0, 0.0, 0.0, false;
        X, 0.0, 0.0, 0.0, 0.0, false;
    ],
);

const CB_PATH: &'static str = "/dev/serial/by-id/usb-STMicroelectronics_Control_Board_v2__Virtual_COM_Port__36313632303251010061003C-if00";

#[tokio::test]
#[serial]
async fn connect_serial() {
    let cb = SerialControlBoard::new(CB_PATH, VEHICLE_DEFINITION)
        .await
        .unwrap();
    cb.reset().await.unwrap();
}

#[tokio::test]
#[serial]
async fn motor_test() {
    const MOTOR_TEST_SPEED: f32 = 0.7;
    const MOTOR_TEST_TIME: u64 = 10;
    let cb = SerialControlBoard::new(CB_PATH, VEHICLE_DEFINITION)
        .await
        .unwrap();
    cb.raw_speed_set([MOTOR_TEST_SPEED; 8]).await.unwrap();
    sleep(Duration::from_secs(MOTOR_TEST_TIME)).await;
    cb.raw_speed_set([0.0; 8]).await.unwrap();
}
