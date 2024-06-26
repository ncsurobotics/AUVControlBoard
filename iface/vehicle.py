################################################################################
# Copyright 2022-2023 Marcus Behel
#
# This file is part of AUVControlBoard.
# 
# AUVControlBoard is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# AUVControlBoard is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with AUVControlBoard.  If not, see <https://www.gnu.org/licenses/>.
################################################################################
# Vehicle definitions (used by launch.py)
# IF ADDING VEHICLE CLASSES HERE, ADD TO THE "vehicles" DICT IN launch.py
################################################################################
# Author: Marcus Behel
# Date: May 12, 2023
# Version: 1.0.3
################################################################################


from collections import OrderedDict
from control_board import ControlBoard
from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any
import time

# Collection of all vehicles
all_vehicles = OrderedDict()

default_vehicle = ""

# Register a vehicle after making it's classes
def register_vehicle(name: str, normal: 'Vehicle', sim: 'Vehicle'):
    global default_vehicle
    if len(all_vehicles) == 0:
        default_vehicle = name
    all_vehicles[name] = (normal, sim)

# Set the default vehicle
def set_default_vehicle(name: str):
    global default_vehicle
    default_vehicle = name


################################################################################
# Vehicle base class
################################################################################

class Vehicle(ABC):
    def configure(self, cb: ControlBoard) -> Tuple[ControlBoard.AckError, str]:
        ack, imu, depth = cb.get_sensor_status()
        if ack != ControlBoard.AckError.NONE:
            return ack, "get_sensor_status"

        ack = cb.set_motor_matrix(self.motor_matrix)
        if ack != ControlBoard.AckError.NONE:
            return ack, "set_motor_matrix"

        ack = cb.set_tpwm(*self.thruster_pwm_config)
        if ack != ControlBoard.AckError.NONE:
            return ack, "set_tpwm"

        ack = cb.set_tinv(self.tinv)
        if ack != ControlBoard.AckError.NONE:
            return ack, "set_tinv"

        ack = cb.set_reldof(*self.reldof)
        if ack != ControlBoard.AckError.NONE:
            return ack, "set_reldof"
        
        imu, imu_cfg = self.imu_config
        if imu == ControlBoard.IMUSensors.BNO055:
            ack = cb.set_bno055_axis(imu_cfg)
            if ack != ControlBoard.AckError.NONE:
                return ack, "set_bno055_axis"
        
        ack = cb.tune_pid_xrot(*self.xrot_pid_tuning)
        if ack != ControlBoard.AckError.NONE:
            return ack, "tune_pid_xrot"

        ack = cb.tune_pid_yrot(*self.yrot_pid_tuning)
        if ack != ControlBoard.AckError.NONE:
            return ack, "tune_pid_yrot"

        ack = cb.tune_pid_zrot(*self.zrot_pid_tuning)
        if ack != ControlBoard.AckError.NONE:
            return ack, "tune_pid_zrot"

        ack = cb.tune_pid_depth(*self.depth_pid_tuning)
        if ack != ControlBoard.AckError.NONE:
            return ack, "tune_pid_depth"

        return ControlBoard.AckError.NONE, ""

    @property
    @abstractmethod
    def motor_matrix(self) -> ControlBoard.MotorMatrix:
        pass

    @property
    @abstractmethod
    def thruster_pwm_config(self) -> Tuple[int, int, int]:
        pass

    @property
    @abstractmethod
    def tinv(self) -> List[bool]:
        pass

    @property
    @abstractmethod
    def reldof(self) -> List[float]:
        pass

    @property
    @abstractmethod
    def imu_config(self) -> Tuple[ControlBoard.IMUSensors, Any]:
        pass

    @property
    @abstractmethod
    def xrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        pass

    @property
    @abstractmethod
    def yrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        pass
    
    @property
    @abstractmethod
    def zrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        pass

    @property
    @abstractmethod
    def depth_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        pass

    @property
    def simulator_vehicle_id(self) -> str:
        return ""

################################################################################


################################################################################
# AquaPack Robotics's SeaWolf VIII
################################################################################

class SW8(Vehicle):
    @property
    def motor_matrix(self) -> ControlBoard.MotorMatrix:
        mat = ControlBoard.MotorMatrix()
        #        MotorNum    x      y      z    pitch   roll     yaw
        mat.set_row(1,    [ -1,    +1,     0,     0,      0,     -1   ])
        mat.set_row(2,    [ +1,    +1,     0,     0,      0,     +1   ])
        mat.set_row(3,    [ -1,    -1,     0,     0,      0,     +1   ])
        mat.set_row(4,    [ +1,    -1,     0,     0,      0,     -1   ])
        mat.set_row(5,    [  0,     0,    -1,    +1,     -1,      0   ])
        mat.set_row(6,    [  0,     0,    -1,    +1,     +1,      0   ])
        mat.set_row(7,    [  0,     0,    -1,    -1,     -1,      0   ])
        mat.set_row(8,    [  0,     0,    -1,    -1,     +1,      0   ])
        return mat

    @property
    def thruster_pwm_config(self) -> Tuple[int, int, int]:
        # Settings for BlueRobotics Basic ESCs
        return 2500, 1500, 400

    @property
    def tinv(self) -> List[bool]:
        tinv = [
            True,           # Thruster 1
            True,           # Thruster 2
            False,          # Thruster 3
            False,          # Thruster 4
            True,           # Thruster 5
            False,          # Thruster 6
            False,          # Thruster 7
            True            # Thruster 8
        ]
        return tinv

    @property
    def reldof(self) -> List[float]:
        reldof = [
            0.7071,         # x
            0.7071,         # y
            1.0,            # z
            0.4413,         # xrot
            1.0,            # yrot
            0.8139          # zrot
        ]
        return reldof

    @property
    def imu_config(self) -> Tuple[ControlBoard.IMUSensors, Any]:
        return ControlBoard.IMUSensors.BNO055, ControlBoard.BNO055Axis.P6

    @property
    def xrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        #       kP      kI      kD      lim     invert
        return  0.8,    0.0,    0.0,    0.6,    False

    @property
    def yrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        #       kP      kI      kD      lim     invert
        return  0.15,   0.0,    0.0,    0.1,    False
    
    @property
    def zrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        #       kP      kI      kD      lim     invert
        return  0.8,    0.0,    0.0,    0.8,    False

    @property
    def depth_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        #       kP      kI      kD      lim     invert
        return  1.5,    0.0,    0.0,    1.0,    False
        # return 0,       0,      0,      0,      False

    @property
    def simulator_vehicle_id(self) -> str:
        return "SW8"


class SW8Sim(SW8):
    @property
    def xrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        #       kP      kI      kD      lim     invert
        return  0.8,    0.0,    0.0,    0.6,    False

    @property
    def yrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        #       kP      kI      kD      lim     invert
        return  0.15,   0.0,    0.0,    0.1,    False
    
    @property
    def zrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        #       kP      kI      kD      lim     invert
        return  0.5,    0.0,    0.0,    0.5,    False

    @property
    def depth_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        #       kP      kI      kD      lim     invert
        return  1.5,    0.0,    0.0,    1.0,    False


register_vehicle("sw8", SW8(), SW8Sim())

################################################################################
