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
# Move the vehicle using RAW mode.
################################################################################
# Author: Marcus Behel
# Date: May 10, 2023
# Version: 1.0.0
################################################################################

from control_board import ControlBoard, Simulator
import time
import threading


def run(cb: ControlBoard, s: Simulator) -> int:
    print("Setting raw mode speeds...", end="")
    # Thruster:     1    2    3    4    5    6    7    8
    if cb.set_raw([0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]) == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1
    
    t_stop = False
    def do_feed():
        while not t_stop:
            cb.feed_motor_watchdog()
            time.sleep(0.25)
    t = threading.Thread(daemon=True, target=do_feed)
    t.start()
    print("Press enter to stop...", end="")
    input("")
    t_stop = True
    t.join()

    print("Stopping...", end="")
    if cb.set_local(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1
    return 0

