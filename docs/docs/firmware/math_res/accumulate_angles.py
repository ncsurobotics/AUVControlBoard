
import numpy as np


# ZYX (deg)
class Euler:
    def __init__(self, pitch: np.float32 = 0.0, roll: np.float32 = 0.0, yaw: np.float32 = 0.0):
        # In degrees
        self.pitch: np.float32 = pitch
        self.roll: np.float32 = roll
        self.yaw: np.float32 = yaw
    
    def __repr__(self) -> str:
        return "Euler(pitch = {}, roll = {}, yaw = {})".format(self.pitch, self.roll, self.yaw)
    
    def to_quaternion(self) -> 'Quaternion':
        ## Pitch about X, Roll about Y, Yaw about Z

        roll_rad = np.deg2rad(self.roll)
        pitch_rad = np.deg2rad(self.pitch)
        yaw_rad = np.deg2rad(self.yaw)

        cr = np.cos(roll_rad * 0.5)
        sr = np.sin(roll_rad * 0.5)
        cp = np.cos(pitch_rad * 0.5)
        sp = np.sin(pitch_rad * 0.5)
        cy = np.cos(yaw_rad * 0.5)
        sy = np.sin(yaw_rad * 0.5)

        q = Quaternion()
        q.w = cp * cr * cy + sp * sr * sy
        q.x = sp * cr * cy - cp * sr * sy
        q.y = cp * sr * cy + sp * cr * sy
        q.z = cp * cr * sy - sp * sr * cy

        return q


class Quaternion:
    def __init__(self, w: np.float32 = 0.0, x: np.float32 = 0.0, y: np.float32 = 0.0, z: np.float32 = 0.0):
        self.w: np.float32 = w
        self.x: np.float32 = x
        self.y: np.float32 = y
        self.z: np.float32 = z
    
    def __repr__(self) -> str:
        return "Quaternion(w = {}, x = {}, y = {}, z = {})".format(self.w, self.x, self.y, self.z)
    
    def to_euler(self) -> Euler:
        angles = Euler()

        # Pitch (about x axis)
        t0 = +2.0 * (self.w * self.x + self.y * self.z)
        t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        angles.pitch = np.arctan2(t0, t1)
     
        # Roll (about y axis)
        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        angles.roll = np.arcsin(t2)
     
        # Yaw (about z axis)
        t3 = +2.0 * (self.w * self.z + self.x * self.y)
        t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        angles.yaw = np.arctan2(t3, t4)
     
        # Convert to degrees
        angles.pitch = np.rad2deg(angles.pitch)
        angles.roll = np.rad2deg(angles.roll)
        angles.yaw = np.rad2deg(angles.yaw)

        return angles
    
    def multiply_scalar(self, s: np.float32) -> 'Quaternion':
        return Quaternion(self.w * s, self.x * s, self.y * s, self.z * s)

    def divide_scalar(self, s: np.float32) -> 'Quaternion':
        return Quaternion(self.w / s, self.x / s, self.y / s, self.z / s)

    def multiply(self, second: 'Quaternion') -> 'Quaternion':
        res = Quaternion()
        res.w = self.w * second.w - self.x * second.x - self.y * second.y - self.z * second.z
        res.x = self.w * second.x + self.x * second.w + self.y * second.z - self.z * second.y
        res.y = self.w * second.y - self.x * second.z + self.y * second.w + self.z * second.x
        res.z = self.w * second.z + self.x * second.y - self.y * second.x + self.z * second.w
        return res

    def inverse(self) -> 'Quaternion':
        return self.conjugate().divide_scalar(self.magnitude())
    
    def conjugate(self) -> 'Quaternion':
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def magnitude(self) -> np.float32:
        return np.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
    
    def dot(self, second: 'Quaternion') -> np.float32:
        return self.w * second.w + self.x * second.x + self.y * second.y + self.z * second.z


# To quat then back to euler "simplifies" the angles
# pitch from -180 to 180
# roll from -90 to 90
# yaw from -180 to 180 (note the bno055 uses 0 to 360, but quaternions will be the same)
prev_euler = Euler(
    pitch = np.float32(0.0),
    roll = np.float32(0.0),
    yaw = np.float32(0.0)
).to_quaternion().to_euler()

curr_euler = Euler(
    pitch = np.float32(0.0),
    roll = np.float32(0.0),
    yaw = np.float32(340.0)
).to_quaternion().to_euler()


print("Prev Euler: {}".format(prev_euler))
print("Curr Euler: {}".format(curr_euler))
print()

prev_quat = prev_euler.to_quaternion()
curr_quat = curr_euler.to_quaternion()

print("Prev Quat: {}".format(prev_quat))
print("Curr Quat: {}".format(curr_quat))
print()

# Calculate smallest explication of angle change between the two given quaternions
diff_quat = Quaternion()
if curr_quat.dot(prev_quat) < 0:
    diff_quat = curr_quat.multiply(prev_quat.multiply_scalar(-1).inverse())
else:
    diff_quat = curr_quat.multiply(prev_quat.inverse())
diff_euler = diff_quat.to_euler()

print("Diff Quat: {}".format(diff_quat))
print("Diff Euler: {}".format(diff_euler))

# Note: The values in the diff euler would be added to accumulated pitch, roll, and yaw