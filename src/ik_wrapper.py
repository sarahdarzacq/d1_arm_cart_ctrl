import pybullet as p
import pybullet_data
import math

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

d1_arm  = p.loadURDF("urdf/d1_550_description.urdf")
end_effector_index = 6

target_pos = [1, 1, 1]

joint_angles_rad = p.calculateInverseKinematics(
    d1_arm,
    end_effector_index,
    target_pos,
    maxNumIterations=100, 
    residualThreshold=1e-5
)

joint_angles_deg = [math.degrees(angle) for angle in joint_angles_rad]

print(f"Joint angles: {joint_angles_deg[:7]}")