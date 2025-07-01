import mujoco
from mujoco import viewer as mj_viewer
import numpy as np
import os
from ik_solver import IKSolver
from utils import second_print, quat2rpy

np.set_printoptions(precision=4, suppress=True)
# torch.set_printoptions(precision=4, sci_mode=False)

# goal = [0.49, 0.13, 0.59]
goal = [0.32, -0.13, 0.32]


def main():
    assets_path = os.path.join(os.path.dirname(__file__), "assets")
    xml = f"{assets_path}/universal_robots_ur5e/scene.xml"
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)

    FRAMERATE = 60 #(Hz)
    frames = []

    # Reset state and time
    mujoco.mj_resetData(model, data)
    
    # Reset to home keyframe
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)

    viewer = mj_viewer.launch_passive(model, data)
    
    # Init parameters
    jacp = np.zeros((3, model.nv)) # translation jacobian
    jacr = np.zeros((3, model.nv)) # rotational jacobian
    step_size = 0.5
    tol = 0.01
    damping = 0.15
    
    # Get error.
    end_effector_id = model.body('wrist_3_link').id
    current_pose = data.body(end_effector_id).xpos
    
    error = np.subtract(goal, current_pose)

    mocap_id = model.body("target").mocapid[0]
    data.mocap_pos[mocap_id] = goal

    body_names = ["shoulder_link",
                  "upper_arm_link",
                  "forearm_link",
                  "wrist_1_link",
                  "wrist_2_link",
                  "wrist_3_link"
                  ]
    for name in body_names:
        joint_pose = data.body(model.body(name).id).xpos
        print(name, joint_pose)
    
    while True:
        if np.linalg.norm(error) >= tol:

            mujoco.mj_jac(model, data, jacp, jacr, goal, end_effector_id)

            # Calculate delta of joint q
            n = jacp.shape[1]
            I = np.identity(n)
            product = jacp.T @ jacp + damping * I
    
            if np.isclose(np.linalg.det(product), 0):
                j_inv = np.linalg.pinv(product) @ jacp.T
            else:
                j_inv = np.linalg.inv(product) @ jacp.T
    
            delta_q = j_inv @ error
    
            # Compute next step
            q = data.qpos.copy()
            q += step_size * delta_q
            
            # Check limits
            for i in range(model.nq):
                q[i] = max(model.jnt_range[i][0], 
                           min(q[i], model.jnt_range[i][1]))
            
            # Set control signal
            data.ctrl = q
            # Step the simulation.
            mujoco.mj_step(model, data)
    
            error = np.subtract(goal, data.body(end_effector_id).xpos)
            second_print(error)

        viewer.sync()
    viewer.close()


def main_numpy():
    assets_path = os.path.join(os.path.dirname(__file__), "assets")
    xml = f"{assets_path}/universal_robots_ur5e/scene.xml"
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)

    mujoco.mj_resetData(model, data)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)

    viewer = mj_viewer.launch_passive(model, data)
    
    left_arm_joints = ["shoulder_pan",
                       "shoulder_lift",
                       "elbow",
                       "wrist_1",
                       "wrist_2",
                       "wrist_3"]
    left_arm_end_effector = "wrist_3_link"
    
    # 初始化IK求解器
    # IKSolver.print(model)
    left_ik = IKSolver(model, left_arm_joints, left_arm_end_effector, "base")

    # Init parameters
    step_size = 0.05
    tol = 0.01
    damping = 0.01

    mocap_id = model.body("target").mocapid[0]
    data.mocap_pos[mocap_id] = goal
    
    left_target = np.eye(4)
    left_target[:3, 3] = goal

    theta_left = data.qpos[left_ik.joint_ids].copy()
    error = None
    while True:
        if error is None or np.linalg.norm(error) >= tol:
            error, d_theta = left_ik.step(left_target, theta_left, damping)
            theta_left += step_size * d_theta
            second_print(f"error: {error}", interval=0.1)
            
            data.qpos[left_ik.joint_ids] = theta_left
            mujoco.mj_step(model, data)

        # if data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > 0:
        #     data.mocap_pos[mocap_id] = goal
        viewer.sync()

    viewer.close()


if __name__ == "__main__":
    # main()
    main_numpy()
