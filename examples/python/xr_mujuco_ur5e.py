import mujoco
from mujoco import viewer as mj_viewer
import numpy as np
import os
from threading import Thread
from ik_solver import IKSolver
from utils import second_print

try:
    import rclpy
    from rclpy.node import Node
except Exception as e:
    print("Forget to source ros2 environment?")
    raise e

try:
    from xr_msgs.msg import Custom
except Exception as e:
    print("Forget to source picoxr-ros2 environment?")
    raise e

np.set_printoptions(precision=4, suppress=True)

rest = np.array([0.5, 0.5, 0.5])
goal = np.array([0.49, 0.63, 0.59])

rest_2 = np.array([0.5, -0.5, 0.5])
goal_2 = np.array([0.49, -0.43, 0.59])


class XrSubscriber(Node):
    def __init__(self):
        super().__init__('xr_subscriber')
        self.subscription = self.create_subscription(
            Custom,
            'xr_pose',
            self.xr_callback,
            10)
        self.subscription	# prevent unused variable warning
        self._cnt = 0
        self.origin = None
        self.origin_2 = None

    def xr_callback(self, msg):
        second_print(f"Received XR msgs: {msg.timestamp_ns}")
        global goal, goal_2
        
        if msg.left_controller.status == 3 and msg.left_controller.pose is not None:
            pose = msg.left_controller.pose   # x,y,z,rx,ry,rz,rw
            if self._cnt == 0:
                self.origin = pose

            delta_xr = pose[:3] - self.origin[:3]
            x, y, z = delta_xr
            delta_mj = np.array([-z, -x, y])
            goal = rest + delta_mj

        if msg.right_controller.status == 3 and msg.right_controller.pose is not None:
            pose = msg.right_controller.pose   # x,y,z,rx,ry,rz,rw
            if self._cnt == 0:
                self.origin_2 = pose

            delta_xr = pose[:3] - self.origin_2[:3]
            x, y, z = delta_xr
            delta_mj = np.array([-z, -x, y])
            goal_2 = rest_2 + delta_mj

        # if msg.head.status == 3:
        #     head_pose = msg.head.pose   # x,y,z,rx,ry,rz,rw
        #     if self._cnt == 0:
        #         self.origin = head_pose
        #     delta_xr = head_pose[:3] - self.origin[:3]
        #     x, y, z = delta_xr
        #     delta_mj = np.array([x, -z, y])
        #     goal = rest + delta_mj
        #     if self._cnt % 100 == 0:
        #         print("set goal to ", goal)
        #     self._cnt += 1

        if self._cnt % 100 == 0:
            # print("set goal to ", goal)
            pass
        self._cnt += 1


def run_ros_node(args=None):
    rclpy.init(args=args)
    subscriber = XrSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


def main_mujoco():
    # run_ros_node(); return    # debug-only

    ros_thread = Thread(target=run_ros_node)
    ros_thread.start()

    assets_path = os.path.join(os.path.dirname(__file__), "../assets")
    xml = f"{assets_path}/universal_robots_ur5e/scene.xml"
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)

    mocap_id = model.body("target").mocapid[0]
    mocap_id_2 = model.body("target__2").mocapid[0]

    # Reset state and time
    mujoco.mj_resetData(model, data)
    
    # Reset to home keyframe
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    
    # Init parameters
    jacp = np.zeros((3, model.nv)) # translation jacobian
    jacr = np.zeros((3, model.nv)) # rotational jacobian
    jacp_2 = np.zeros((3, model.nv)) # translation jacobian
    jacr_2 = np.zeros((3, model.nv)) # rotational jacobian
    step_size = 0.5
    tol = 0.01
    damping = 0.01
    
    # Get error.
    end_effector_id = model.body('wrist_3_link').id
    current_pose = data.body(end_effector_id).xpos

    end_effector_id_2 = model.body('wrist_3_link__2').id
    current_pose_2 = data.body(end_effector_id_2).xpos
    
    error = np.subtract(goal, current_pose)
    error_2 = np.subtract(goal_2, current_pose_2)

    viewer = mj_viewer.launch_passive(model, data)
    
    cnt = 0
    while True:
        if np.linalg.norm(error) >= tol:

            data.mocap_pos[mocap_id] = goal
            data.mocap_pos[mocap_id_2] = goal_2
            # print("get goal: ", goal)
            
            # left arm
            mujoco.mj_jac(model, data, jacp, jacr, goal, end_effector_id)
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
            q[:6] += step_size * delta_q[:6]

            # right arm
            mujoco.mj_jac(model, data, jacp_2, jacr_2, goal_2, end_effector_id_2)
            product = jacp_2.T @ jacp_2 + damping * I
            if np.isclose(np.linalg.det(product), 0):
                j_inv = np.linalg.pinv(product) @ jacp_2.T
            else:
                j_inv = np.linalg.inv(product) @ jacp_2.T
            delta_q = j_inv @ error_2
            # Compute next step
            q_2 = data.qpos.copy()
            q_2[6:] += step_size * delta_q[6:]
            
            # Check limits
            for i in range(model.nq):
                q[i] = max(model.jnt_range[i][0],  min(q[i], model.jnt_range[i][1]))
                q_2[i] = max(model.jnt_range[i][0],  min(q_2[i], model.jnt_range[i][1]))
            
            # Set control signal
            # data.ctrl = q
            data.ctrl[:6] = q[:6]
            data.ctrl[6:] = q_2[6:]

            # Step the simulation.
            mujoco.mj_step(model, data)
    
            error = np.subtract(goal, data.body(end_effector_id).xpos)
            error_2 = np.subtract(goal_2, data.body(end_effector_id_2).xpos)

            if cnt % 100 == 0:
                print(f"ik error: {error}, error_2: {error_2}")
            cnt += 1

        viewer.sync()

    viewer.close()
    ros_thread.join()


def main():
    # run_ros_node(); return    # debug-only

    ros_thread = Thread(target=run_ros_node)
    ros_thread.start()

    assets_path = os.path.join(os.path.dirname(__file__), "../assets")
    xml = f"{assets_path}/universal_robots_ur5e/scene_double.xml"
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)

    mocap_id = model.body("target").mocapid[0]
    mocap_id_2 = model.body("target__2").mocapid[0]

    # Reset state and time, set to home status
    mujoco.mj_resetData(model, data)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    
    # Init parameters
    step_size = 0.5
    tol = 0.01
    damping = 0.15
    max_iter = 100000

    left_arm_joints = ["shoulder_pan",
                       "shoulder_lift",
                       "elbow",
                       "wrist_1",
                       "wrist_2",
                       "wrist_3"]
    left_arm_end_effector = "wrist_3_link"
    left_ik = IKSolver(model, left_arm_joints, left_arm_end_effector, "base")
    theta_left = data.qpos[left_ik.joint_ids].copy()

    right_arm_joints = ["shoulder_pan__2",
                       "shoulder_lift__2",
                       "elbow__2",
                       "wrist_1__2",
                       "wrist_2__2",
                       "wrist_3__2"]
    right_arm_end_effector = "wrist_3_link__2"
    right_ik = IKSolver(model, right_arm_joints, right_arm_end_effector, "base_second")
    theta_right = data.qpos[right_ik.joint_ids].copy()

    viewer = mj_viewer.launch_passive(model, data)
    for i in range(max_iter):

        data.mocap_pos[mocap_id] = goal
        data.mocap_pos[mocap_id_2] = goal_2

        left_target = np.eye(4)
        left_target[:3, 3] = goal
        right_target = np.eye(4)
        right_target[:3, 3] = goal_2

        error_left, d_theta = left_ik.step(left_target, theta_left, damping)
        theta_left += step_size * d_theta

        error_right, d_theta = right_ik.step(right_target, theta_right, damping)
        theta_right += step_size * d_theta

        second_print(f"{i} / {max_iter}, error: {np.linalg.norm(error_left)}, {np.linalg.norm(error_right)}")
        
        data.qpos[left_ik.joint_ids] = theta_left
        data.qpos[right_ik.joint_ids] = theta_right

        mujoco.mj_step(model, data)
        viewer.sync()

    viewer.close()
    ros_thread.join()


if __name__ == "__main__":
    main()
    # main_mujoco()
