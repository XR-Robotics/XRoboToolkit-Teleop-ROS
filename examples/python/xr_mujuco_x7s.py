import mujoco
from mujoco import viewer as mj_viewer
import numpy as np
import os
from threading import Thread
from ik_solver import IKSolver
from utils import second_print
from transforms3d.quaternions import quat2mat, qmult, qinverse, mat2quat
from transforms3d.euler import quat2euler, euler2mat

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

rest = np.array([
    [1, 0, 0, 0.37],
    [0, 1, 0, 0.15],
    [0, 0, 1, 0.26],
    [0, 0, 0, 1.0],
    ])
goal = np.array([
    [1, 0, 0, 0.57],
    [0, 1, 0, 0.15],
    [0, 0, 1, 0.60],
    [0, 0, 0, 1.0],
    ])

rest_2 = np.array([0.37, -0.15, 0.26])
goal_2 = np.array([0.57, -0.15, 0.60])


def get_relative_rotation(A, B):
    return qmult(B, qinverse(A))


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
        
        # (x, y, z) => (-z, -x, y)
        # self.T_xr_mj = np.zeros((4, 4))
        # self.T_xr_mj[0, 0] = 1
        # self.T_xr_mj[1, 1] = 1
        # self.T_xr_mj[2, 2] = 1
        # self.T_xr_mj[3, 3] = 1

        self.T_xr_mj = np.array([
            # [0,  0, -1],
            # [-1, 0,  0],
            # [0,  1,  0],

            [0,  0, -1],
            [-1, 0,  0],
            [0,  1,  0],

            ])

    def xr_callback(self, msg):
        second_print(f"Received XR msgs: {msg.timestamp_ns}")
        global goal, goal_2
        
        if msg.left_controller.status == 3 and msg.left_controller.pose is not None:
            pose = msg.left_controller.pose   # x,y,z,rx,ry,rz,rw
            if self._cnt == 0:
                self.origin = pose
                self.origin_wxyz = [pose[-1], pose[3], pose[4], pose[5]]

            rot_wxyz = [pose[-1], pose[3], pose[4], pose[5]]
            rot_quat = get_relative_rotation(self.origin_wxyz, rot_wxyz)
            rot_rpy = quat2euler(rot_quat, 'rzyx')

            # (x, y, z) => (-z, -x, y)
            delta_mj = np.eye(4)
            rot_rpy_mj =  [rot_rpy[1], rot_rpy[2], -rot_rpy[0]]
            delta_mj[:3, :3] = euler2mat(*rot_rpy_mj, 'rzyx')
            x, y, z = pose[:3] - self.origin[:3]
            delta_mj[:3, 3] = np.array([-z, -x, y])
            goal = rest @ delta_mj

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


def main():
    # run_ros_node(); return    # debug-only

    ros_thread = Thread(target=run_ros_node)
    ros_thread.start()

    assets_path = os.path.join(os.path.dirname(__file__), "../assets")
    xml = f"{assets_path}/arx_x7s/scene.xml"
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    
    IKSolver.print(model)

    mocap_id = model.body("target").mocapid[0]
    mocap_id_2 = model.body("target__2").mocapid[0]

    # Reset state and time, set to home status
    mujoco.mj_resetData(model, data)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    
    # Init parameters
    step_size = 0.5
    tol = 0.01
    damping = 0.40
    max_iter = 100000

    left_arm_joints = [f"joint{i}" for i in range(5, 12)]   # 7dof
    left_arm_end_effector = "link11"
    left_ik = IKSolver(model, left_arm_joints, left_arm_end_effector, ["root", "link1", "link2"])
    theta_left = data.qpos[left_ik.joint_ids].copy()
    # ee_left_pos = right_ik.forward_kinematics(theta_right)

    right_arm_joints = [f"joint{i}" for i in range(14, 21)]  # 7 dof
    right_arm_end_effector = "link20"
    right_ik = IKSolver(model, right_arm_joints, right_arm_end_effector, ["root", "link1", "link2"])
    theta_right = data.qpos[right_ik.joint_ids].copy()
    # ee_right_pos = right_ik.forward_kinematics(theta_right)

    viewer = mj_viewer.launch_passive(model, data)

    for i in range(max_iter):
        
        data.mocap_pos[mocap_id] = goal[:3, 3]
        data.mocap_quat[mocap_id] = mat2quat(goal[:3, :3])
        data.mocap_pos[mocap_id_2] = goal_2

        left_target = goal 
        right_target = np.eye(4)
        right_target[:3, 3] = goal_2
        
        error_left, d_theta = left_ik.step(left_target, theta_left, damping)
        theta_left += step_size * d_theta

        error_right, d_theta = right_ik.step(right_target, theta_right, damping)
        theta_right += step_size * d_theta

        second_print(f"{i} / {max_iter}, error: {np.linalg.norm(error_left)}, {np.linalg.norm(error_right)}")

        for i, j in enumerate(left_ik.joint_ids):
            theta_left[i] = max(model.jnt_range[j][0], min(theta_left[i], model.jnt_range[j][1]))
        for i, j in enumerate(right_ik.joint_ids):
            theta_right[i] = max(model.jnt_range[j][0], min(theta_right[i], model.jnt_range[j][1]))

        data.qpos[left_ik.joint_ids] = theta_left
        data.qpos[right_ik.joint_ids] = theta_right
        
        # fix other joint
        data.qpos[0] = 0
        data.qpos[1] = 0
        data.qpos[2] = 0
        data.qpos[3] = 0
        data.qpos[11] = 0   # left gripper
        data.qpos[12] = 0
        data.qpos[20] = 0   # right gripper
        data.qpos[21] = 0
        
        mujoco.mj_step(model, data)

        if data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > 0:
            data.mocap_pos[mocap_id] = goal[:3, 3]
            data.mocap_pos[mocap_id_2] = goal_2

        viewer.sync()

    viewer.close()
    ros_thread.join()


if __name__ == "__main__":
    main()
