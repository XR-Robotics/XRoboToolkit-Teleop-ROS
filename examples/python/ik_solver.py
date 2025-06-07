import mujoco
from mujoco import viewer as mj_viewer
import numpy as np


__all__ = ["IKSolver"]


def parse_urdf_params(model, joint_names):
    """
    从MuJoCo模型中提取机械臂参数
    :param model: MuJoCo模型对象
    :param joint_names: 要控制的关节名称列表
    :return: dict包含关节轴方向、连杆长度等信息
    """
    urdf_params = {
        "joint_axes": [],
        "link_transforms": []  # 每个连杆相对于父连杆的变换
    }
    
    # 遍历关节链提取参数
    for jnt_name in joint_names:
        jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_name)
        if jnt_id == -1:
            raise ValueError(f"Joint {jnt_name} not found in model")
        
        # 获取关节轴方向（在局部坐标系）
        axis = model.jnt_axis[jnt_id].copy()
        urdf_params["joint_axes"].append(axis)
        
        # 获取连杆变换（从URDF的<origin>标签解析）
        body_id = model.jnt_bodyid[jnt_id]
        parent_body_id = model.body_parentid[body_id]
        
        # 提取相对于父连杆的变换
        pos = model.body_pos[body_id]
        quat = model.body_quat[body_id]
        mat = np.zeros((4,4))
        quat_mat = np.zeros((9, 1))
        mujoco.mju_quat2Mat(quat_mat, quat)
        mat[:3, :3] = quat_mat.reshape(3, 3)
        mat[:3, 3] = pos
        mat[3, 3] = 1
        urdf_params["link_transforms"].append(mat)
    
    return urdf_params


class IKSolver:
    def __init__(self, model, joint_names, end_effector_body_name, base_body_name):
        self.model = model
        self.joint_names = joint_names
        self.end_effector_body_name = end_effector_body_name
        self.num_joints = len(joint_names)

        self.end_effector_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, end_effector_body_name
        )
        if self.end_effector_body_id == -1:
            raise ValueError(f"Body {end_effector_body_name} not found in model")
        self.joint_ids, self.joint_axes, self.link_transforms = self._parse_kinematic_chain()
        self.base_transform = self._parse_base_transform(base_body_name)

    def _parse_kinematic_chain(self):
        """
        反向遍历从末端执行器到基座的运动链，提取关节参数
        """
        joint_ids = []
        joint_axes = []
        link_transforms = []
        
        current_body_id = self.end_effector_body_id
        while True:
            # 找到驱动当前 Body 的关节
            jnt_id = self.model.body_jntadr[current_body_id]
            if jnt_id == -1:
                break  # 无更多父关节
                
            jnt_type = self.model.jnt_type[jnt_id]
            if jnt_type != mujoco.mjtJoint.mjJNT_HINGE:
                break  # 仅处理旋转关节

            # 记录关节参数
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
            if joint_name not in self.joint_names:
                break  # 超出控制关节范围

            joint_ids.append(jnt_id)
            joint_axes.append(self.model.jnt_axis[jnt_id].copy())
            
            # 提取连杆变换（从父Body到当前Body）
            body_id = current_body_id
            parent_body_id = self.model.body_parentid[body_id]
            pos = self.model.body_pos[body_id]
            quat = self.model.body_quat[body_id]

            mat = np.eye(4)
            quat_mat = np.zeros((9, 1))
            mujoco.mju_quat2Mat(quat_mat, quat)
            mat[:3, :3] = quat_mat.reshape(3, 3)
            mat[:3, 3] = pos
            link_transforms.append(mat)
            
            current_body_id = parent_body_id
        
        # 反转顺序（基座 -> 末端）
        joint_ids = joint_ids[::-1]
        joint_axes = joint_axes[::-1]
        link_transforms = link_transforms[::-1]

        assert joint_ids, f"No found joint index: {joint_ids}"
        
        return joint_ids, joint_axes, link_transforms
    
    def _parse_base_transform(self, base_body_name):
        """
        Get robot base transform from world coordinate.
        """
        names = []
        if isinstance(base_body_name, str):
            names = [base_body_name]
        elif isinstance(base_body_name, (list, tuple)):
            names = base_body_name
        else:
            raise NotImplementedError
        
        M = np.eye(4)
        for name in names:
            pos = self.model.body(name).pos
            quat = self.model.body(name).quat
            mat = np.eye(4)
            quat_mat = np.zeros((9, 1))
            mujoco.mju_quat2Mat(quat_mat, quat)
            mat[:3, :3] = quat_mat.reshape(3, 3)
            mat[:3, 3] = pos
            M = M @ mat
        return M

    def forward_kinematics(self, theta, verbose=False):
        """
        基于URDF参数的正运动学
        :param theta: 关节角度数组（弧度）
        :return: 末端4x4齐次变换矩阵（相对于基座）
        """
        T = self.base_transform
        for i in range(self.num_joints):
            # 根据URDF参数构建每个关节的变换矩阵
            joint_axis = self.joint_axes[i]
            # 旋转关节的变换
            R = self._axis_angle_to_rot(joint_axis, theta[i])
            # 平移连杆 x 旋转关节的变换
            T = T @ self.link_transforms[i] @ R
            if verbose:
                print(i, T[:3, 3])
        return T

    def _axis_angle_to_rot(self, axis, angle):
        """将旋转轴和角度转换为旋转矩阵"""
        """使用MuJoCo内置函数优化旋转矩阵生成"""
        quat = np.zeros((4, 1), dtype=np.float64)
        if axis.shape == (3, ):
            axis = axis[:, None]
        assert axis.shape == (3, 1)
        mujoco.mju_axisAngle2Quat(quat, axis, angle)
        R = np.eye(4)
        R_rot = np.eye(3).flatten()
        mujoco.mju_quat2Mat(R_rot, quat)
        R[:3, :3] = R_rot.reshape(3, 3)
        return R

        # axis = axis / np.linalg.norm(axis)
        # c, s = np.cos(angle), np.sin(angle)
        # x, y, z = axis
        # return np.array([
        #     [c + (1 - c)*x**2, (1 - c)*x*y - s*z, (1 - c)*x*z + s*y, 0],
        #     [(1 - c)*y*x + s*z, c + (1 - c)*y**2, (1 - c)*y*z - s*x, 0],
        #     [(1 - c)*z*x - s*y, (1 - c)*z*y + s*x, c + (1 - c)*z**2, 0],
        #     [0, 0, 0, 1]
        # ])
    
    def compute_jacobian(self, theta):
        """
        计算雅可比矩阵（数值法）
        :param theta: 当前关节角度
        :return: 6xN雅可比矩阵（位置+旋转）
        """
        #  jac_pos = np.zeros((3, len(self.joint_ids)))
        #  jac_rot = np.zeros((3, len(self.joint_ids)))
        #  mujoco.mj_jac(self.model, self.data, jac_pos, jac_rot, self.end_effector_body_id)
        #  return np.vstack([jac_pos, jac_rot])

        # """
        # 使用 MuJoCo 内置函数计算雅可比矩阵（更高效）
        # """
        # # 设置当前关节角度
        # self.model.qpos[self.joint_ids] = theta
        # mujoco.mj_forward(self.model, self.data)
        # 
        # # 计算雅可比矩阵（位置和旋转）
        # jac_pos = np.zeros((3, len(self.joint_ids)))
        # jac_rot = np.zeros((3, len(self.joint_ids)))
        # mujoco.mj_jac(
        #     self.model, self.data, 
        #     jac_pos, jac_rot, 
        #     self.end_effector_body_id
        # )
        # return np.vstack([jac_pos, jac_rot])

        J = np.zeros((6, self.num_joints))
        epsilon = 1e-6
        T0 = self.forward_kinematics(theta)
        pos0 = T0[:3, 3]
        rot0 = T0[:3, :3]
        
        for i in range(self.num_joints):
            theta_perturbed = theta.copy()
            theta_perturbed[i] += epsilon
            T_perturbed = self.forward_kinematics(theta_perturbed)
            pos_perturbed = T_perturbed[:3, 3]
            rot_perturbed = T_perturbed[:3, :3]
            
            # 位置导数
            J[:3, i] = (pos_perturbed - pos0) / epsilon
            # 旋转导数（轴角变化）
            delta_rot = rot_perturbed @ rot0.T
            angle_axis = self._rot_to_axis_angle(delta_rot)
            J[3:, i] = angle_axis / epsilon
        return J
    
    def _rot_to_axis_angle(self, R):
        """旋转矩阵转轴角"""
        arg = (np.trace(R) - 1) / 2
        arg_clipped = np.clip(arg, -1.0, 1.0) 
        theta = np.arccos(arg_clipped)

        if np.isclose(theta, 0.0):  # 处理 theta ≈ 0 的情况 [[10]]
            return np.zeros(3)
        axis = 1/(2*np.sin(theta)) * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
        return axis * theta
   
    def step(self, target, theta, damping=0.01):
        """Step in IK-loop"""
        T = self.forward_kinematics(theta)
        pos_error = target[:3, 3] - T[:3, 3]
        rot_error = self._rot_to_axis_angle(target[:3, :3] @ T[:3, :3].T)
        error = np.concatenate([pos_error, rot_error])

        J = self.compute_jacobian(theta)

        n = J.shape[1]
        I = np.identity(n)

        # d_theta = J.T @ np.linalg.pinv(J @ J.T + damping * I) @ error  # 伪逆 + 阻尼

        product = J.T @ J + damping * I
        if np.isclose(np.linalg.det(product), 0):
            J_inv = np.linalg.pinv(product) @ J.T
        else:
            J_inv = np.linalg.inv(product) @ J.T
        d_theta = J_inv @ error

        return error, d_theta

    def inverse_kinematics(self, target_pose, initial_theta, max_iter=100, tol=1e-4, step=0.1, vis_kwargs={}):
        """
        逆运动求解
        :param target_pose: 目标位姿（4x4齐次矩阵）
        :param initial_theta: 初始关节角度猜测
        :param max_iter: 最大迭代次数
        :param tol: 收敛阈值
        :return: 关节角度数组或None（未收敛）
        """
        if vis_kwargs:
            model = vis_kwargs["model"]
            data = vis_kwargs["data"]
            viewer = mj_viewer.launch_passive(model, data)
            max_iter = 100000000
        
        theta = initial_theta.copy()
        for i in range(max_iter):
            T = self.forward_kinematics(theta)
            pos_error = target_pose[:3, 3] - T[:3, 3]
            rot_error = self._rot_to_axis_angle(target_pose[:3, :3] @ T[:3, :3].T)
            error = np.concatenate([pos_error, rot_error])

            if np.linalg.norm(error) < tol and not vis_kwargs:
                return theta
            
            J = self.compute_jacobian(theta)
            d_theta = J.T @ np.linalg.pinv(J @ J.T + 1e-6*np.eye(6)) @ error  # 伪逆 + 阻尼

            theta += step * d_theta  # 步长缩放
            print(f"{i} / {max_iter}, error: {np.linalg.norm(error)}")
            
            if vis_kwargs:
                data.qpos[self.joint_ids] = theta
                mujoco.mj_step(model, data)
                viewer.sync()

        if vis_kwargs:
            viewer.close()
        return None  # 未收敛
    
    @staticmethod
    def print(model):
        print("joint names:")
        for jnt_id in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
            print(" ", joint_name)

        print("body names: ")
        for body_id in range(model.nbody):
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            print(" ", body_name)
