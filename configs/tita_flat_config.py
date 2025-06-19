from configs import TitaRoughCfg, TitaRoughCfgPPO


class TitaFlatCfg(TitaRoughCfg):
    class env(TitaRoughCfg.env):
        num_privileged_obs = 27 + 2 + 2
        num_propriceptive_obs = 27 + 2 + 2 # plus 2 dof vel(wheels) and 2 actions(wheels)
        num_actions = 8

    class terrain(TitaRoughCfg.terrain):
        mesh_type = "plane"
        measure_heights_critic = False

    class commands(TitaRoughCfg.commands):
        num_commands = 3
        heading_command = False
        resampling_time = 5.

        class ranges(TitaRoughCfg.commands.ranges):
            lin_vel_x = [-1.0, 1.0]  # min max [m/s]
            heading = [-1.0, 1.0]
            lin_vel_y = [0, 0]
            ang_vel_yaw = [-3.14, 3.14]
    
    class init_state(TitaRoughCfg.init_state):
        # pos = [0.0, 0.0, 0.8] # origin
        pos = [0.0, 0.0, 0.34]  # x,y,z [m]
        rot = [0.0, 0.0, 0.0, 1.0]  # x,y,z,w [quat]
        lin_vel = [0.0, 0.0, 0.0]  # x,y,z [m/s]
        ang_vel = [0.0, 0.0, 0.0]  # x,y,z [rad/s]
        default_joint_angles = {  # target angles when action = 0.0
            "left_roll_joint": 0.0,
            "right_roll_joint": 0.0,
            "left_pitch_joint": 0.8,
            "right_pitch_joint": 0.8,
            "left_knee_joint": -1.5,
            "right_knee_joint": -1.5,
            "left_wheel_joint": 0.0,
            "right_wheel_joint": 0.0,
        }   
    
    class control(TitaRoughCfg.control):
        control_type = "P_AND_V" # P: position, V: velocity, T: torques. 
                                 # P_AND_V: some joints use position control 
                                 # and others use vecocity control.
        # PD Drive parameters:
        stiffness = {
            "left_roll_joint": 30,
            "left_pitch_joint": 30,
            "left_knee_joint": 30,
            "right_roll_joint": 30,
            "right_pitch_joint": 30,
            "right_knee_joint": 30,
            "left_wheel_joint": 0.0,
            "right_wheel_joint": 0.0,
        }  # [N*m/rad]
        damping = {
            "left_roll_joint": 0.5,
            "left_pitch_joint": 0.5,
            "left_knee_joint": 0.5,
            "right_roll_joint": 0.5,
            "right_pitch_joint": 0.5,
            "right_knee_joint": 0.5,
            "left_wheel_joint": 0.5,
            "right_wheel_joint": 0.5,
        }  # [N*m*s/rad]
        # action scale: target angle = actionscale * action + defaultangle
        # action_scale_pos is the action scale of joints that use position control
        # action_scale_vel is the action scale of joints that use velocity control
        action_scale_pos = 0.25
        action_scale_vel = 8
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4       

    class asset(TitaRoughCfg.asset):
        foot_name = "_wheel_link"
        foot_radius = 0.095
        penalize_contacts_on = ["base_link", "left_knee_link", "right_knee_link"]
        terminate_after_contacts_on = ["base_link", "left_knee_link", "right_knee_link"]     
        replace_cylinder_with_capsule = False       
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter
    
    class domain_rand(TitaRoughCfg.domain_rand):
        friction_range = [0.2, 1.6]
        added_mass_range = [-0.5, 2]
        
    class rewards(TitaRoughCfg.rewards):
        class scales(TitaRoughCfg.rewards.scales):
            # base class
            lin_vel_z = 0.0 # off 不奖励z方向移动
            ang_vel_xy = 0.0 # off 忽略XY面角速度
            orientation = -5.0 # 很重要，不加的话会导致存活时间下降 强烈惩罚姿态不稳定以对抗跌倒或旋转失衡
            base_height = -20.0 # 强烈惩罚base高度偏离目标高度
            torques = -2.5e-05  #惩罚使用较大电机扭矩，促进能量效率。
            dof_vel = 0.0 # off # 关节速度不计入奖励。
            dof_acc = -2.5e-07 # 轻微惩罚关节加速度变化，有利于平滑控制。
            action_rate = -0.01 # 轻微惩罚动作变化，促进平稳运动。
            collision = -10.0 # 强烈惩罚碰撞，防止机器人与地面或其他物体发生碰撞。
            termination = 0.0 # off 终止奖励不计入，可能会导致过早终止。
            dof_pos_limits = -2.0 # 轻微惩罚关节位置超出限制，促进关节在安全范围内运动。
            torque_limits = 0.0 # off不惩罚扭矩限制，可能会导致过度使用电机。
            tracking_lin_vel = 10.0 # 跟踪线速度奖励，促进机器人沿目标速度移动。
            tracking_ang_vel = 5.0 # off 角速度跟踪奖励，促进机器人沿目标角速度旋转。
            feet_air_time = 0.0 # off 脚离地时间不奖励。
            no_fly = 1.0 # off 	鼓励双轮总有一只触地（不“飞起”）。
            unbalance_feet_air_time = 0.0 # off
            unbalance_feet_height = 0.0 # off
            feet_stumble = 0.0 # off	暂不计入脚步不平衡奖励。
            stand_still = -1.0 # 不奖励静止状态，可能导致机器人不动。
            feet_contact_forces = 0.0 # off 接触力未计入奖励。
            feet_distance = -100 # -100 	脚之间的水平距离不合适时惩罚很大。
            survival = 0.1 # 生存奖励，鼓励机器人尽可能长时间存活。
            # new added
            wheel_adjustment = 1.0 # 1.0 off 轮子调整奖励，鼓励轮子在地面上保持适当位置。
            inclination = 0.0 # off 倾斜奖励，可能会导致机器人在倾斜地面上不稳定。
            leg_symmetry = 10.0 # 10.0 鼓励腿部对称性，促进平衡和稳定。

        base_height_target = 0.4
        soft_dof_pos_limit = 0.95  # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 1.0
        min_feet_distance = 0.57
        max_feet_distance = 0.60
        tracking_sigma = 0.1 # tracking reward = exp(-error^2/sigma)
        nominal_foot_position_tracking_sigma = 0.005
        nominal_foot_position_tracking_sigma_wrt_v = 0.5
        # base_height_target = 0.65 + 0.1664
        leg_symmetry_tracking_sigma = 0.001
        foot_x_position_sigma = 0.001

class TitaFlatCfgPPO(TitaRoughCfgPPO):
    class policy(TitaRoughCfgPPO.policy):
        actor_hidden_dims = [128, 64, 32]
        critic_hidden_dims = [128, 64, 32]

    class runner(TitaRoughCfgPPO.runner):
        experiment_name = 'tita_flat'
        max_iterations = 80000
