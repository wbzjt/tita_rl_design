# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from configs.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
from global_config import ROOT_DIR
class TitaConstraintRoughCfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs = 2048

        n_scan = 187
        n_priv_latent =  4 + 1 + 8 + 8 + 8 + 6 + 1 + 2 + 1 - 3
        n_proprio = 33
        history_len = 10
        num_observations = n_proprio + n_scan + history_len*n_proprio + n_priv_latent

    class init_state( LeggedRobotCfg.init_state ):
        # pos = [0.0, 0.0, 0.34] # x,y,z [m]
        pos = [0.0, 0.0, 0.20] # x,y,z [m]
        rot = [0, 0.0, 0.0, 1]  # x, y, z, w [quat]
        lin_vel = [0.0, 0.0, 0.0]  # x, y, z [m/s]
        ang_vel = [0.0, 0.0, 0.0]  # x, y, z [rad/s]       
        default_joint_angles = {

            # 'joint_left_leg_1': 0,
            # 'joint_right_leg_1': 0,

            # 'joint_left_leg_2': 0.8,
            # 'joint_right_leg_2': 0.8,

            # 'joint_left_leg_3': -1.5,
            # 'joint_right_leg_3': -1.5,

            # 'joint_left_leg_4': 0,
            # 'joint_right_leg_4': 0,

            'joint_left_leg_1': -0.0,
            'joint_right_leg_1': 0.00,

            'joint_left_leg_2': -0.8,
            'joint_right_leg_2': -0.8,

            'joint_left_leg_3': 1.5,
            'joint_right_leg_3': 1.5,

            'joint_left_leg_4': 0,
            'joint_right_leg_4': 0,
        }


    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 40}  # [N*m/rad]
        damping = {'joint': 1.0}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        hip_scale_reduction = 0.5

        use_filter = True

    class commands( LeggedRobotCfg.control ):
        curriculum = False
        max_curriculum = 1.
        num_commands = 4  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10.  # time before command are changed[s]
        heading_command = True  # if true: compute ang vel command from heading error
        global_reference = False

        class ranges:
            # lin_vel_x = [0.0, 1.0]  # min max [m/s]
            lin_vel_x = [-1.0, 0.0]  # min max [m/s]

            lin_vel_y = [-0.0, 0.0]  # min max [m/s]
            ang_vel_yaw = [-1, 1]  # min max [rad/s]
            heading = [-3.14, 3.14]

    class asset( LeggedRobotCfg.asset ):

        # file = '{ROOT_DIR}/resources/tita/urdf/tita_description.urdf'
        file = '{ROOT_DIR}/resources/wheel_leg_v6/urdf/wheel_leg_v6.urdf'

        foot_name = "leg_4"
        name = "tita"
        penalize_contacts_on = ["base_link", "left_leg_3", "right_leg_3"]
        terminate_after_contacts_on = ["base_link", "left_leg_3", "right_leg_3"] 
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        # base_height_target = 0.4
        base_height_target = 0.255

        class scales( LeggedRobotCfg.rewards.scales ):
            torques = 0.0
            powers = -2e-5
            termination = 0.0 # off 终止奖励不计入，可能会导致过早终止。
            tracking_lin_vel = 5.0 # 跟踪线速度奖励，促进机器人沿目标速度移动。
            tracking_ang_vel = 2.0 # off 角速度跟踪奖励，促进机器人沿目标角速度旋转。
            lin_vel_z = -0.0
            ang_vel_xy = -0.05
            dof_vel = 0.0
            dof_acc = -2.5e-7
            base_height = -20.0 # 强烈惩罚base高度偏离目标高度
            feet_air_time = 0.0
            collision = -15.0 # 强烈惩罚碰撞，防止机器人与地面或其他物体发生碰撞。
            feet_stumble = 0.0
            action_rate = -0.01
            action_smoothness= 0
            stand_still = -1.0 # 不奖励静止状态，可能导致机器人不动。
            foot_clearance= -0.0
            orientation=-6.0


    class domain_rand( LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.2, 2.75]
        randomize_restitution = True
        restitution_range = [0.0,1.0]
        randomize_base_mass = True
        added_mass_range = [-1., 3.]
        randomize_base_com = True
        added_com_range = [-0.1, 0.1]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1

        randomize_motor = True
        motor_strength_range = [0.8, 1.2]

        randomize_kpkd = True
        kp_range = [0.8, 1.2]
        kd_range = [0.8, 1.2]

        randomize_lag_timesteps = True
        lag_timesteps = 3

        disturbance = False
        disturbance_range = [-30.0, 30.0]
        disturbance_interval = 8
    
    class depth( LeggedRobotCfg.depth):
        use_camera = False
        camera_num_envs = 192
        camera_terrain_num_rows = 10
        camera_terrain_num_cols = 20

        position = [0.27, 0, 0.03]  # front camera
        angle = [-5, 5]  # positive pitch down

        update_interval = 1  # 5 works without retraining, 8 worse

        original = (106, 60)
        resized = (87, 58)
        horizontal_fov = 87
        buffer_len = 2
        
        near_clip = 0
        far_clip = 2
        dis_noise = 0.0
        
        scale = 1
        invert = True
    
    # constraint出处
    class costs:  # costs for the constraint
        class scales: 
            pos_limit = 0.0001 # 位置限制约束
            torque_limit = 0.0001 # 力矩限制约束
            dof_vel_limits = 0.0001 # 速度限制约束
            # vel_smoothness = 0.1 
            # acc_smoothness = 0.0000000001
            #collision = 0.1
            feet_contact_forces = 0.0001 # 足部接触力约束
            stumble = 0.001 # 6个成本约束
        class d_values:
            pos_limit = 1.0
            torque_limit = 1.0
            dof_vel_limits = 0.5
            # vel_smoothness = 0.0
            # acc_smoothness = 200.0
            #collision = 0.0
            feet_contact_forces = 0.5
            stumble = 0.2

 
    
    class cost:
        num_costs = 5
    
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'  # "heightfield" # none, plane, heightfield or trimesh
        measure_heights = True
        include_act_obs_pair_buf = False

class TitaConstraintRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
        learning_rate = 1.e-3
        max_grad_norm = 0.01
        num_learning_epochs = 5
        num_mini_batches = 4 # mini batch size = num_envs*nsteps / nminibatches
        cost_value_loss_coef = 0.1
        cost_viol_loss_coef = 0.1

    class policy( LeggedRobotCfgPPO.policy):
        init_noise_std = 1.0
        continue_from_last_std = True
        scan_encoder_dims = [128, 64, 32]
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        #priv_encoder_dims = [64, 20]
        priv_encoder_dims = []
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
        # only for 'ActorCriticRecurrent':
        rnn_type = 'lstm'
        rnn_hidden_size = 512
        rnn_num_layers = 1

        tanh_encoder_output = False
        num_costs = 5

        teacher_act = True
        imi_flag = True
      
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = 'test_barlowtwins_feetcontact'
        experiment_name = 'tita_constraint'
        policy_class_name = 'ActorCriticBarlowTwins'
        runner_class_name = 'OnConstraintPolicyRunner'
        algorithm_class_name = 'NP3O'
        max_iterations = 100000
        num_steps_per_env = 24
        resume = False
        resume_path = 'tita_example_10000.pt'

 

  
