/*============================= RL ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/FSMState_RL.h"
#include "common/timeMarker.h"
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

FSMState_RL::FSMState_RL(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::RL, "rl"),
  input_0(new float[33]),
  input_1(new float[330]),
  output(new float[8]),
  output_last(new float[8]),
  input_1_temp(new float[297])
{
  std::cout << "[FSMState_RL] 构造函数开始" << std::endl;
  cuda_test_ = std::make_shared<CudaTest>("/home/lew/tita_rl_sim2sim2real_test/engine/policy.engine");
  std::cout << "[FSMState_RL] cuda_test_构造完成, cuda init: " << cuda_test_->get_cuda_init() << std::endl;
}

void FSMState_RL::enter()
{
  std::cout << "[FSMState_RL] enter() 开始" << std::endl;
  _data->state_command->firstRun = true;

  for (int i = 0; i < 2; i++)
  {
    wheel_init_pos_abs_[i] = _data->low_state->q[4*i+3];
    desired_pos[4 * i + 3] = 0;
    desired_pos[4 * i] = _data->low_state->q[4*i];
    desired_pos[4 * i + 1] = _data->low_state->q[4*i+1];
    desired_pos[4 * i + 2] = _data->low_state->q[4*i+2];
  }
  for(int i = 0; i < 4; i++)
  {
    obs_.dof_pos[i + 4] = _data->low_state->q[i];
    obs_.dof_vel[i + 4] = _data->low_state->dq[i];
    obs_.dof_pos[i] = _data->low_state->q[i + 4];
    obs_.dof_vel[i] = _data->low_state->dq[i + 4];
  }

  params_.action_scale = 0.5;
  params_.num_of_dofs = 8;
  params_.lin_vel_scale = 2.0;
  params_.ang_vel_scale = 0.25;
  params_.dof_pos_scale = 1.0;
  params_.dof_vel_scale = 0.05;

  params_.commands_scale[0] = params_.lin_vel_scale;
  params_.commands_scale[1] = params_.lin_vel_scale;
  params_.commands_scale[2] = params_.ang_vel_scale;

  const float default_dof_pos_tmp[8] = {0, 0.8, -1.5, 0, 0, 0.8, -1.5, 0};
  for (int i = 0; i < 8; i++)
  {
    params_.default_dof_pos[i] = default_dof_pos_tmp[i];
  }

  x_vel_cmd_ = 0.;
  pitch_cmd_ = 0.;

  for (int i = 0; i < 297; i++)
    input_1.get()[i] = 0;
  for (int i = 0; i < 8; i++)
    output_last.get()[i] = 0;

  obs_.forward_vec[0] = 1.0;
  obs_.forward_vec[1] = 0.0;
  obs_.forward_vec[2] = 0.0;

  for (int j = 0; j < 8; j++)
  {
    action[j] = obs_.dof_pos[j];
  }
  a_l.setZero();

  for (int i = 0; i < history_length; i++)
  {
    // torch::Tensor obs_tensor = GetObs();
    // // append obs to obs buffer
    // obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);
    _GetObs();

    for (int i = 0; i < 297; i++)
      input_1_temp.get()[i] = input_1.get()[i + 33];

    for (int i = 0; i < 297; i++)
      input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < 33; i++)
      input_1.get()[i + 297] = input_0.get()[i];
  }
  std::cout << "[FSMState_RL] enter() 初始化完成" << std::endl;

  for (int i = 0; i < 10; i++)
  {
    _Forward();
  }

  threadRunning = true;
  if (thread_first_)
  {
    forward_thread = std::thread(&FSMState_RL::_Run_Forward, this);
    thread_first_ = false;
  }
  stop_update_ = false;
}

void FSMState_RL::run()
{
  // _data->state_command->clear();
  // _data->low_cmd->zero();
  x_vel_cmd_ = _data->state_command->rc_data_->twist_linear[point::X];
  pitch_cmd_ = _data->state_command->rc_data_->twist_angular[point::Z];
  // _data->state_command->rc_data_->twist_angular[point::Z]
  _data->low_cmd->qd.setZero();
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd.setZero();
  for(int i = 0; i < 8; i++)
  {
      if(i % 4 == 3)
      {
      _data->low_cmd->tau_cmd[i] = 0.5 * desired_pos[i] + 0.5 * (0 - _data->low_state->dq[i]);
      }
      else
      {
      _data->low_cmd->tau_cmd[i] = 30 * (desired_pos[i] - _data->low_state->q[i]) + 0.5 * (0 - _data->low_state->dq[i]);
      }
  }
}

void FSMState_RL::exit() 
{
  stop_update_ = true;
}

FSMStateName FSMState_RL::checkTransition()
{
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  switch (_data->state_command->desire_data_->fsm_state_name)
  {
  case FSMStateName::RECOVERY_STAND:
    this->_nextStateName = FSMStateName::RECOVERY_STAND;
    break;

  case FSMStateName::RL: // normal c
    break;

  case FSMStateName::TRANSFORM_DOWN:
    this->_nextStateName = FSMStateName::TRANSFORM_DOWN;
    break;

  case FSMStateName::PASSIVE: // normal c
    this->_nextStateName = FSMStateName::PASSIVE;
    break;
  default:
    break;
  }
  return this->_nextStateName;
}

void FSMState_RL::_GetObs()
{
  // omegawb = state_estimate_->omegaBody;
  // qwb = state_estimate_->orientation;
  // pwb = state_estimate_->position;
  // vwb = state_estimate_->vWorld;
    std::vector<float> obs_tmp;
    // compute gravity
    Mat3<double> _B2G_RotMat = this->_data->state_estimator->getResult().rBody;
    Mat3<double> _G2B_RotMat = this->_data->state_estimator->getResult().rBody.transpose();

    Vec3<double> angvel = a_l;
    a_l = 0.97*this->_data->state_estimator->getResult().omegaBody + 0.03*a_l;
    Vec3<double> projected_gravity = _B2G_RotMat * Vec3<double>(0.0, 0.0, -1.0);
    Vec3<double> projected_forward = _G2B_RotMat * Vec3<double>(1.0, 0.0, 0.0);
    // gravity
    // _gxFilter->addValue(angvel(0,0));
    // _gyFilter->addValue(angvel(1,0));
    // _gzFilter->addValue(angvel(2,0));
    //
    obs_tmp.push_back(angvel(0)*0.25);
    obs_tmp.push_back(angvel(1)*0.25);
    obs_tmp.push_back(angvel(2)*0.25);
    for(int i=0; i < 3; ++i)
    {
        std::cout << "angvel[" << i << "]=" << angvel(i) << std::endl;
    }
    for (int i = 0; i < 3; ++i)
    {
        obs_tmp.push_back(projected_gravity(i));
        std::cout << "gravity[" << i << "]=" << projected_gravity(i) << std::endl;
    }

    // cmd
    float rx = pitch_cmd_;//rx * (1 - smooth) + (std::fabs(_lowState->userValue.rx) < dead_zone ? 0.0 : _lowState->userValue.rx) * smooth;
    float ly = x_vel_cmd_;//ly * (1 - smooth) + (std::fabs(_lowState->userValue.ly) < dead_zone ? 0.0 : _lowState->userValue.ly) * smooth;

    float max = 1.0;
    float min = -1.0;

    float rot = rx*3.14;
    float vel = ly*2;

    double heading = 0.;
    double angle = (double)rot - heading;
    angle = fmod(angle,2.0*M_PI);
    if(angle > M_PI)
    {
        angle = angle - 2.0*M_PI;
    }
    angle = angle*0.5;
    angle = std::max(std::min((float)angle, max), min);
    angle = angle * 0.25;

    // obs_tmp.push_back(vel);
    // obs_tmp.push_back(0.0);
    // obs_tmp.push_back(angle);
    // std::cout << "cmd: vel=" << vel << ", angle=" << angle << std::endl;

    // pos
    for (int i = 0; i < 8; ++i)
    {   
        if(i % 4 == 3)
        {
            continue;
        }
        float pos = (this->obs_.dof_pos[i]  - this->params_.default_dof_pos[i]) * params_.dof_pos_scale;
        obs_tmp.push_back(pos);
        std::cout << "pos[" << i << "]=" << pos << std::endl;
    }
    // vel
    for (int i = 0; i < 8; ++i)
    {
        float vel = this->obs_.dof_vel[i] * params_.dof_vel_scale;
        obs_tmp.push_back(vel);
        std::cout << "vel[" << i << "]=" << vel << std::endl;
    }

    // last action
    //float index[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
    for (int i = 0; i < 8; ++i)
    {
        obs_tmp.push_back(output_last.get()[i]);
        std::cout << "last action[" << i << "]=" << output_last.get()[i] << std::endl;
    }
        //cmd
    obs_tmp.push_back(vel);
    obs_tmp.push_back(0.0);
    obs_tmp.push_back(angle);
    std::cout << "cmd: vel=" << vel << ", angle=" << angle << std::endl;
    std::cout << "obs_tmp size: " << obs_tmp.size() << std::endl;

    for(int i = 0; i < obs_tmp.size(); i++)
    {
        input_0.get()[i] = obs_tmp[i];
        std::cout << "input_0[" << i << "]=" << input_0.get()[i] << std::endl;
    }

}

void FSMState_RL::_Forward()
{
  std::cout << "[FSMState_RL] _Forward() 开始" << std::endl;
    _GetObs();
  std::cout << "[FSMState_RL] _Forward() 调用推理前" << std::endl;
  std::cout << "input_0 ptr: " << input_0.get() << ", input_1 ptr: " << input_1.get() << ", output ptr: " << output.get() << std::endl;
  std::cout << std::endl;
  cuda_test_->do_inference(input_0.get(), input_1.get(), output.get());
  std::cout << "[FSMState_RL] _Forward() 推理完成" << std::endl;

    for (int i = 0; i < 297; i++)
        input_1_temp.get()[i] = input_1.get()[i + 33];

    for (int i = 0; i < 297; i++)
        input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < 33; i++)
        input_1.get()[i + 297] = input_0.get()[i];

    for (int i = 0; i < 8; i++)
        output_last.get()[i] = output.get()[i];

}

void FSMState_RL::_Run_Forward()
{
  std::cout << "[FSMState_RL] _Run_Forward() 线程启动" << std::endl;
  while (threadRunning)
  {
    long long _start_time = getSystemTime();

    if (!stop_update_)
    {
      for (int i = 0; i < 4; i++)
      {
        obs_.dof_pos[i + 4] = _data->low_state->q[i];
        obs_.dof_vel[i + 4] = _data->low_state->dq[i];
        obs_.dof_pos[i] = _data->low_state->q[i + 4];
        obs_.dof_vel[i] = _data->low_state->dq[i + 4];
      }
      obs_.dof_pos[3] = 0;
      obs_.dof_pos[7] = 0;

      _Forward();

      for (int j = 0; j < 8; j++)
      {
        action[j] = output.get()[j] * params_.action_scale + params_.default_dof_pos[j];
      }

      for (int i = 0; i < 4; i++)
      {
        desired_pos[i+4] = action[i];
        desired_pos[i] = action[i+4];
        // std::cerr << "desired_pos" << i << ":" << desired_pos[i] << std::endl;
      }

      std::cout << "action "<< action[0] << " " << action[1] << " " << action[2] << " "
                << action[3] << " " << action[4] << " " << action[5] << " "
                << action[6] << " " << action[7] << std::endl;
    }

    absoluteWait(_start_time, (long long)(0.01 * 1000000));
  }
  std::cout << "[FSMState_RL] _Run_Forward() 线程结束" << std::endl;
  threadRunning = false;
}
