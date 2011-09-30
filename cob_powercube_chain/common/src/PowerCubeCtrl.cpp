/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_powercube_chain
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Dec 2010
 *
 * \brief
 *   Implementation of powercube control.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <cob_powercube_chain/PowerCubeCtrl.h>
<<<<<<< HEAD
//FOR DEBUG
#include <ros/ros.h>


// Modified
ros::Time last_time_pub_;
int countr = 0;


#define PCTRL_CHECK_INITIALIZED() \
if ( isInitialized()==false )											\
{																		\
    m_ErrorMessage.assign("Manipulator not initialized.");              \
	return false;														\
}

PowerCubeCtrl::PowerCubeCtrl(PowerCubeCtrlParams * params)
{
  m_mutex = PTHREAD_MUTEX_INITIALIZER;

  m_CANDeviceOpened = false;
  m_Initialized = false;

  m_params = params;

  m_horizon = 0.01; // sec

    }

  /** The Destructor
   */
PowerCubeCtrl::~PowerCubeCtrl()
{
  if (m_CANDeviceOpened)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_closeDevice()" << std::endl;
    PCube_closeDevice(m_DeviceHandle);
    pthread_mutex_unlock(&m_mutex);
  }
}

bool PowerCubeCtrl::Init(PowerCubeCtrlParams * params)
{
  int ret = 0;
  int DOF = m_params->GetDOF();
  std::string CanModule = m_params->GetCanModule();
  std::string CanDevice = m_params->GetCanDevice();
  std::vector<int> ModulIDs = m_params->GetModuleIDs();
  int CanBaudrate = m_params->GetBaudrate();
  std::vector<double> MaxVel = m_params->GetMaxVel();
  std::vector<double> MaxAcc = m_params->GetMaxAcc();
  std::vector<double> Offsets = m_params->GetOffsets();
  std::vector<double> LowerLimits = m_params->GetLowerLimits();
  std::vector<double> UpperLimits = m_params->GetUpperLimits();

  std::cout << " D  O  F  :" << DOF << std::endl;
  m_status.resize(DOF);
  m_dios.resize(DOF);
  m_positions.resize(DOF);

  std::cout << "=========================================================================== " << std::endl;
  std::cout << "PowerCubeCtrl:Init: Trying to initialize with the following parameters: " << std::endl;
  std::cout << "DOF: " << DOF << std::endl;
  std::cout << "CanModule: " << CanModule << std::endl;
  std::cout << "CanDevice: " << CanDevice << std::endl;
  std::cout << "CanBaudrate: " << CanBaudrate << std::endl;
  std::cout << "ModulIDs: ";
  for (int i = 0; i < DOF; i++)
  {
    std::cout << ModulIDs[i] << " ";
  }
  std::cout << std::endl << "maxVel: ";
  for (int i = 0; i < DOF; i++)
  {
    std::cout << MaxVel[i] << " ";
  }
  std::cout << std::endl << "maxAcc: ";
  for (int i = 0; i < DOF; i++)
  {
    std::cout << MaxAcc[i] << " ";
  }
  std::cout << std::endl << "upperLimits: ";
  for (int i = 0; i < DOF; i++)
  {
    std::cout << UpperLimits[i] << " ";
  }
  std::cout << std::endl << "lowerLimits: ";
  for (int i = 0; i < DOF; i++)
  {
    std::cout << LowerLimits[i] << " ";
  }
  std::cout << std::endl << "offsets: ";
  for (int i = 0; i < DOF; i++)
  {
    std::cout << Offsets[i] << " ";
  }
  std::cout << std::endl << "=========================================================================== " << std::endl;
  std::ostringstream InitStr;
  InitStr << CanModule << ":" << CanDevice << "," << CanBaudrate;
  std::cout << "initstring = " << InitStr.str().c_str() << std::endl;

  // open device
  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_openDevice()" << std::endl;
  ret = PCube_openDevice(&m_DeviceHandle, InitStr.str().c_str());
  pthread_mutex_unlock(&m_mutex);
  if (ret != 0)
  {
    std::ostringstream errorMsg;
    errorMsg << "Could not open device " << CanDevice << ", m5api error code: " << ret;
    m_ErrorMessage = errorMsg.str();
    return false;
  }
  m_CANDeviceOpened = true;

  // reset all modules
  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_resetAll()" << std::endl;
  ret = PCube_resetAll(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);
  if (ret != 0)
  {
    std::ostringstream errorMsg;
    errorMsg << "Could not reset all modules, m5api error code: " << ret;
    m_ErrorMessage = errorMsg.str();
    return false;
  }
  // Make sure m_IdModules is clear of Elements:
  ModulIDs.clear();

  // check number of modules connected to the bus
  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_getModuleCount()" << std::endl;
  int number_of_modules = PCube_getModuleCount(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);
  std::cout << "found " << number_of_modules << " modules." << std::endl;

  // Check if the modules are connected
  for (int i = 0; i < DOF; i++)
  {
    unsigned long serNo;

    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_getModuleSerialNo()" << std::endl;
    ret = PCube_getModuleSerialNo(m_DeviceHandle, ModulIDs[i], &serNo);
    pthread_mutex_unlock(&m_mutex);
    if (ret != 0)
    {
      std::ostringstream errorMsg;
      errorMsg << "Could not find Module with ID " << ModulIDs[i] << ", m5api error code: " << ret;
      m_ErrorMessage = errorMsg.str();
      return false;
    }
    // otherwise success
    std::cout << "Found module " << ModulIDs[i] << std::endl;
  }

  std::vector<std::string> errorMessages;
  PC_CTRL_STATUS status;
  getStatus(status, errorMessages);
  if ((status != PC_CTRL_OK) && (status != PC_CTRL_NOT_HOMED))
  {
    m_ErrorMessage.assign("");
    for (int i = 0; i < DOF; i++)
    {
      m_ErrorMessage.append(errorMessages[i]);
      m_ErrorMessage.append("\n");
    }
    return false;
  }
  else if (status == PC_CTRL_NOT_HOMED)
  {
    std::cout << "PowerCubeCtrl:Init: Homing is executed ...\n";
    bool successful = false;
    successful = doHoming();
    if (!successful)
    {
      std::cout << "PowerCubeCtrl:Init: homing not successful, aborting ...\n";
      return false;
    }
  }

  // Set angle offsets to hardware
  for (int i = 0; i < DOF; i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_setHomeOffset()" << std::endl;
    PCube_setHomeOffset(m_DeviceHandle, ModulIDs[i], Offsets[i]);
    pthread_mutex_unlock(&m_mutex);
  }

  // Set limits to hardware
  for (int i = 0; i < DOF; i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_setMinPos()" << std::endl;
    PCube_setMinPos(m_DeviceHandle, ModulIDs[i], LowerLimits[i]);
    pthread_mutex_unlock(&m_mutex);

    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_setMaxPos()" << std::endl;
    PCube_setMaxPos(m_DeviceHandle, ModulIDs[i], UpperLimits[i]);
    pthread_mutex_unlock(&m_mutex);
  }

  // Set max velocity to hardware
  setMaxVelocity(MaxVel);

  // Set max acceleration to hardware
  setMaxAcceleration(MaxAcc);

  // set synchronous or asynchronous movements
  setSyncMotion();
  //setASyncMotion();

  // @todo check status again
  m_Initialized = true;

  return true;
}

bool PowerCubeCtrl::Close()
{
  if (m_CANDeviceOpened)
  {
    m_Initialized = false;
    m_CANDeviceOpened = false;

    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_closeDevice()" << std::endl;
    PCube_closeDevice(m_DeviceHandle);
    pthread_mutex_unlock(&m_mutex);

    return true;
  }
  else
  {
    return false;
  }
}

bool PowerCubeCtrl::MoveJointSpaceSync(const std::vector<double>& target)
{
  PCTRL_CHECK_INITIALIZED();
  unsigned int DOF = m_params->GetDOF();

  std::vector<std::string> errorMessages;
  PC_CTRL_STATUS status;
  getStatus(status, errorMessages);
  if ((status != PC_CTRL_OK))
  {
    m_ErrorMessage.assign("");
    for (unsigned int i = 0; i < DOF; i++)
    {
      m_ErrorMessage.append(errorMessages[i]);
      m_ErrorMessage.append("\n");
    }
    return false;
  }

  std::vector<double> vel(DOF);
  std::vector<double> acc(DOF);

  double TG = 0;

  try
  {

    // calculate which joint takes the longest time to reach goal
    std::vector<double> times(DOF);

    for (unsigned int i = 0; i < DOF; i++)
    {
      RampCommand rm(m_positions[i], m_velocities[i], target[i], m_params->GetMaxAcc()[i], m_params->GetMaxVel()[i]);
      times[i] = rm.getTotalTime();
    }

    // determine the joint index that has the greatest value for time
    int furthest = 0;

    double max = times[0];

    for (unsigned int i = 1; i < DOF; i++)
    {
      if (times[i] > max)
      {
        max = times[i];
        furthest = i;
      }
    }

    RampCommand rm_furthest(m_positions[furthest], m_velocities[furthest], target[furthest],
                            m_params->GetMaxAcc()[furthest], m_params->GetMaxVel()[furthest]);

    double T1 = rm_furthest.T1();
    double T2 = rm_furthest.T2();
    double T3 = rm_furthest.T3();

    // total time:
    TG = T1 + T2 + T3;

    // calculate velocity and acceleration for all joints:
    acc[furthest] = m_params->GetMaxAcc()[furthest];
    vel[furthest] = m_params->GetMaxVel()[furthest];

    for (unsigned int i = 0; i < DOF; i++)
    {
      if (int(i) != furthest)
      {
        double a;
        double v;
        RampCommand::calculateAV(m_positions[i], m_velocities[i], target[i], TG, T3, m_params->GetMaxAcc()[i],
                                 m_params->GetMaxVel()[i], a, v);

        acc[i] = a;
        vel[i] = v;
      }
    }
  }
  catch (...)
  {
    return false;
  }

  // Send motion commands to hardware
  for (unsigned int i = 0; i < DOF; i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_moveRamp()" << std::endl;
    PCube_moveRamp(m_DeviceHandle, m_params->GetModuleIDs()[i], target[i], fabs(vel[i]), fabs(acc[i]));
    pthread_mutex_unlock(&m_mutex);
  }

  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_startMotionAll()" << std::endl;
  PCube_startMotionAll(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);

  return true;
}

bool PowerCubeCtrl::MoveVel(const std::vector<double>& velocities)
{
  PCTRL_CHECK_INITIALIZED();

  unsigned int DOF = m_params->GetDOF();
  std::vector<double> lowerLimits = m_params->GetLowerLimits();
  std::vector<double> upperLimits = m_params->GetUpperLimits();

  // check dimensions
  if (velocities.size() != DOF)
  {
    m_ErrorMessage = "Skipping command: Commanded velocities and DOF are not same dimension.";
    return false;
  }

  for (unsigned int i = 0; i < DOF; i++)
  {
    // check limits
    if (velocities[i] < lowerLimits[i] || velocities[i] > upperLimits[i])
    {
      std::ostringstream errorMsg;
      errorMsg << "Skipping command: Velocity " << velocities[i] << " exceeds limit " << lowerLimits[i] << " - " << upperLimits[i] << "for axis " << i;
      m_ErrorMessage = errorMsg.str();
      return false;
    }
  }

  std::vector<std::string> errorMessages;
  PC_CTRL_STATUS status;
  getStatus(status, errorMessages);
  if ((status != PC_CTRL_OK))
  {
    m_ErrorMessage.assign("");
    for (unsigned int i = 0; i < DOF; i++)
    {
      m_ErrorMessage.append(errorMessages[i]);
      m_ErrorMessage.append("\n");
    }
    return false;
  }
  //#############################################
  //############  D  E  B  U  G  ################
  //#############################################
  
  //Init
  float delta_t;
  
  //Berechne delta_t
  delta_t = ros::Time::now().toSec() - last_time_pub_.toSec(); 
  
  //Display Timegap with time before and after update
  std::cout << "\n-------\nTimegap\n-------\n time now = " << ros::Time::now() << "\n last time = " << last_time_pub_ << "\n difference = " << delta_t << std::endl;
				
  last_time_pub_ = ros::Time::now(); 
	
  for (unsigned int i = 0; i < DOF; i++) //vorher i < DOF
  {
 
    float pos;
    float cmd_pos;
    float cmd_pos_abs; // absolute position
    unsigned short cmd_time; // time in milliseconds
    
    // ToDo: Hier getHorizon() Funktion benutzen um die aktuelle Frequenz auszulesen die in arm_sdh.yaml gespeichert ist. Daraus mit 1/getHorizon() den ersten Wert ermitteln.
	if (countr == 0)
	{
		cmd_time = 17; 
		if (i == DOF-1)
			countr = countr + 1;
	}

	else
	{
		cmd_time = delta_t * 1000;
	}

    cmd_pos = (cmd_time/1000.0) * velocities[i];
	pthread_mutex_lock(&m_mutex);
    PCube_moveStepExtended(m_DeviceHandle, m_params->GetModuleID(i), m_positions[i] + cmd_pos, cmd_time, &m_status[i], &m_dios[i], &pos);
    pthread_mutex_unlock(&m_mutex);
	m_positions[i] = (double)pos;
  }

  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_startMotionAll()" << std::endl;
  PCube_startMotionAll(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);

  return true;
  
}

/// @brief Stops the manipulator immediately
bool PowerCubeCtrl::Stop()
{
  // stop should be executes without checking any conditions
  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_haltAll()" << std::endl;
  PCube_haltAll(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);

  // after halt the modules don't accept move commands any more, they first have to be reseted
  usleep(500000);
  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_resetAll()" << std::endl;
  PCube_resetAll(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);

  return true;
}

/// @brief Recovers the manipulator after an emergency stop
bool PowerCubeCtrl::Recover()
{
  std::vector<std::string> errorMessages;
  PC_CTRL_STATUS status;

  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_haltAll()" << std::endl;
  PCube_haltAll(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);

  usleep(500000);

  pthread_mutex_lock(&m_mutex);
  //std::cout << "------------------------------> PCube_resetAll()" << std::endl;
  PCube_resetAll(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);

  getStatus(status, errorMessages);
  if (status == PC_CTRL_NOT_HOMED)
  {
    if (!doHoming())
    {
      return false;
    }
  }

  // check if modules are back to normal state
  getStatus(status, errorMessages);
  if ((status != PC_CTRL_OK))
  {
    m_ErrorMessage.assign("");
    for (int i = 0; i < m_params->GetDOF(); i++)
    {
      m_ErrorMessage.append(errorMessages[i]);
      m_ErrorMessage.append("\n");
    }
    return false;
  }

  // modules successfully recovered
  return true;

}

/// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
bool PowerCubeCtrl::setMaxVelocity(double maxVelocity)
{
  PCTRL_CHECK_INITIALIZED();
  for (int i = 0; i < m_params->GetDOF(); i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_setMaxVel()" << std::endl;
    PCube_setMaxVel(m_DeviceHandle, m_params->GetModuleID(i), maxVelocity);
    pthread_mutex_unlock(&m_mutex);

    std::vector<double> maxVelocities(maxVelocity);
    m_params->SetMaxVel(maxVelocities);
  }

  return true;
}

bool PowerCubeCtrl::setMaxVelocity(const std::vector<double>& maxVelocities)
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_params->GetDOF(); i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_setMaxVel()" << std::endl;
    PCube_setMaxVel(m_DeviceHandle, m_params->GetModuleID(i), maxVelocities[i]);
    pthread_mutex_unlock(&m_mutex);
    m_params->SetMaxVel(maxVelocities);
  }

  return true;
}

/// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
bool PowerCubeCtrl::setMaxAcceleration(double maxAcceleration)
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_params->GetDOF(); i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_setMaxAcc()" << std::endl;
    PCube_setMaxAcc(m_DeviceHandle, m_params->GetModuleID(i), maxAcceleration);
    pthread_mutex_unlock(&m_mutex);
    std::vector<double> maxAccelerations(maxAcceleration);
    m_params->SetMaxAcc(maxAccelerations);
  }

  return true;
}

bool PowerCubeCtrl::setMaxAcceleration(const std::vector<double>& maxAccelerations)
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_params->GetDOF(); i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_setMaxAcc()" << std::endl;
    PCube_setMaxAcc(m_DeviceHandle, m_params->GetModuleID(i), maxAccelerations[i]);
    pthread_mutex_unlock(&m_mutex);
    m_params->SetMaxAcc(maxAccelerations);
  }

  return true;
}

/// @brief Sets the horizon (sec).
/// The horizon is the maximum step size which will be commanded to the powercube chain. In case
/// of a failure this is the time the powercube chain will continue to move until it is stopped.
bool PowerCubeCtrl::setHorizon(double horizon)
{
  m_horizon = horizon;

  return true;
}

/// @brief Gets the horizon (sec).
/// The horizon is the maximum step size which will be commanded to the powercube chain. In case
/// of a failure this is the time the powercube chain will continue to move until it is stopped.
double PowerCubeCtrl::getHorizon()
{
  return m_horizon;
}

/// @brief Configure powercubes to start all movements synchronously
/// Tells the Modules not to start moving until PCube_startMotionAll is called
bool PowerCubeCtrl::setSyncMotion()
{
  if (m_CANDeviceOpened)
  {
    for (int i = 0; i < m_params->GetDOF(); i++)
    {
      unsigned long confword;

      // get config
      pthread_mutex_lock(&m_mutex);
      //std::cout << "------------------------------> PCube_getConfig()" << std::endl;
      PCube_getConfig(m_DeviceHandle, m_params->GetModuleID(i), &confword);
      pthread_mutex_unlock(&m_mutex);

      // set config to synchronous
      pthread_mutex_lock(&m_mutex);
      //std::cout << "------------------------------> PCube_setConfig()" << std::endl;
      PCube_setConfig(m_DeviceHandle, m_params->GetModuleID(i), confword | CONFIGID_MOD_SYNC_MOTION);
      pthread_mutex_unlock(&m_mutex);
    }
    return true;
  }
  else
  {
    return false;
  }
}

/// @brief Configure powercubes to start all movements asynchronously
/// Tells the Modules to start immediately
bool PowerCubeCtrl::setASyncMotion()
{
  if (m_CANDeviceOpened)
  {
    for (int i = 0; i < m_params->GetDOF(); i++)
    {
      unsigned long confword;

      // get config
      pthread_mutex_lock(&m_mutex);
      //std::cout << "------------------------------> PCube_getConfig()" << std::endl;
      PCube_getConfig(m_DeviceHandle, m_params->GetModuleID(i), &confword);
      pthread_mutex_unlock(&m_mutex);

      // set config to asynchronous
      pthread_mutex_lock(&m_mutex);
      //std::cout << "------------------------------> PCube_setConfig()" << std::endl;
      PCube_setConfig(m_DeviceHandle, m_params->GetModuleID(i), confword & (~CONFIGID_MOD_SYNC_MOTION));
      pthread_mutex_unlock(&m_mutex);
    }
    return true;
  }
  else
  {
    return false;
  }
}

/// @brief Returns the current states
bool PowerCubeCtrl::updateStates()
{
  PCTRL_CHECK_INITIALIZED();

  unsigned int DOF = m_params->GetDOF();

  unsigned long state;
  unsigned char dio;
  float position;
  for (unsigned int i = 0; i < DOF; i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_getStateDioPos()" << std::endl;
    PCube_getStateDioPos(m_DeviceHandle, m_params->GetModuleID(i), &state, &dio, &position);
    pthread_mutex_unlock(&m_mutex);

    m_status[i] = state;
    m_dios[i] = dio;
    m_positions[i] = position;
<<<<<<< HEAD
	m_control_positions[i] = position;

=======
>>>>>>> 398b7f320cfd85f46127fa3b23df8b3324c980a3
    
    // @todo calculate vel and acc
    //m_velocities = ???;
    //m_accelerations = ???
  }

  return true;
}

/// @brief Gets the status of the modules
bool PowerCubeCtrl::getStatus(PC_CTRL_STATUS& status, std::vector<std::string>& errorMessages)
{
  unsigned int DOF = m_params->GetDOF();
  std::vector<int> ModuleIDs = m_params->GetModuleIDs();

  errorMessages.clear();
  errorMessages.resize(DOF);

  status = PC_CTRL_ERR;

  for (unsigned int i = 0; i < DOF; i++)
  {
    std::ostringstream errorMsg;

    if (m_status[i] & STATEID_MOD_POW_VOLT_ERR)
    {
      errorMsg << "Error in Module " << ModuleIDs[i] << ": ";
      errorMsg << "Motor voltage below minimum value!";
      errorMessages[i] = errorMsg.str();
      status = PC_CTRL_POW_VOLT_ERR;
    }
/*    else if (!(m_status[i] & STATEID_MOD_HOME))
    {
      errorMsg << "Warning: Module " << ModuleIDs[i];
      errorMsg << " is not referenced!";
      errorMessages[i] = errorMsg.str();
      status = PC_CTRL_NOT_HOMED;
    }*/
    else if (m_status[i] & STATEID_MOD_ERROR)
    {
      errorMsg << "Error in  Module " << ModuleIDs[i];
      errorMsg << " : Status code: " << std::hex << m_status[i];
      errorMessages[i] = errorMsg.str();
      status = PC_CTRL_ERR;
    }
    else
    {
      errorMsg << "Module with Id " << ModuleIDs[i];
      errorMsg << ": Status OK.";
      errorMessages[i] = errorMsg.str();
      status = PC_CTRL_OK;
    }
  }
  return true;
}

/// @brief Returns true if some cubes are still moving
bool PowerCubeCtrl::statusMoving()
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_params->GetDOF(); i++)
  {
    if (m_status[i] & STATEID_MOD_MOTION)
      return true;
  }
  return false;
}

/// @brief gets the current positions
std::vector<double> PowerCubeCtrl::getPositions()
{
  return m_positions;
}

/// @brief gets the current velocities
std::vector<double> PowerCubeCtrl::getVelocities()
{
  // @todo calculate new velocities before returning
  return m_velocities;
}

/// @brief gets the current positions
std::vector<double> PowerCubeCtrl::getAccelerations()
{
  // @todo calculate new accelerations before returning
  return m_accelerations;
}

/// @brief does homing for all Modules
bool PowerCubeCtrl::doHoming()
{
  unsigned int DOF = m_params->GetDOF();
  std::vector<int> ModuleIDs = m_params->GetModuleIDs();

  // start homing
  int ret = 0;
  for (unsigned int i = 0; i < DOF; i++)
  {
    pthread_mutex_lock(&m_mutex);
    //std::cout << "------------------------------> PCube_homeModule()" << std::endl;
    ret = PCube_homeModule(m_DeviceHandle, ModuleIDs[i]);
    pthread_mutex_unlock(&m_mutex);
    if (ret != 0)
    {
      std::ostringstream errorMsg;
      errorMsg << "Can't start homing for module " << ModuleIDs[i] << ", m5api error code: " << ret;
      m_ErrorMessage = errorMsg.str();
      return false;
    }
  }

  // wait until all modules are homed
  double max_homing_time = 10.0; // seconds
  double homing_time = 0.0;
  double intervall = 0.1;

  for (unsigned int i = 0; i < DOF; i++)
  {
    unsigned long int help;
    do
    {
      pthread_mutex_lock(&m_mutex);
      //std::cout << "------------------------------> PCube_getModuleState()" << std::endl;
      PCube_getModuleState(m_DeviceHandle, ModuleIDs[i], &help);
      pthread_mutex_unlock(&m_mutex);

      usleep(intervall * 1000000); // convert sec to usec
    } while ((help & STATEID_MOD_HOME) == 0);
    m_status[i] = help;
  }

  for (unsigned int i = 0; i < DOF; i++)
  {
    homing_time = 0.0;
    /*while ((homing_time < max_homing_time))
     {
     updateStates();
     if ((m_status[i] & STATEID_MOD_HOME))
     {
     std::cout << "Module " << ModuleIDs[i] << " homed in " << homing_time << "sec." << std::endl;
     }

     usleep(intervall * 1000000); // convert sec to usec
     homing_time = homing_time + intervall;
     }
     */
    // check result
    if (!(m_status[i] & STATEID_MOD_HOME))
    {
      std::cout << "Homing failed: Error in  Module " << ModuleIDs[i] << std::endl;
      m_pc_status = PC_CTRL_NOT_HOMED;
      return false;
    }
  }

  // homing all modules successfully
  m_pc_status = PC_CTRL_OK;
  return true;

}
