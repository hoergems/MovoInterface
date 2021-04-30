#ifndef _DEFS_HPP_
#define _DEFS_HPP_

#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <Kinova.API.EthCommLayerUbuntu.h>
#include <Kinova.API.EthCommandLayerUbuntu.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <oppt/defs.hpp>

#define NUM_JOINTS 7

namespace movo {

//extern int NUM_JOINTS;
//Handle for the library's command layer.
extern void * commandLayer_handle;
extern void * commLayer_handle;
//Function pointers to the functions we need
extern int (*MyInitAPI)(void);
extern int (*MyInitEthernetAPI)(EthernetCommConfig &);
extern int (*MyInitCommunicationEthernet)(EthernetCommConfig &);
extern int (*MyCloseAPI)();
extern int (*MyInitFingers)();
extern int (*MyRefresDevicesList)();
extern int (*MyGetClientConfigurations)(ClientConfigurations &);
extern int (*MyGetControlType)(int &);
extern int(*MyGetQuickStatus)(QuickStatus &);
extern int(*MyGetAllRobotIdentity)(RobotIdentity robotIdentity[MAX_KINOVA_DEVICE], int &count);
extern int(*MyRestoreFactoryDefault)();
extern int(*MyStartControlAPI)();
extern int(*MyStopControlAPI)();
extern int(*MyStartCurrentLimitation)();
extern int(*MyStopCurrentLimitation)();
extern int(*MySetAngularControl)();
extern int(*MyGetAngularCommand)(AngularPosition &);
extern int(*MyGetAngularPosition)(AngularPosition &);
extern int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
extern int(*MyGetActiveDevice)(KinovaDevice &);
extern int(*MyGetDeviceCount)(int &result);
extern int(*MyScanForNewDevice)();
extern int(*MySetActiveDevice)(KinovaDevice device);
extern int(*MyGetActuatorAcceleration)(AngularAcceleration & Response);
extern int(*MyGetAngularVelocity)(AngularPosition & Response);
extern int(*MyMoveHome)();
extern int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, FloatType OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
extern int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
extern int(*MySetTorqueSafetyFactor)(float factor);
extern int(*MySendAngularTorqueCommand)(float Command[NUM_JOINTS]);
extern int(*MySendCartesianForceCommand)(float Command[NUM_JOINTS]);
extern int(*MySetGravityVector)(float Command[3]);
extern int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
extern int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
extern int(*MySetGravityType)(GRAVITY_TYPE Type);
extern int(*MySetTorqueVibrationController)(float value);
extern int(*MyGetAngularForceGravityFree)(AngularPosition &);
extern int(*MyGetCartesianForce)(CartesianPosition &);
extern int(*MyGetCartesianPosition)(CartesianPosition &);
extern int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
extern int(*MySetTorqueCommandMax)(float Command[COMMAND_SIZE]);
extern int(*MyGetTrajectoryTorqueMode)(int &);
extern int(*MyGetAngularForce)(AngularPosition &);
extern int(*MySetTorqueZero)(int ActuatorAddress);
extern int (*MySendBasicTrajectory)(TrajectoryPoint command);
extern int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
extern int(*MySetTorqueActuatorDamping)(float Command[COMMAND_SIZE]);
extern int(*MySetTorqueDampingMax)(float Command[COMMAND_SIZE]);
extern int(*MyEraseAllTrajectories)();
extern int(*MySetTorqueRobotProtection)(int protection);
extern int(*MyGetSystemErrorCount)(unsigned int &count);
extern int(*MyGetSystemError)(unsigned int index, SystemError &Response);
extern int(*MyClearErrorLog)();
extern int(*MySetTorqueInactivityType)(int);
extern int(*MyGetAngularTorqueCommand)(float Command[NUM_JOINTS]);
extern int(*MyGetAngularCurrent)(AngularPosition&);
extern int(*MyGetAngularCurrentMotor)(AngularPosition&);
extern int(*MySetAngularTorqueMinMax)(AngularInfo&, AngularInfo&);

}

#endif