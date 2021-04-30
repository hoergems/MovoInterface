#ifndef _MOVO_API_
#define _MOVO_API_
#include <oppt/opptCore/core.hpp>
#include "defs.hpp"
#include <mutex>

namespace movo {
class MovoAPI {
public:
	MovoAPI();

	virtual ~MovoAPI() = default;

	bool Initialized() const;

	int InitAPI() const;

	int CloseAPI() const;

	int GetClientConfigurations(ClientConfigurations &config) const;

	int StartControlAPI() const;

	int StopControlAPI() const;

	int SetAngularControl() const;

	int GetQuickStatus(QuickStatus &status) const;

	int GetControlType(int &response) const;

	int RestoreFactoryDefault() const;

	int InitEthernetAPI(EthernetCommConfig &config);

	int InitCommunicationEthernet(EthernetCommConfig &config);

	int InitFingers() const;

	int RefreshDeviceList() const;

	int GetAllRobotIdentity(RobotIdentity identity[MAX_KINOVA_DEVICE], int &count) const;

	int GetDeviceCount(int &result) const;

	int GetAngularCommand(AngularPosition &angularPosition) const;

	int GetAngularPosition(AngularPosition &angularPosition) const;

	int GetDevices(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) const;

	int GetActiveDevice(KinovaDevice &device) const;

	int SetActiveDevice(KinovaDevice device) const;

	int ScanForNewDevice() const;

	int GetActuatorAcceleration(AngularAcceleration &acceleration) const;

	int GetAngularVelocity(AngularPosition &velocity) const;

	int GetAngularTorqueCommand(float Command[NUM_JOINTS]) const;

	int MoveHome() const;

	int RunGravityZEstimationSequence(ROBOT_TYPE type, FloatType OptimalzParam[OPTIMAL_Z_PARAM_SIZE]) const;

	int SwitchTrajectoryTorque(GENERALCONTROL_TYPE type) const;

	int SetTorqueSafetyFactor(const float &factor) const;

	int SendAngularTorqueCommand(float Command[NUM_JOINTS]) const;

	int SendCartesianForceCommand(float Command[NUM_JOINTS]) const;

	int SetGravityVector(float Command[3]) const;

	int SetGravityPayload(float Command[GRAVITY_PAYLOAD_SIZE]) const;

	int SetGravityOptimalZParam(float Command[GRAVITY_PARAM_SIZE]) const;

	int SetGravityType(GRAVITY_TYPE Type) const;

	int StartCurrentLimitation() const;

	int StopCurrentLimitation() const;

	int SetTorqueVibrationController(const float &value) const;

	int GetAngularForceGravityFree(AngularPosition &position) const;

	int GetCartesianForce(CartesianPosition &position) const;

	int GetCartesianPosition(CartesianPosition &position) const;

	int SetTorqueControlType(TORQUECONTROL_TYPE type) const;

	int SetTorqueCommandMax(float Command[COMMAND_SIZE]) const;

	int GetTrajectoryTorqueMode(int &mode) const;

	int GetAngularForce(AngularPosition &position) const;

	int SetTorqueZero(int ActuatorAddress) const;

	int SendBasicTrajectory(TrajectoryPoint command) const;

	int SendAdvanceTrajectory(TrajectoryPoint command) const;

	int SetTorqueActuatorDamping(float Command[COMMAND_SIZE]) const;

	int SetTorqueDampingMax(float Command[COMMAND_SIZE]) const;

	int SetTorqueRobotProtection(int &protection) const;

	void ApplyTorques(float Torques[NUM_JOINTS], const FloatType &duration) const;

	int EraseAllTrajectories() const;

	int GetSystemErrorCount(unsigned int &count) const;

	int ClearErrorLog() const;

	int GetSystemError(unsigned int index, SystemError &Response) const;

	int SetTorqueInactivityType(int type) const;

	int GetAngularCurrent(AngularPosition &response) const;

	int GetAngularCurrentMotor(AngularPosition &response) const;

	int SetAngularTorqueMinMax(AngularInfo &min, AngularInfo &max) const;

private:
	void declareFunctions_();

	mutable std::mutex movoAPIMutex_;
};

}

#endif