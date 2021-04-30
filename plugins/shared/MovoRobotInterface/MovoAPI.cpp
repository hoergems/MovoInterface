#include "MovoAPI.hpp"
#include <chrono>
#include <thread>

using std::cout;
using std::endl;

void * movo::commandLayer_handle;
void * movo::commLayer_handle;
int(*movo::MyInitEthernetAPI)(EthernetCommConfig &);
int(*movo::MyInitCommunicationEthernet)(EthernetCommConfig &);
int(*movo::MyInitAPI)(void);
int(*movo::MyGetClientConfigurations)(ClientConfigurations &);
int(*movo::MyStartControlAPI)();
int(*movo::MyStopControlAPI)();
int(*movo::MySetAngularControl)();
int(*movo::MyGetControlType)(int &);
int(*movo::MyGetQuickStatus)(QuickStatus &);
int(*movo::MyRestoreFactoryDefault)();
int(*movo::MyCloseAPI)();
int(*movo::MyInitFingers)();
int(*movo::MyRefresDevicesList)();
int(*movo::MyGetAllRobotIdentity)(RobotIdentity robotIdentity[MAX_KINOVA_DEVICE], int &count);
int(*movo::MyGetAngularCommand)(AngularPosition &);
int(*movo::MyGetAngularPosition)(AngularPosition &);
int(*movo::MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*movo::MyGetActiveDevice)(KinovaDevice &);
int(*movo::MyGetDeviceCount)(int &result);
int(*movo::MySetActiveDevice)(KinovaDevice device);
int(*movo::MyScanForNewDevice)();
int(*movo::MyGetActuatorAcceleration)(AngularAcceleration & Response);
int(*movo::MyGetAngularVelocity)(AngularPosition & Response);
int(*movo::MyStartCurrentLimitation)();
int(*movo::MyStopCurrentLimitation)();
int(*movo::MyMoveHome)();
int(*movo::MyRunGravityZEstimationSequence)(ROBOT_TYPE type, FloatType OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
int(*movo::MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
int(*movo::MySetTorqueSafetyFactor)(float factor);
int(*movo::MySendAngularTorqueCommand)(float Command[NUM_JOINTS]);
int(*movo::MySendCartesianForceCommand)(float Command[NUM_JOINTS]);
int(*movo::MySetGravityVector)(float Command[3]);
int(*movo::MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
int(*movo::MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
int(*movo::MySetGravityType)(GRAVITY_TYPE Type);
int(*movo::MySetTorqueVibrationController)(float value);
int(*movo::MySetTorqueCommandMax)(float Command[COMMAND_SIZE]);
int(*movo::MyGetAngularForceGravityFree)(AngularPosition &);
int(*movo::MyGetCartesianForce)(CartesianPosition &);
int(*movo::MyGetCartesianPosition)(CartesianPosition &);
int(*movo::MySetTorqueControlType)(TORQUECONTROL_TYPE type);
int(*movo::MyGetTrajectoryTorqueMode)(int &);
int(*movo::MyGetAngularForce)(AngularPosition &);
int(*movo::MySetTorqueZero)(int ActuatorAddress);
int(*movo::MySendBasicTrajectory)(TrajectoryPoint command);
int(*movo::MySendAdvanceTrajectory)(TrajectoryPoint command);
int(*movo::MySetTorqueActuatorDamping)(float Command[COMMAND_SIZE]);
int(*movo::MySetTorqueDampingMax)(float Command[COMMAND_SIZE]);
int(*movo::MyEraseAllTrajectories)();
int(*movo::MySetTorqueRobotProtection)(int);
int(*movo::MyGetSystemErrorCount)(unsigned int &);
int(*movo::MyGetSystemError)(unsigned int index, SystemError &Response);
int(*movo::MyClearErrorLog)();
int(*movo::MySetTorqueInactivityType)(int);
int(*movo::MyGetAngularTorqueCommand)(float Command[NUM_JOINTS]);
int(*movo::MyGetAngularCurrent)(AngularPosition&);
int(*movo::MyGetAngularCurrentMotor)(AngularPosition&);
int(*movo::MySetAngularTorqueMinMax)(AngularInfo&, AngularInfo&);


namespace movo {
MovoAPI::MovoAPI() {
	commandLayer_handle = dlopen("Kinova.API.EthCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
	commLayer_handle = dlopen("Kinova.API.CommLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
	declareFunctions_();

}

void MovoAPI::declareFunctions_() {
	MyInitEthernetAPI = (int(*)(EthernetCommConfig &)) dlsym(commandLayer_handle, "Ethernet_InitEthernetAPI");
	MyInitCommunicationEthernet = (int(*)(EthernetCommConfig &)) dlsym(commLayer_handle, "InitCommunicationEthernet");
	MyInitAPI = (int(*)(void)) dlsym(commandLayer_handle, "Ethernet_InitAPI");
	MyStartControlAPI = (int(*)()) dlsym(commandLayer_handle, "Ethernet_StartControlAPI");
	MyStopControlAPI = (int(*)()) dlsym(commandLayer_handle, "Ethernet_StopControlAPI");
	MySetAngularControl = (int(*)()) dlsym(commandLayer_handle, "Ethernet_SetAngularControl");
	MyRestoreFactoryDefault = (int(*)()) dlsym(commandLayer_handle, "Ethernet_RestoreFactoryDefault");
	MyRefresDevicesList = (int(*)(void)) dlsym(commandLayer_handle, "Ethernet_RefresDevicesList");
	MyGetControlType = (int(*)(int &)) dlsym(commandLayer_handle, "Ethernet_GetControlType");
	MyGetQuickStatus = (int(*)(QuickStatus &)) dlsym(commandLayer_handle, "Ethernet_GetQuickStatus");
	MyGetClientConfigurations = (int(*)(ClientConfigurations &)) dlsym(commandLayer_handle, "Ethernet_GetClientConfigurations");
	MyGetAllRobotIdentity = (int(*)(RobotIdentity identity[MAX_KINOVA_DEVICE], int &count)) dlsym(commandLayer_handle, "Ethernet_GetAllRobotIdentity");
	MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "Ethernet_CloseAPI");
	MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularCommand");
	MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularPosition");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "Ethernet_GetDevices");
	MyGetDeviceCount = (int(*)(int &)) dlsym(commLayer_handle, "GetDeviceCount");
	MyGetActiveDevice = (int(*)(KinovaDevice &)) dlsym(commLayer_handle, "GetActiveDevice");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "Ethernet_SetActiveDevice");
	MyScanForNewDevice = (int(*)()) dlsym(commLayer_handle, "ScanForNewDevice");
	MyStartCurrentLimitation = (int(*)()) dlsym(commandLayer_handle, "Ethernet_StartCurrentLimitation");
	MyStopCurrentLimitation = (int(*)()) dlsym(commandLayer_handle, "Ethernet_StopCurrentLimitation");
	MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "Ethernet_GetActuatorAcceleration");
	MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularVelocity");
	MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, FloatType OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "Ethernet_RunGravityZEstimationSequence");
	MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "Ethernet_SwitchTrajectoryTorque");
	MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "Ethernet_SetTorqueSafetyFactor");
	MySendAngularTorqueCommand = (int(*)(float Command[NUM_JOINTS])) dlsym(commandLayer_handle, "Ethernet_SendAngularTorqueCommand");
	MySendCartesianForceCommand = (int(*)(float Command[NUM_JOINTS])) dlsym(commandLayer_handle, "Ethernet_SendCartesianForceCommand");
	MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "Ethernet_SetGravityVector");
	MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetGravityPayload");
	MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetGravityOptimalZParam");
	MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "Ethernet_SetGravityType");
	MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularForceGravityFree");
	MyGetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "Ethernet_GetCartesianForce");
	MyGetCartesianPosition = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "Ethernet_GetCartesianPosition");
	MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "Ethernet_SetTorqueVibrationController");
	MySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "Ethernet_SetTorqueControlType");
	MySetTorqueCommandMax = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "Ethernet_SetTorqueCommandMax");
	MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "Ethernet_MoveHome");
	MyGetTrajectoryTorqueMode = (int(*)(int &)) dlsym(commandLayer_handle, "Ethernet_GetTrajectoryTorqueMode");
	MyGetAngularForce = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "Ethernet_GetAngularForce");
	MySetTorqueZero = (int(*)(int)) dlsym(commandLayer_handle, "Ethernet_SetTorqueZero");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "Ethernet_SendBasicTrajectory");
	MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "Ethernet_SendAdvanceTrajectory");
	MySetTorqueActuatorDamping = (int (*)(float Command[NUM_JOINTS])) dlsym(commandLayer_handle, "Ethernet_SetTorqueActuatorDamping");
	MySetTorqueDampingMax = (int (*)(float Command[NUM_JOINTS])) dlsym(commandLayer_handle, "Ethernet_SetTorqueDampingMax");
	MyEraseAllTrajectories = (int(*)()) dlsym(commandLayer_handle, "Ethernet_EraseAllTrajectories");
	MyInitFingers = (int(*)()) dlsym(commandLayer_handle, "Ethernet_InitFingers");
	MySetTorqueRobotProtection = (int(*)(int protection)) dlsym(commandLayer_handle, "Ethernet_SetTorqueRobotProtection");
	MyGetSystemErrorCount = (int(*)(unsigned int &)) dlsym(commandLayer_handle, "Ethernet_GetSystemErrorCount");
	MyGetSystemError = (int(*)(unsigned int idx, SystemError & response)) dlsym(commandLayer_handle, "Ethernet_GetSystemError");
	MyClearErrorLog = (int(*)()) dlsym(commandLayer_handle, "Ethernet_ClearErrorLog");
	MySetTorqueInactivityType = (int(*)(int)) dlsym(commandLayer_handle, "Ethernet_SetTorqueInactivityType");
	MyGetAngularTorqueCommand = (int(*)(float Command[NUM_JOINTS])) dlsym(commandLayer_handle, "Ethernet_GetAngularTorqueCommand");
	MyGetAngularCurrent = (int(*)(AngularPosition&)) dlsym(commandLayer_handle, "Ethernet_GetAngularCurrent");
	MyGetAngularCurrentMotor = (int(*)(AngularPosition&)) dlsym(commandLayer_handle, "Ethernet_GetAngularCurrentMotor");
	MySetAngularTorqueMinMax = (int(*)(AngularInfo&, AngularInfo&)) dlsym(commandLayer_handle, "Ethernet_SetAngularTorqueMinMax");
}

bool MovoAPI::Initialized() const {
	if ((MyInitEthernetAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyGetAngularPosition == NULL)
	        || (MySetActiveDevice == NULL) || (MyGetDevices == NULL) || (MyGetAngularVelocity == NULL) ||
	        (MyRestoreFactoryDefault == NULL) || (MyGetControlType == NULL) || (MyGetQuickStatus == NULL))	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		return false;
	}

	return true;
}

int MovoAPI::InitAPI() const {
	return MyInitAPI();
}

int MovoAPI::CloseAPI() const {
	return MyCloseAPI();
}

int MovoAPI::GetClientConfigurations(ClientConfigurations &config) const {
	return MyGetClientConfigurations(config);
}

int MovoAPI::GetQuickStatus(QuickStatus &status) const {
	return MyGetQuickStatus(status);
}

int MovoAPI::StartControlAPI() const {
	return MyStartControlAPI();
}

int MovoAPI::StopControlAPI() const {
	return MyStopControlAPI();
}

int MovoAPI::SetAngularControl() const {
	return MySetAngularControl();
}

int MovoAPI::GetControlType(int &response) const {
	return MyGetControlType(response);
}

int MovoAPI::RestoreFactoryDefault() const {
	return MyRestoreFactoryDefault();
}

int MovoAPI::InitEthernetAPI(EthernetCommConfig &config) {
	return MyInitEthernetAPI(config);
}

int MovoAPI::InitCommunicationEthernet(EthernetCommConfig &config) {
	return MyInitCommunicationEthernet(config);
}

int MovoAPI::RefreshDeviceList() const {
	return MyRefresDevicesList();
}

int MovoAPI::GetAllRobotIdentity(RobotIdentity identity[MAX_KINOVA_DEVICE], int &count) const {
	return MyGetAllRobotIdentity(identity, count);
}

int MovoAPI::InitFingers() const {
	return MyInitFingers();
}

int MovoAPI::GetDeviceCount(int &result) const {
	cout << "HELLO" << endl;
	return MyGetDeviceCount(result);
}

int MovoAPI::GetActiveDevice(KinovaDevice &device) const {
	return MyGetActiveDevice(device);
}

int MovoAPI::GetAngularCommand(AngularPosition &angularPosition) const {
	return MyGetAngularCommand(angularPosition);
}

int MovoAPI::GetAngularPosition(AngularPosition &angularPosition) const {
	return MyGetAngularPosition(angularPosition);
}

int MovoAPI::GetAngularCurrent(AngularPosition &response) const {
	return MyGetAngularCurrent(response);
}

int MovoAPI::GetAngularCurrentMotor(AngularPosition &response) const {
	return MyGetAngularCurrentMotor(response);
}

int MovoAPI::GetDevices(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) const {
	return MyGetDevices(devices, result);
}

int MovoAPI::StartCurrentLimitation() const {
	return MyStartCurrentLimitation();
}

int MovoAPI::StopCurrentLimitation() const {
	return MyStopCurrentLimitation();
}

int MovoAPI::SetActiveDevice(KinovaDevice device) const {
	return MySetActiveDevice(device);
}

int MovoAPI::ScanForNewDevice() const {
	return MyScanForNewDevice();
}

int MovoAPI::GetActuatorAcceleration(AngularAcceleration &acceleration) const {
	return MyGetActuatorAcceleration(acceleration);
}

int MovoAPI::GetAngularVelocity(AngularPosition &velocity) const {
	return MyGetAngularVelocity(velocity);
}

int MovoAPI::MoveHome() const {
	return MyMoveHome();
}

int MovoAPI::RunGravityZEstimationSequence(ROBOT_TYPE type, FloatType OptimalzParam[OPTIMAL_Z_PARAM_SIZE]) const {
	return MyRunGravityZEstimationSequence(type, OptimalzParam);
}

int MovoAPI::SwitchTrajectoryTorque(GENERALCONTROL_TYPE type) const {
	movoAPIMutex_.lock();
	int res = MySwitchTrajectoryTorque(type);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetTorqueSafetyFactor(const float &factor) const {
	movoAPIMutex_.lock();
	int res = MySetTorqueSafetyFactor(factor);
	movoAPIMutex_.unlock();
	return res;	
}

int MovoAPI::SendAngularTorqueCommand(float Command[NUM_JOINTS]) const {
	movoAPIMutex_.lock();
	int res = MySendAngularTorqueCommand(Command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SendCartesianForceCommand(float Command[NUM_JOINTS]) const {
	movoAPIMutex_.lock();
	int res = MySendCartesianForceCommand(Command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetGravityVector(float Command[3]) const {
	movoAPIMutex_.lock();
	int res = MySetGravityVector(Command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetGravityPayload(float Command[GRAVITY_PAYLOAD_SIZE]) const {
	movoAPIMutex_.lock();
	int res = MySetGravityPayload(Command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetGravityOptimalZParam(float Command[GRAVITY_PARAM_SIZE]) const {
	movoAPIMutex_.lock();
	int res = MySetGravityOptimalZParam(Command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetGravityType(GRAVITY_TYPE Type) const {
	movoAPIMutex_.lock();
	int res = MySetGravityType(Type);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetTorqueVibrationController(const float &value) const {
	movoAPIMutex_.lock();
	int res = MySetTorqueVibrationController(value);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::GetAngularForceGravityFree(AngularPosition &position) const {
	movoAPIMutex_.lock();
	int res = MyGetAngularForceGravityFree(position);
}

int MovoAPI::GetCartesianForce(CartesianPosition &position) const {
	movoAPIMutex_.lock();
	int res = MyGetCartesianForce(position);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::GetCartesianPosition(CartesianPosition &position) const {
	movoAPIMutex_.lock();
	int res = MyGetCartesianPosition(position);
	movoAPIMutex_.unlock();
	return res;
}


int MovoAPI::SetTorqueControlType(TORQUECONTROL_TYPE type) const {
	movoAPIMutex_.lock();
	int res = MySetTorqueControlType(type);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::GetTrajectoryTorqueMode(int &mode) const {
	movoAPIMutex_.lock();
	int res = MyGetTrajectoryTorqueMode(mode);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::GetAngularForce(AngularPosition &position) const {
	int res = MyGetAngularForce(position);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::GetSystemErrorCount(unsigned int &count) const {
	movoAPIMutex_.lock();
	int res = MyGetSystemErrorCount(count);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::ClearErrorLog() const {
	movoAPIMutex_.lock();
	int res = MyClearErrorLog();
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::GetSystemError(unsigned int index, SystemError &Response) const {
	movoAPIMutex_.lock();
	int res = MyGetSystemError(index, Response);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetTorqueZero(int ActuatorAddress) const {
	movoAPIMutex_.lock();
	int res = 0;
	while (res == 0)
		res = MySetTorqueZero(ActuatorAddress);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SendBasicTrajectory(TrajectoryPoint command) const {
	movoAPIMutex_.lock();
	int res = 0;
	while (res == 0)
		res = MySendBasicTrajectory(command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SendAdvanceTrajectory(TrajectoryPoint command) const {
	movoAPIMutex_.lock();
	int res = 0;
	while (res == 0)
		res = MySendAdvanceTrajectory(command);
    movoAPIMutex_.unlock();
	return res;
}


int MovoAPI::SetTorqueActuatorDamping(float Command[COMMAND_SIZE]) const {
	movoAPIMutex_.lock();
	int res = MySetTorqueActuatorDamping(Command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetTorqueDampingMax(float Command[COMMAND_SIZE]) const {
	movoAPIMutex_.lock();
	int res = MySetTorqueDampingMax(Command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetTorqueCommandMax(float Command[COMMAND_SIZE]) const {
	movoAPIMutex_.lock();
	int res = MySetTorqueCommandMax(Command);
	movoAPIMutex_.unlock();
	return res;
}


int MovoAPI::SetTorqueRobotProtection(int &protection) const {
	movoAPIMutex_.lock();
	int res = MySetTorqueRobotProtection(protection);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::EraseAllTrajectories() const {
	movoAPIMutex_.lock();
	int res = MyEraseAllTrajectories();
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetTorqueInactivityType(int type) const {
	movoAPIMutex_.lock();
	int res = MySetTorqueInactivityType(type);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::GetAngularTorqueCommand(float Command[NUM_JOINTS]) const {
	movoAPIMutex_.lock();
	int res = MyGetAngularTorqueCommand(Command);
	movoAPIMutex_.unlock();
	return res;
}

int MovoAPI::SetAngularTorqueMinMax(AngularInfo &min, AngularInfo &max) const {
	movoAPIMutex_.lock();
	int res = MySetAngularTorqueMinMax(min, max);
	movoAPIMutex_.unlock();
	return res;
}

void MovoAPI::ApplyTorques(float Torques[NUM_JOINTS], const FloatType &duration) const {
	auto timeNow = std::chrono::system_clock::now();
	std::chrono::milliseconds tick(20);
	std::chrono::duration<FloatType> dur;
	FloatType totalDuration = 0.0;
	int res;
	while (true) {
		// Send at a rate of 20 hz
		timeNow = std::chrono::system_clock::now();
		res = SendAngularTorqueCommand(Torques);
		std::this_thread::sleep_for(tick);
		dur = std::chrono::system_clock::now() - timeNow;
		totalDuration += dur.count();
		if (totalDuration >= duration) {
			return;
		}

	}
}

}