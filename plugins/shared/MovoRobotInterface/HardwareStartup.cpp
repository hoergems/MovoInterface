#include "HardwareStartup.hpp"

namespace oppt {
HardwareStartup::HardwareStartup():
	movoAPI_(new movo::MovoAPI) {

}

bool HardwareStartup::startupSequence(const std::string &localIP, const int &arm) {
	cout << "Execute startup sequence" << endl;
	if (startupSequenceFinished_) {
		cout << "Startup sequence has already finshed" << endl;
		return false;
	}

	int programResult = 0;
	if (!(movoAPI_->Initialized()))
		return false;

	//movoAPI_->CloseAPI();
	//////////////////////////////////////////////////////////////////////////////
	// Set the local IP and subnet mask
	//////////////////////////////////////////////////////////////////////////////
	std::string subnet_mask = "255.255.255.0";

	//////////////////////////////////////////////////////////////////////////////
	// Setup the ethernet comm config for the left arm
	//////////////////////////////////////////////////////////////////////////////
	EthernetCommConfig config;
	config.localIpAddress = inet_addr(localIP.c_str());
	config.subnetMask = inet_addr(subnet_mask.c_str());

	// Left arm
	if (arm == 1) {
		config.robotIpAddress = inet_addr("10.66.171.16");
		config.localCmdport = 24000;
		config.localBcastPort = 24024;
		config.robotPort = 44000;
		config.rxTimeOutInMs = 1;
	} else {
		// Right arm
		config.robotIpAddress = inet_addr("10.66.171.15");
		config.localCmdport = 25000;
		config.localBcastPort = 25025;
		config.robotPort = 55000;
		config.rxTimeOutInMs = 1;
	}

	// Initialize the API
	cout << "Init ethernet API" << endl;
	programResult = movoAPI_->InitEthernetAPI(config);
	//programResult = movoAPI_->InitAPI();
	if (programResult != 1) {
		cout << "Couldn't init API" << endl;
		return programResult;
	}

	//////////////////////////////////////////////////////////////////////////////
	// Make sure that we can communicate with the arm
	//////////////////////////////////////////////////////////////////////////////
	AngularPosition dataCommand;
	dataCommand.InitStruct();
	int dataCommandResult = -1;
	while (dataCommandResult != NO_ERROR_KINOVA) {
		dataCommandResult = movoAPI_->GetAngularCommand(dataCommand);
		if (dataCommandResult != NO_ERROR_KINOVA) {
			cout << "DATA COMMAND FAILED" << endl;
		}
	}
	/**int dataCommandResult = movoAPI_->GetAngularCommand(dataCommand);
	if (dataCommandResult != NO_ERROR_KINOVA) {
		cout << "Data command failed." << endl;
		return 0;
	}*/

	cout << "API initialized" << endl;

	//////////////////////////////////////////////////////////////////////////////
	//Make sure that the robot is in POSITION control
	//////////////////////////////////////////////////////////////////////////////
	int mode = -1;
	while (mode != 0) {
		movoAPI_->SwitchTrajectoryTorque(POSITION);
		movoAPI_->GetTrajectoryTorqueMode(mode);
		if (mode != 0)
			cout << "ERROR: Mode is NOT in POSITION" << endl;
	}

	cout << "Arm is in POSITION control mode" << endl;

	//////////////////////////////////////////////////////////////////////////////
	// The 7DOFs JACO arm requires to refresh the device list and set
	// the device corresponding to the arm as the active device
	//////////////////////////////////////////////////////////////////////////////
	int refreshResult = -1;
	while (refreshResult != NO_ERROR_KINOVA) {
		refreshResult = movoAPI_->RefreshDeviceList();
		if (refreshResult != NO_ERROR_KINOVA)
			cout << "Couldn't refresh devices. Trying again" << endl;
	}

	cout << "Refreshed device list" << endl;

	//////////////////////////////////////////////////////////////////////////////
	// Get the list of devices
	//////////////////////////////////////////////////////////////////////////////
	KinovaDevice devices[MAX_KINOVA_DEVICE];
	int getDevicesResult = -1;
	int numDevices = 0;
	cout << "Getting devices..." << endl;
	while (getDevicesResult != NO_ERROR_KINOVA) {
		numDevices = movoAPI_->GetDevices(devices, getDevicesResult);
	}

#ifdef ARM_HACK
	KinovaDevice d;
	std::string serialNumber = "WO510177-0";
	strcpy(d.SerialNumber, serialNumber.c_str());
	std::string model = "Spherical 7DOF Serv";
	strcpy(d.Model, model.c_str());
	d.VersionMajor = 6;
	d.VersionMinor = 2;
	d.VersionRelease = 5;
	d.DeviceType = 8;
	d.DeviceID = 0;
#else
	if (numDevices != 1) {
		cout << "Can't find primary device" << endl;
		return 0;
	}
#endif

	//////////////////////////////////////////////////////////////////////////////
	// Set the active device to the arm
	//////////////////////////////////////////////////////////////////////////////
#ifdef ARM_HACK
	int setResult = movoAPI_->SetActiveDevice(d);
#else
	int setResult = movoAPI_->SetActiveDevice(devices[0]);
#endif
	if (setResult != NO_ERROR_KINOVA) {
		cout << "Couldn't set active device. Can't continue" << endl;
		return 0;
	}

	cout << "Set active device" << endl;

	//////////////////////////////////////////////////////////////////////////////
	// Initialize the fingers
	//////////////////////////////////////////////////////////////////////////////
	cout << "Initializing fingers" << endl;
	int initFingersResults = -1;
	while (initFingersResults != NO_ERROR_KINOVA) {
		initFingersResults = movoAPI_->InitFingers();
		if (initFingersResults != NO_ERROR_KINOVA) {
			cout << "Couldn't initialize fingers. Trying again" << endl;
		}
	}

	startupSequenceFinished_ = true;
	return true;

}

std::unique_ptr<movo::MovoAPI> HardwareStartup::releaseMovoAPI() {
	return std::move(movoAPI_);
}
}