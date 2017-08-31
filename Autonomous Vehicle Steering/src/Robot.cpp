#include <Joystick.h>
#include <SampleRobot.h>
#include <CANTalon.h>
#include <Timer.h>
#include <wpilib.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <algorithm>

using namespace std;

const double ROTATIONS_FROM_MIN_ANGLE_TO_CENTER = 0.37; // How far the motor rotates to turn the wheel to center
const unsigned int MICROSECONDS_FOR_CALIBRATION_STEP = 3000000; // Microseconds to wait between movements
const double POSITION_LIMIT_ROTATIONS = 0.15; // How far in either direction the motor is allowed to rotate

class Robot: public frc::SampleRobot {
public:
	void OperatorControl() {
		// Turn off the relays
		SetRelay(communicationRelay, false);
		SetRelay(recordingRelay, false);
		SetRelay(autoDriveRelay, false);
		SetRelay(calibrationRelay, false);

		// Initialize the Talon SRX
		CANTalon *talon = new CANTalon(0);
		talon->SetFeedbackDevice(CANTalon::QuadEncoder); // Relative encoder input
		talon->ConfigEncoderCodesPerRev(10240); // One rotation, after the 10:1 gearbox
		talon->SetSensorDirection(true);
		talon->ConfigNominalOutputVoltage(0, 0);
		talon->SetAllowableClosedLoopErr(0);
		talon->SetControlMode(CANSpeedController::kPosition); // Use integrated PID
		talon->SetF(0);
		talon->SetP(0.1);
		talon->SetI(0);
		talon->SetD(0);

		if (!calibrated) {
			// Center the steering wheel, then disable the motor (but not the encoder)
			talon->ConfigPeakOutputVoltage(4.5, -4.5);
			calibrated = CalibrateSteeringWheel(talon);
			talon->ConfigPeakOutputVoltage(0, 0);
		}

		// Enable an indicator relay if calibration succeeded
		SetRelay(calibrationRelay, calibrated);

		while (IsEnabled()) {
			cout << "Position: " << talon->GetPosition() << endl;
			cout << "EncPosition: " << talon->GetEncPosition() << endl;
			UpdatePositionFile(talon->GetPosition());

			// Get a boolean saying whether the steering angle should be used, followed by the steering angle
			auto values = GetSteeringAngleFromJetsonAndSetRelays();
			if (get<0>(values)) {
				cout << "setting crap" << endl;
				double steeringAngle = get<1>(values); // Get the steering angle
				double limitedSteeringAngle = LimitRotation(steeringAngle); // Limit it to avoid damage to the hardware
				talon->ConfigPeakOutputVoltage(5.5, -5.5); // Turn on the voltage to the motor
				talon->Set(limitedSteeringAngle); // Turn the steering wheel
			} else {
				talon->ConfigPeakOutputVoltage(0, 0); // Disable all voltage to the motor
			}
		}
	}

private:
// Initialize the relays connected to the LEDs
	Relay *recordingRelay = new Relay(0);
	Relay *communicationRelay = new Relay(1);
	Relay *autoDriveRelay = new Relay(2);
	Relay *calibrationRelay = new Relay(3);

// Counter that ensures the relay doesn't flip on and off with every frame
	const unsigned int jetsonInactiveTimeOut = 20;
	unsigned int jetsonInactiveTimer = 0;

	bool calibrated = false; // Makes sure we only calibrate once

	bool CalibrateSteeringWheel(CANTalon *talon) {
		// It is assumed that the wheel starts turned to the maximum in the negative direction
		talon->SetPosition(ROTATIONS_FROM_MIN_ANGLE_TO_CENTER); // Orient the steering system so that zero lies in the center
		double almostMaxAngle = 0.9 * ROTATIONS_FROM_MIN_ANGLE_TO_CENTER;
		talon->Set(almostMaxAngle); // Turn almost to the maximum in the positive direction
		usleep(MICROSECONDS_FOR_CALIBRATION_STEP); // Wait for the movement to complete
		if (abs(talon->Get() - almostMaxAngle) > 0.03) // If we are too far off the desired limit angle
			return false; // Return failure
		talon->Set(0); // Turn back to the center
		usleep(MICROSECONDS_FOR_CALIBRATION_STEP); // Wait for the wheel to center before continuing
		return true;
	}

	double LimitRotation(double position) {
		// Limit a rotation value to an absolute value less than or equal to POSITION_LIMIT_ROTATIONS
		double actualPosition;
		if (position > POSITION_LIMIT_ROTATIONS)
			actualPosition = POSITION_LIMIT_ROTATIONS;
		else if (position < -POSITION_LIMIT_ROTATIONS)
			actualPosition = -POSITION_LIMIT_ROTATIONS;
		else
			actualPosition = position;
		return actualPosition;
	}

	void UpdatePositionFile(double talonPosition) {
		// Save the current motor position prefixed with "out" to a temp file, to be read by the Jetson
		ofstream encValFile;
		encValFile.open("/home/lvuser/temp.encval", ios::out | ios::app);
		encValFile << "out" << talonPosition << "\r\n";
		encValFile.close();

		// Overwrite the old file with the current temp file
		system("mv /home/lvuser/temp.encval /home/lvuser/latest.encval");
	}

	tuple<bool, double> GetSteeringAngleFromJetsonAndSetRelays() {
		// Values for return
		bool jetsonActive = false; // Is the Jetson alive and working?
		bool recordingEnabled = false; // Is the Jetson recording encoder values?
		bool autoDrive = false; // Is the Jetson in autonomous driving mode?
		double steeringAngle; // What is the desired steering angle (for auto drive)?

		// Static values that keep track of the values returned from the Jetson the last time communication succeeded.
		static bool pastRecordingEnabled = false;
		static bool pastAutoDrive = false;
		static double pastSteeringAngle = false;

		try {
			// Attempt to open the file
			ifstream inFile;
			inFile.open("/home/lvuser/values.txt");
			if (!inFile.good()) {
				throw runtime_error("Data file does not exist"); // File has not been replaced, fail into the catch block
			}
			string values[3];

			// Place lines of file into array
			string line;
			for (int i = 0; getline(inFile, line, ' '); i++) {
				cout << line << endl;
				values[i] = line;
			}

			system("rm /home/lvuser/values.txt"); // Delete the file, the Jetson should soon replace it with new data

			// Parse proper values from strings
			recordingEnabled = bool(stoi(values[0]));
			cout << "recording enabled: " << recordingEnabled << endl;
			autoDrive = bool(stoi(values[1]));
			steeringAngle = stof(values[2]);

			jetsonActive = true; // If the file can be read and parsed, the Jetson is active

			cout << "File successfully parsed" << endl;
		} catch (...) {
			// If there are errors, we print an error message
			// This can happen when we have deleted the file and it has not yet been replaced
			cout << "File error caught" << endl;
		}

		if (jetsonActive) { // If the Jetson is communicating with us
			// Turn on the applicable relays and reset the timer
			pastRecordingEnabled = recordingEnabled;
			pastAutoDrive = autoDrive;
			pastSteeringAngle = steeringAngle;
			SetRelay(communicationRelay, true);
			jetsonInactiveTimer = 0; // Reset the counter
		} else {
			jetsonInactiveTimer++; // Increment the timer
			if (jetsonInactiveTimer > jetsonInactiveTimeOut) { // If we have tried to read the file jetsonInactiveTimeOut times without success
				SetRelay(communicationRelay, false); // Turn off the jetsonActiveRelay LED
				pastRecordingEnabled = false;
				pastAutoDrive = false;
			}
		}

		// Set the relays
		SetRelay(recordingRelay, pastRecordingEnabled);
		SetRelay(autoDriveRelay, pastAutoDrive);

		return make_tuple(pastAutoDrive, pastSteeringAngle);
	}

	void SetRelay(Relay *relay, bool on) {
		// Turn an arbitrary relay on or off, inverted because kOn is the default
		auto relayValue = on ? Relay::Value::kOn : Relay::Value::kOff;
		relay->Set(relayValue);
	}
};

START_ROBOT_CLASS(Robot)
