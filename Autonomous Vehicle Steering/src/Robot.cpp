#include <Joystick.h>
#include <SampleRobot.h>
#include <CANTalon.h>
#include <Timer.h>
#include <wpilib.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>

using namespace std;

const bool TRAINING_MODE = true;
const double ROTATIONS_FROM_MIN_ANGLE_TO_CENTER = 0.3; // How far the motor rotates to turn the wheel to center
const unsigned int MICROSECONDS_FOR_CALIBRATION_STEP = 3000000;

class Robot: public frc::SampleRobot {
public:
	void OperatorControl() {
		// Initialize the Talon SRX
		CANTalon *talon = new CANTalon(0);
		talon->SetFeedbackDevice(CANTalon::QuadEncoder); // Relative encoder input
		talon->ConfigEncoderCodesPerRev(10240); // One rotation, after the 10:1 gearbox
		talon->SetSensorDirection(true);
		talon->ConfigNominalOutputVoltage(0, 0);
		talon->ConfigPeakOutputVoltage(12, -12);
		talon->SetAllowableClosedLoopErr(0);
		talon->SetControlMode(CANSpeedController::kPosition); // Use integrated PID
		talon->SetF(0);
		talon->SetP(0.1);
		talon->SetI(0);
		talon->SetD(0);

		if (TRAINING_MODE) {
			// Center the steering wheel, then disable the motor (but not the encoder)
			talon->ConfigPeakOutputVoltage(4.5, -4.5);
			bool calibrationSucceeded = CalibrateSteeringWheel(talon);
			cout << calibrationSucceeded << endl;
			talon->ConfigPeakOutputVoltage(0, 0);
		}

		while (IsEnabled()) {
			cout << "Position: " << talon->GetPosition() << endl;
			cout << "EncPosition: " << talon->GetEncPosition() << endl;
			UpdatePositionFile(talon->GetPosition());
			ReadValuesFromJetsonAndSetRelays();
//			auto values = ReadValuesFromJetsonAndSetRelays();
//			bool jetsonEnabled = get<0>(values);
//			double steeringAngle = get<3>(values);
//			if (jetsonEnabled)
//				talon->Set(steeringAngle / 100);
		}
	}

private:
	// Initialize the relays connected to the LEDs
	Relay *jetsonActiveRelay = new Relay(1);
	Relay *recordingEnabledRelay = new Relay(2);

	// Counter that ensures the relay doesn't flip on and off with every frame
	const unsigned int jetsonInactiveTimeOut = 20;
	unsigned int jetsonInactiveTimer = 0;

	bool CalibrateSteeringWheel(CANTalon *talon) {
		// It is assumed that the wheel starts turned to the maximum in the negative direction
		talon->SetPosition(ROTATIONS_FROM_MIN_ANGLE_TO_CENTER); // Orient the steering system so that zero lies in the center
		double almostMaxAngle = 0.9 * ROTATIONS_FROM_MIN_ANGLE_TO_CENTER;
		usleep(MICROSECONDS_FOR_CALIBRATION_STEP);
		talon->Set(almostMaxAngle); // Turn almost to the maximum in the positive direction

		if (abs(talon->Get() - almostMaxAngle) > 0.03) // If we are too far off the desired limit angle
			return false; // Return failure

		usleep(MICROSECONDS_FOR_CALIBRATION_STEP); // Wait for the movement to complete
		talon->Set(0); // Turn back to the center
		usleep(MICROSECONDS_FOR_CALIBRATION_STEP); // Wait for the wheel to center before continuing
		return true;
	}

	void UpdatePositionFile(double talonPosition) {
		// Save the current motor position prefixed with "out" to a temp file, to be read by the Jetson
		ofstream encValFile;
		encValFile.open("/home/lvuser/temp.encval", ios::out | ios::app);
		encValFile << "out" << talonPosition;
		encValFile.close();

		// Delete the old file and replace it with the current temp file
		system("rm /home/lvuser/latest.encval");
		system("mv /home/lvuser/temp.encval /home/lvuser/latest.encval");
	}

	tuple<bool, bool, bool, double> ReadValuesFromJetsonAndSetRelays() {
		// Values for return
		bool jetsonActive; // Is the Jetson alive and working?
		bool recordingEnabled; // Is the Jetson recording encoder values?
		bool autoDrive; // Is the Jetson in autonomous driving mode?
		double steeringAngle; // What is the desired steering angle (for auto drive)?


		try {
			// Attempt to open the file
			ifstream inFile;
			inFile.open("/home/lvuser/values.txt");
			if (!inFile.good()) {
				throw runtime_error("Data file does not exist"); // File has not been replaced, fail into the catch block
			}
			const char *values[3];

			// Place lines of file into array
			string line;
			for (int i = 0; getline(inFile, line); i++) {
				values[i] = line.c_str();
			}

			system("rm /home/lvuser/values.txt"); // Delete the file, the Jetson should soon replace it with new data

			// Parse proper values from strings
			recordingEnabled = bool(atoi(values[0]));
			autoDrive = bool(atoi(values[1]));
			steeringAngle = atof(values[2]);

			jetsonActive = true; // If the file can be read and parsed, the Jetson is active

			cout << "File successfully parsed" << endl;
		}
		catch (...) {
			// If there are errors, we say the Jetson is inactive and leave garbage values for everything else
			// This can happen when we have deleted the file and it has not yet been replaced
			jetsonActive = false;
			cout << "File error caught" << endl;
		}

		// Set the relays according to the values for return
		SetRelay(recordingEnabledRelay, recordingEnabled);

		if (jetsonActive) {
			SetRelay(jetsonActiveRelay, true);
			jetsonInactiveTimer = 0; // Reset the counter
		} else {
			jetsonInactiveTimer++; // Increment the timer
			if (jetsonInactiveTimer > jetsonInactiveTimeOut) // If we have tried to read the file jetsonInactiveTimeOut times without success
				SetRelay(jetsonActiveRelay, false); // Turn off the jetsonActiveRelay LED
		}

		return make_tuple(jetsonActive, recordingEnabled, autoDrive, steeringAngle);
	}

	void SetRelay(Relay *relay, bool on) {
		// Turn an arbitrary relay on or off, inverted because kOn is the default
		auto relayValue = on ? Relay::Value::kOn : Relay::Value::kOff;
		relay->Set(relayValue);
	}
};

START_ROBOT_CLASS(Robot)
