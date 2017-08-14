#include <Joystick.h>
#include <SampleRobot.h>
#include <CANTalon.h>
#include <Timer.h>
#include <wpilib.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>

#define TRAINING_MODE true

using namespace std;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as
 * range from -1 to 1 making it easy to work together. The program also delays a
 * short time in the loop to allow other threads to run. This is generally a
 * good idea, especially since the joystick values are only transmitted from the
 * Driver Station once every 20ms.
 */


class Robot: public frc::SampleRobot {
public:

	/**
	 * Runs the motor from the output of a Joystick.
	 */
	void OperatorControl() {
		CANTalon *talon = new CANTalon(0);
		talon->SetEncPosition(0);
		talon->SetFeedbackDevice(CANTalon::QuadEncoder);
		talon->ConfigEncoderCodesPerRev(1024);
		talon->SetSensorDirection(true);
		talon->ConfigNominalOutputVoltage(0, 0);
		talon->SetAllowableClosedLoopErr(0);
		talon->SetControlMode(CANSpeedController::kPosition);
		talon->SetF(0.0);
		talon->SetP(0.1);
		talon->SetI(0.001);
		talon->SetD(0.0);

		if (TRAINING_MODE)
			talon->ConfigPeakOutputVoltage(0, 0);
		else
			talon->ConfigPeakOutputVoltage(12, -12);

		while (IsOperatorControl() && IsEnabled()) {
			talon->Set(2.0);
			cout << "Position: " << talon->GetPosition() << endl;
			cout << "EncPosition: " << talon->GetEncPosition() << endl;
			ofstream encValFile;
			encValFile.open("/home/lvuser/temp.encval", ios::out | ios::app);
			encValFile << "out" << talon->GetPosition();
			encValFile.close();
			system("rm /home/lvuser/latest.encval");
			system("mv /home/lvuser/temp.encval /home/lvuser/latest.encval");
			frc::Wait(kUpdatePeriod);  // Wait 5ms for the next update.
		}
	}

private:
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)
