#include <Joystick.h>
#include <SampleRobot.h>
#include <CANTalon.h>
#include <Timer.h>
#include <wpilib.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>

#define TRAINING_MODE false

using namespace std;


class Robot: public frc::SampleRobot {
public:
	void OperatorControl() {
		CANTalon *talon = new CANTalon(0);
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
			talon->Set(0);
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
