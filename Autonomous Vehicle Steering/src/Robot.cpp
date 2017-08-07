#include <Joystick.h>
#include <SampleRobot.h>
#include <CANTalon.h>
#include <Timer.h>
#include <wpilib.h>
#include <stdlib.h>
#include <stdio.h>

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
		talon->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		talon->SetSensorDirection(false);
		talon->ConfigNominalOutputVoltage(0, 0);
		talon->ConfigPeakOutputVoltage(12, 12);
		talon->SelectProfileSlot(0);
		talon->SetAllowableClosedLoopErr(0);
		talon->SetF(0.0);
		talon->SetP(0.1);
		talon->SetI(0.0);
		talon->SetD(0.0);
		talon->SetControlMode(CANSpeedController::ControlMode::kPosition);
		//talon->Set(0.5);
		talon->SetEncPosition(1000);

		while (IsOperatorControl() && IsEnabled()) {
			/* Set the motor controller's output. This takes a number from -1
			 * (100% speed in reverse) to +1 (100% speed forwards).
			 */

			talon->SetControlMode(CANSpeedController::ControlMode::kPosition);
			std::cout << talon->GetEncPosition() << std::endl;

//			if (abs(t->GetEncPosition()) < 300000)
//				t->Set(0.2);
//			else
//				t->Set(0);
			//m_motor.Set(0.1); //m_stick.GetY());

			frc::Wait(kUpdatePeriod);  // Wait 5ms for the next update.
		}
	}

private:
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)
