// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef OI_H
#define OI_H

#include "frc/WPILib.h"

class OI {
private:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<frc::Joystick> operator1;
	std::shared_ptr<frc::JoystickButton> operatorBtn1_X_Carry;
	std::shared_ptr<frc::JoystickButton> operatorBtn4_Y_RocketHighPlacement;
	std::shared_ptr<frc::JoystickButton> operatorBtn3_B_RocketMediumPlacement;
	std::shared_ptr<frc::JoystickButton> operatorBtn2_A_RocketLowPlacement;
	std::shared_ptr<frc::JoystickButton> operatorBtn10_Start_CargoShipPlacement;
	std::shared_ptr<frc::JoystickButton> operatorBtn9_BACK_SwapGripper;
	std::shared_ptr<frc::JoystickButton> operatorBtn_8_RT_GroundIntake;
	std::shared_ptr<frc::JoystickButton> operatorBtn6_RB_HPCargoIntake;
	std::shared_ptr<frc::JoystickButton> operatorBtn5_LB_HPHatchIntake;
	std::shared_ptr<frc::Joystick> driver;
	std::shared_ptr<frc::JoystickButton> driverBtn7_LT_Spin180;
	std::shared_ptr<frc::JoystickButton> driverBtn8_RT_GoToBall;
	std::shared_ptr<frc::JoystickButton> driverBtn1_X_LineUpToBay;
	std::shared_ptr<frc::JoystickButton> driverBtn6_RB_Intake;
	std::shared_ptr<frc::JoystickButton> driverBtn5_LB_Release;
	std::shared_ptr<frc::JoystickButton> driverBtn10_START_ClimbMedium;
	std::shared_ptr<frc::JoystickButton> driverBtn9_BACK_ClimbHigh;
	std::shared_ptr<frc::JoystickButton> driverBtn4_Y_LineUpToRocketFar;
	std::shared_ptr<frc::JoystickButton> driverBtn3_B_LineUpToRocketMiddle;
	std::shared_ptr<frc::JoystickButton> driverBtn2_A_LineUpToRocketClose;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<frc::Joystick> autoSwitch;
public:
	OI();

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES

	std::shared_ptr<frc::Joystick> getdriver();
	std::shared_ptr<frc::Joystick> getoperator1();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES
	std::shared_ptr<frc::Joystick> getAutoSwitch();
};

#endif
