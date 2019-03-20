// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "OI.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/Climb.h"
#include "Commands/DriveRobot.h"
#include "Commands/GoToBall.h"
#include "Commands/IntakeControl.h"
#include "Commands/LineUpToBay.h"
#include "Commands/LineUpToRocket.h"
#include "Commands/MoveArm.h"
#include "Commands/SwapGripper.h"
#include "Commands/Spin180.h"
#include "Commands/HatchClampControl.h"


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
    // Process operator interface input here.
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    operator1.reset(new frc::Joystick(1));
    
    operatorBtn1_X_Carry.reset(new frc::JoystickButton(operator1.get(), 1));
    operatorBtn1_X_Carry->WhenPressed(new MoveArm(1));
    operatorBtn4_Y_RocketHighPlacement.reset(new frc::JoystickButton(operator1.get(), 4));
    operatorBtn4_Y_RocketHighPlacement->WhenPressed(new MoveArm(4));
    operatorBtn3_B_RocketMediumPlacement.reset(new frc::JoystickButton(operator1.get(), 3));
    operatorBtn3_B_RocketMediumPlacement->WhenPressed(new MoveArm(3));
    operatorBtn2_A_RocketLowPlacement.reset(new frc::JoystickButton(operator1.get(), 2));
    operatorBtn2_A_RocketLowPlacement->WhenPressed(new MoveArm(2));
    operatorBtn10_Start_CargoShipPlacement.reset(new frc::JoystickButton(operator1.get(), 10));
    operatorBtn10_Start_CargoShipPlacement->WhenPressed(new MoveArm(10));
    operatorBtn9_BACK_SwapGripper.reset(new frc::JoystickButton(operator1.get(), 9));
    operatorBtn9_BACK_SwapGripper->WhenPressed(new SwapGripper());
    operatorBtn_8_RT_GroundIntake.reset(new frc::JoystickButton(operator1.get(), 8));
    operatorBtn_8_RT_GroundIntake->WhenPressed(new MoveArm(6));
    operatorBtn6_RB_HPCargoIntake.reset(new frc::JoystickButton(operator1.get(), 6));
    operatorBtn6_RB_HPCargoIntake->WhenPressed(new MoveArm(8));
    operatorBtn5_LB_HPHatchIntake.reset(new frc::JoystickButton(operator1.get(), 7));
    operatorBtn5_LB_HPHatchIntake->WhenPressed(new MoveArm(7));
    
    driver.reset(new frc::Joystick(0));
    //driverBtn7_LT_Spin180.reset(new frc::JoystickButton(driver.get(), 7));
    //driverBtn7_LT_Spin180->WhenPressed(new Spin180());
    driverBtn8_RT_GoToBall.reset(new frc::JoystickButton(driver.get(), 8));
    driverBtn8_RT_GoToBall->WhileHeld(new GoToBall());
    driverBtn1_X_LineUpToBay.reset(new frc::JoystickButton(driver.get(), 1));
    driverBtn1_X_LineUpToBay->WhenPressed(new MoveArm(12));
    driverBtn6_RB_Intake.reset(new frc::JoystickButton(driver.get(), 6));
    driverBtn6_RB_Intake->WhenPressed(new IntakeControl());
    driverBtn5_LB_Release.reset(new frc::JoystickButton(driver.get(), 5));
    driverBtn5_LB_Release->WhileHeld(new HatchClampControl());
    driverBtn10_START_ClimbMedium.reset(new frc::JoystickButton(driver.get(), 10));
    driverBtn10_START_ClimbMedium->WhenPressed(new Climb(0));
    driverBtn9_BACK_ClimbHigh.reset(new frc::JoystickButton(driver.get(), 9));
    driverBtn9_BACK_ClimbHigh->WhenPressed(new Climb(1));
    driverBtn4_Y_LineUpToRocketFar.reset(new frc::JoystickButton(driver.get(), 4));
    driverBtn4_Y_LineUpToRocketFar->WhenPressed(new LineUpToRocket(2));
    driverBtn3_B_LineUpToRocketMiddle.reset(new frc::JoystickButton(driver.get(), 3));
    driverBtn3_B_LineUpToRocketMiddle->WhenPressed(new LineUpToBay());
    driverBtn2_A_LineUpToRocketClose.reset(new frc::JoystickButton(driver.get(), 2));
    driverBtn2_A_LineUpToRocketClose->WhenPressed(new LineUpToRocket(0));

    // SmartDashboard Buttons
    //frc::SmartDashboard::PutData("GoToBall", new GoToBall());
    frc::SmartDashboard::PutData("Move Wrist", new MoveArm(0));
    frc::SmartDashboard::PutData("Move Elbow", new MoveArm(1));
    frc::SmartDashboard::PutData("Move Shoulder", new MoveArm(2));
    frc::SmartDashboard::PutData("Set PID", new MoveArm(3));
    frc::SmartDashboard::PutData("Position 1", new MoveArm(4));
    frc::SmartDashboard::PutData("Position 2", new MoveArm(5));
    frc::SmartDashboard::PutData("SwapGripper", new SwapGripper());
    frc::SmartDashboard::PutData("ClimbHere", new Climb(1));
    //frc::SmartDashboard::PutData("LineUpToBay", new LineUpToBay());
    //frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    autoSwitch.reset(new frc::Joystick(2));
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<frc::Joystick> OI::getdriver() {
   return driver;
}

std::shared_ptr<frc::Joystick> OI::getoperator1() {
   return operator1;
}


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
std::shared_ptr<frc::Joystick> OI::getAutoSwitch() {
	return autoSwitch;
}
   	// If we want to read the POV, the following works in any 
    // class.  
    // Returns -1 if nothing
	// pressed and then values of 0, 90, 180, 270 when pressed.
	// It does report the 45 degree angles as well but you won't be able
	// to get one of those without first hitting a 0,90,180 or 270
	// WARNING:  An unplugged joystick will cause GetPOV() to return
	// a 0 instead of the -1 so be careful what we connect to 
	// the 0 degree reading of the POV.
	// printf("Operator POV=%d\n\r",Robot::oi->getoperator1()->GetPOV());

