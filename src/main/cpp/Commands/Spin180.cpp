// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Spin180.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

Spin180::Spin180(): frc::Command() {
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drivetrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void Spin180::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void Spin180::Execute() {
    Robot::drivetrain->setLeftMotor(0.5);
    Robot::drivetrain->setRightMotor(0.5);
    spinTime++;
}

// Make this return true when this Command no longer needs to run execute()
bool Spin180::IsFinished() {
    if(spinTime > 100){
        return true;    
    }
    else{
        Robot::drivetrain->setLeftMotor(0);
        Robot::drivetrain->setRightMotor(0);
        return false;
    }
}

// Called once after isFinished returns true
void Spin180::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Spin180::Interrupted() {

}
