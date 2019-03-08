// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Climb.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

Climb::Climb(int climbLevel): frc::Command() {
    m_climbLevel = climbLevel;
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	//Requires(Robot::climber.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

    done = false;
	done2 = false;

}
// Called just before this Command runs the first time
void Climb::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
	//done2 = Robot::manipulatorArm->moveToXY(33.0,34.0,-280.0,20.0);
	//if(done2)
	//	done = Robot::manipulatorArm->moveToXY(33.0,34.0,-280.0,20.0);
    //printf("\nHere: %i",m_climbLevel);
    if (m_climbLevel == true) {
		done = Robot::manipulatorArm->moveToXY(25.0,50.0,-280.0,20.0);
        //Robot::climber->moveTo();
    }
    else {
		done = Robot::manipulatorArm->moveToXY(33.0,34.0,-280.0,20.0);
        //Robot::climber->reset();
    }
}

// Make this return true when this Command no longer needs to run execute()
bool Climb::IsFinished() {
    return done;
}
// Called once after isFinished returns true
void Climb::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Climb::Interrupted() {

}
