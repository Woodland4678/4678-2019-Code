// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/LineUpToRocket.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

LineUpToRocket::LineUpToRocket(int side): frc::Command() {
    m_side = side;

        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drivetrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void LineUpToRocket::Initialize() {
    Robot::drivetrain->initAutoScore();
    done = false;
}

// Called repeatedly when this Command is scheduled to run
void LineUpToRocket::Execute() {
    if((!Robot::oi->getdriver()->GetRawButton(2))&&(!Robot::oi->getdriver()->GetRawButton(3))&&(!Robot::oi->getdriver()->GetRawButton(4)))
        {
        done = true;
        return;
        }
    //if(Robot::oi->getdriver()->GetRawButton(2))
    //    done = Robot::climber->lock();
    //else
    //    done = Robot::climber->unlock();
    done = Robot::drivetrain->autoScore();
}

// Make this return true when this Command no longer needs to run execute()
bool LineUpToRocket::IsFinished() {
    return done;
}

// Called once after isFinished returns true
void LineUpToRocket::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LineUpToRocket::Interrupted() {

}
