// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/GoToBall.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

GoToBall::GoToBall(): frc::Command() {
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drivetrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void GoToBall::Initialize() {

    printf("Here\n");
    Robot::lidar->readLidar();
    printf("Here2\n");
}

// Called repeatedly when this Command is scheduled to run
void GoToBall::Execute() {

    if (Robot::lidar->readComplete() == true)  {
        printf("Here3\n");
        Robot::lidar->filterData( false, 60, 300, 50, 1500);
        printf("Here4\n");
        lidattp centrePt = Robot::lidar->findCargo();
        printf("centre point angle: %i", centrePt.angle);
        printf("centre point distance: %i", centrePt.dist);
        printf("time stamp: %i", centrePt.tstamp);
        
        cargoIsFound = true;
    }
}

// Make ifthis return true when this Command no longer needs to run execute()
bool GoToBall::IsFinished() {
    printf("Here5\n");
    if (cargoIsFound) {
        printf("Here6\n");
        return true;
    }
    else {
        return false;
    }
    
}

// Called once after isFinished returns true
void GoToBall::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GoToBall::Interrupted() {}
