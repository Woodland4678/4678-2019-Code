// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef DRIVEROBOT_H
#define DRIVEROBOT_H


#include "frc/commands/Subsystem.h"
#include "Robot.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class DriveRobot: public frc::Command {
public:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
	DriveRobot();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:

	double oldLeftPower = 0.0;
	double oldRightPower = 0.0;
	double maxChange = 0.015;
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES
};

#endif