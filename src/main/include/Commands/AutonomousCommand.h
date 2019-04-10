// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef AUTONOMOUS_COMMAND_H
#define AUTONOMOUS_COMMAND_H


#include "frc/commands/Subsystem.h"
#include "Robot.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class AutonomousCommand: public frc::Command {
public:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
	AutonomousCommand();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
	bool getSecondHatch();
	//bool driverControl();
	bool scoreFirstHatch();
	bool scoreFarRocket();
private:
	int autoMode = 0;
	int autoSide = 0;
	int autoState = 0;
	int farRocketState = 0;
	int autoStateSecondHatch = 0;
	double secondHatchTurnAmount = 0;
	int cnt = 0;
	int autonomousSelection = 0;
	bool armMovement0 = false;
	bool armMovement1 = false;
	bool armMovement2 = false;
	bool armMovement3 = false;

	int autoScore = 0;
	bool done = false;
	double initialGyroValue = 0;

	double amountToTurn = 0;

	double joyX;
	double joyY;

	int leftCmStraightBack;
	int rightCmStraightBack;

	int leftArc = 0;
	int rightArc = 0;

	int straightBackRampUpDistance;

	double secondHatchAmountToTurn = 0;

	double amountToTurnAfterRocket = 0;
	int rightArcAfterRocketOne = 0;
	int leftArcAfterRocketOne = 0;
	int rightArcAfterRocketTwo = 0;
	int leftArcAfterRocketTwo = 0;
	//enum autoState {driveFoward = 1, turnToCargoShip = 2, scoreHatch = 3, backAfterScore = 4};
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES
};

#endif
