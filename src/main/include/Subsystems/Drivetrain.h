// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H
#include "frc/commands/Subsystem.h"
#include "frc/WPILib.h"
#include "rev/CANSparkMax.h"


class Drivetrain: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<rev::CANSparkMax> leftMaster;
	std::shared_ptr<rev::CANSparkMax> leftSlaveOne;
	std::shared_ptr<rev::CANSparkMax> leftSlaveTwo;
	std::shared_ptr<rev::CANSparkMax> rightMaster;
	std::shared_ptr<rev::CANSparkMax> rightSlaveOne;
	std::shared_ptr<rev::CANSparkMax> rightSlaveTwo;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	
public:
	Drivetrain();
	void InitDefaultCommand() override;
	void Periodic() override;
	void configMotors();
	void setLeftMotor(double power);
	void setRightMotor(double power);
	double getLeftEncoder();
	double getRightEncoder();

	double  percentDone()  const;
	bool goToDistance(double rightCentimeters, double leftCentimeters, double power, int rampUpDistance,
			int rampDownDistance, double startingPower, double endingPower);
	void resetGoToDistanceState();
	bool GyroTurn(double current, double turnAmount,double p, double i, double d);
	
	void setToCoast();
	void setToBrake();

	void initAutoScore();
	bool autoScore();
	
	//void test(double power);


	void joystickDriveCalculator();
	double const deadRadiusSquared = 0.05*0.05;		// this is the radius of the deadzone squared
	double const turningFxnCoefficient = 0.7;		// this is the scaller that is used to modify turning

	int goToDistanceState = 0;
	double startingLeftDistance = 0;
	double startingRightDistance = 0;
	// go-to distance variables
	double targetLeft;
	double targetRight;
	double currentLeft;
	double currentRight;
	double currentLeftCentimeters;
	double currentRightCentimeters;
	double leftPercentThere;
	double rightPercentThere;
	double leftMotorMultiplier;
	double rightMotorMultiplier;
	double encoderClicksPerCentimeter = 0.2183; //there were 10.45 clicks per full rotation, and circumference of wheels are 47.879 cm
	double GO_TO_DISTANCE_CORRECTION_SPEED = 5.0;
	double powerOffset;
	double leftPower = 0;
	double rightPower = 0;

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

	int as_m_case = 0;
	int as_m_SubCase = 0;
	bool as_move1 = false;
	int as_mode = 0;
	int as_cnt = 0;
	double as_angle;
	double as_distance = 0;
	double as_distEnd = 0;
	double as_distWaist = 0;
};

#endif