// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#ifndef MANIPULATORARM_H
#define MANIPULATORARM_H

#include "frc/commands/Subsystem.h"
#include "frc/WPILib.h"
#include "ctre/phoenix.h"
#include "rev/CANSparkMax.h"
#include <fstream>
#include "Subsystems/Arm/ArmSegment.h"
#include "Subsystems/Arm/Movement.h"

/**
 * @author ExampleAuthor
 */
class ManipulatorArm: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<rev::CANSparkMax> waist;
	std::shared_ptr<rev::CANSparkMax> shoulder;
	std::shared_ptr<rev::CANSparkMax> elbow;
	std::shared_ptr<WPI_TalonSRX> wrist;
	std::shared_ptr<WPI_VictorSPX> intakeWheels;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<frc::AnalogInput> waistPot;
	std::shared_ptr<frc::AnalogInput> shoulderPot;
	std::shared_ptr<frc::AnalogInput> elbowPot;
	std::shared_ptr<frc::AnalogInput> wristPot;
	std::shared_ptr<frc::Servo> hatchServo;

	std::shared_ptr<frc::DigitalInput> cargoSensor;
	std::shared_ptr<frc::DigitalInput> hatchSensor;
	
	ArmSegment *m_Segs[4];
	ArmMotion *m_Moves[5];
	bool m_Skip[4][5];
	int m_readCnt[4];
	bool m_SkipFinal[4];

	int m_MotionCase = 0;
	int m_SwapCase = 0;

	std::ofstream logfile; // stream for writing to file
	char buf[256]; // Buffer for writing data

	int m_Positions[32][3];
	int m_PosCnt = 0;

	int m_IntakeMode = 1; // 0=empty, 1=hatch, 2=cargo.
	bool m_InCargoPosition = false; // Set to true when we might be taking in cargo.
	bool m_InHatchPosition = false; // Set to trun when we might be taking in a hatch.
	int m_SpawTarget = 0;

	bool doneS = false;
	bool doneE = false;
	bool doneW = false;
	bool doneA = false;

	double m_VectorWheelsX = 0;
	double m_VectorWheelsY = 0;
	double m_VectorBottomX = 0;
	double m_VectorBottomY = 0;
	double m_VectorHatchX = 0;
	double m_VectorHatchY = 0;


public:
// constructor
	ManipulatorArm();

// functions
	void InitDefaultCommand() override;
	void Periodic() override;

	bool Init();
	bool moveTo(int pos);
	bool moveToNew(int pos);
	bool moveToXY(double x, double y, double wristAbsTarget, double waistTarget, double inchesPerSecond);

	void addPosition(int shoulderAngle, int elbowAngle, int wristAngle);
	bool swapIntakes();
	bool moveWaist(double angle);
	bool isHatchMode();
	void addPoint(int index,double shoulderAngle, double elbowAngle, double wristAngle1, double wristAngle2);
	void setShoulderandElbow(double shoulderAngle, double elbowAngle);
	void InverseKinematics(double x, double y);
	void incAbsAngleTarget();
	void decAbsAngleTarget();
	void setAbsAngleWrist(double angle);
	void setAbsAngleTargetByType();
	void setInCargoPosition();
	void clearInCargoPosition();
	bool getInCargoPosition();
	bool ifCargo();
	bool ifHatch();
	void setInHatchPosition();
	void clearInHatchPosition();
	bool getInHatchPosition();
	void setIntakeMode(int mode);
	int getIntakeMode();
	void fineMotion();

	void calculateWristVectors();

	void InitCalibrate();
	bool Calibrate();

	void disableWrist();
	double getOffset(int axis);
	double getWaistAngle();


	//Intakes
	void intakeWheelsSpin(double power);
	void intakeWheelsOff();

	void releaseHatch();
	void grabHatch();

	void openLog();
	void CloseLog();
	
	int m_CurrentPosition = 0;
	int m_TargetPosition = 0;
	int m_PrevPosition = 0;
	int m_StartIndex = 3; // Where we are when a move begins on array 0 of sequence of moves
	int m_TargetIndex; 
	double m_StartX,m_StartY;
	double m_TargetX,m_TargetY;
	double m_StartW;
	double m_TargetWaist = 0.0; // Target Waist Angle
	double m_XinchesPerSecond,m_YinchesPerSecond;
	double m_InvKinShoulderAngle,m_InvKinElbowAngle;
	int m_Cycles,m_CycleCount;
	bool m_FineLimitHit = false;

	double getEndEffectorX();
	double getEndEffectorY();
	bool m_inCalibration;
	int m_CalibrationState,m_CalCnt,m_Wristbits,m_CalCnt2,m_ShRef;
	double deltaWR, prevWr;

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
};

#endif
