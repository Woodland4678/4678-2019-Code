// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/ManipulatorArm.h"
#include "Subsystems/Arm/ArmSegment.h"
#include "ctre/phoenix.h"
#include <fstream>
#include "rev/CANSparkMax.h"
#include "Subsystems/Arm/Movement.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "../../include/Commands/MoveArm.h"

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

#define SHOULDER_ANGLE_1    165 // At hard stop
#define SHOULDER_ANGLE_2      0 // Straight Out the front horizontal level
#define SHOULDER_ENCODER_1   2632.96
#define SHOULDER_ENCODER_2    2474.96

#define ELBOW_ANGLE_1     -167 // Hatch clamp roller section more or less resting on arm.
#define ELBOW_ANGLE_2     0 // Straight up from shoulder (measured between the 2 pivot points)
#define ELBOW_ENCODER_2   -14456.875 //-14458.90    -5208
#define ELBOW_ENCODER_1   -14619.5

//#define WRIST_ANGLE_1     0 // Cargo Straight Out
#define WRIST_ANGLE_1     -90 // Hatch Clamp at 90 deg, roller part of claw inwards.
#define WRIST_ANGLE_2     -180 // Hatch Clamp Straight Out.
#define WRIST_ENCODER_1   833
#define WRIST_ENCODER_2   1857 // 1857 - 833 = 1024 (0.25 x 4096 = 1/4 rotation = 90 deg)

#define WAIST_ANGLE_1     -68 // Fully CCW
#define WAIST_ANGLE_2     68 // Fully CW.  0 is straight ahead.
#define WAIST_ENCODER_1   -127.0
#define WAIST_ENCODER_2   0.0


#define SHOULDER_POT_1    2884
#define SHOULDER_POT_2    286

#define ELBOW_POT_1    4
#define ELBOW_POT_2    2481 //2467

//#define WRIST_POT_1    4 // Cargo Intake Straight Out - in dead zone.
#define WRIST_POT_1    905 // Hatch Clamp at 90 deg, roller part of claw inwards.
#define WRIST_POT_2    2500 // Hatch Clamp Straight Out.
// 3770 At 90 deg, roller part of cargo claw outwards.
// 905 At 90 deg the other way.  2500 - 905 = 1595
// 3770 - 2500 = 1270. 

#define WAIST_POT_1    3757 
#define WAIST_POT_2    641 // 

ManipulatorArm::ManipulatorArm() : frc::Subsystem("ManipulatorArm") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    waist.reset(new rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless));
    shoulder.reset(new rev::CANSparkMax(10, rev::CANSparkMax::MotorType::kBrushless));
    elbow.reset(new rev::CANSparkMax(9, rev::CANSparkMax::MotorType::kBrushless));
    wrist.reset(new WPI_TalonSRX(8));
    intakeWheels.reset(new WPI_VictorSPX(11));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	waistPot.reset(new frc::AnalogInput(0));
	shoulderPot.reset(new frc::AnalogInput(1));
	elbowPot.reset(new frc::AnalogInput(2));
	wristPot.reset(new frc::AnalogInput(3));

	cargoSensor.reset(new frc::DigitalInput(0));
	hatchSensor.reset(new frc::DigitalInput(1));

	hatchServo.reset(new frc::Servo(0));
    AddChild("Servo", hatchServo);

    m_Segs[0] = new ArmSegment(shoulder, shoulderPot, 33, NULL, 0, 13.7);
    m_Segs[1] = new ArmSegment(elbow,elbowPot,32, m_Segs[0]);
    m_Segs[2] = new ArmSegment(wrist,wristPot,11, m_Segs[1]);
	m_Segs[3] = new ArmSegment(waist,waistPot,14, NULL);

    m_Moves[0] = new ArmMotion(0,0,10,0);
	m_Moves[1] = new ArmMotion(0,0,10,0);
	m_Moves[2] = new ArmMotion(0,0,10,0);
	m_Moves[3] = new ArmMotion(0,0,10,0);
	m_Moves[4] = new ArmMotion(0,0,10,0);
}

bool ManipulatorArm::Init(){
	//Shoulder
    //m_Segs[0]->setSensorDirection(true);
    //m_Segs[0]->setVoltageLimit(true, 12);
    m_Segs[0]->setPID(0.1,0,0);
	m_Segs[0]->setCurrentPeakLimit(true,40.0);
	// By default, closed loop error is set to 5 - found using the Phoenix Tuner
    //m_Segs[0]->setLimits(2619, 165, -7818, -13);
	m_Segs[0]->setEncoderValues(SHOULDER_ENCODER_1, SHOULDER_ANGLE_1, SHOULDER_ENCODER_2, SHOULDER_ANGLE_2);
	m_Segs[0]->setPotValues(SHOULDER_POT_1, SHOULDER_ANGLE_1, SHOULDER_POT_2, SHOULDER_ANGLE_2);
	m_Segs[0]->initEncoderValues();
	
	//Elbow
    //m_Segs[1]->setSensorDirection(true); 
	//m_Segs[1]->setEncoderValues(14475, 100, -123, -150);
	m_Segs[1]->setCurrentPeakLimit(true,40.0);
	m_Segs[1]->setEncoderValues(ELBOW_ENCODER_1, ELBOW_ANGLE_1, ELBOW_ENCODER_2, ELBOW_ANGLE_2);
	m_Segs[1]->setPotValues(ELBOW_POT_1, ELBOW_ANGLE_1, ELBOW_POT_2, ELBOW_ANGLE_2);
	// By default, closed loop error is set to 5 - found using the Phoenix Tuner
	bool com = m_Segs[1]->initEncoderValues();
    //m_Segs[1]->setVoltageLimit(true, 12);
    m_Segs[1]->setPID(0.075,0,0);

	frc::SmartDashboard::PutNumber("Elbow Init",com);

	//Wrist
    m_Segs[2]->setVoltageLimit(true, 8); // ****** Steve change to 12 from 8 March 2 1:56
	//m_Segs[2]->setEncoderValues(-1919, 180, 60, 0);
	m_Segs[2]->setEncoderValues(WRIST_ENCODER_1, WRIST_ANGLE_1, WRIST_ENCODER_2, WRIST_ANGLE_2);
	m_Segs[2]->setPotValues(WRIST_POT_1, WRIST_ANGLE_1, WRIST_POT_2, WRIST_ANGLE_2);
	// By default, closed loop error is set to 5 - found using the Phoenix Tuner
	// printf("2 CLErr=%d\n\r",m_Segs[2]->getClosedLoopError());
	m_Segs[2]->initEncoderValues();	
    m_Segs[2]->setPID(7,0,0);  // **** Steve changed PID from (1,0,0)
	m_Segs[2]->setAbsAngleTarget(-200); // This is the target (once we're clear to move wrist)
	
	//Waist
	m_Segs[3]->setEncoderValues(WAIST_ENCODER_1, WAIST_ANGLE_1, WAIST_ENCODER_2, WAIST_ANGLE_2);
	m_Segs[3]->setPotValues(WAIST_POT_1, WAIST_ANGLE_1, WAIST_POT_2, WAIST_ANGLE_2);
	m_Segs[3]->initEncoderValues();	
    m_Segs[3]->setPID(0.05,0,0);
	// m_Segs[3]->setVoltageLimit(true, 3);
	m_Segs[3]->setCurrentPeakLimit(true,30.0); // Set current limit for now.
	// By default, closed loop error is set to 5 for talons.  Not sure about Spark Max.

	addPosition(87,-126,-325); 	//0 - Rocket Cargo Low 25.5 inches away from frame ::Updated March 2 10:49
	addPosition(105,-83,-384);	//1 - Rocket Cargo Medium 25.5 inches away from frame ::Updated March 2 10:49
	addPosition(165,-169,-211);	//2 - Rocket Cargo High 6.5 inches away from frame ::Updated March 2 10:49
	addPosition(77,-128,-126);	//3 - Low Hatch 19in / Hatchs 25 inches away from frame ::Updated March 2 1:46
	addPosition(108,-102,-188);	//4 - Rocket Hatch Medium ::Updated March 2 
	addPosition(98,-36,-234);		//5 - Rocket Hatch High 9 inches away from frame ::Updated March 2 11:30
	addPosition(108,-116,-340);	//6 - Cargo Human Station 27 inches away from frame ::Updated March 2 10:49
	addPosition(104,-91,-414);	//7 - Cargo Cargoship 7 inches away from frame ::Updated March 2 10:49
	addPosition(69,-117,-400);	//8 - Climb Medium ::Updated March 2 10:49
	addPosition(83,-109,-433);	//9 - Climb High ::Updated March 2 10:49
	addPosition(165,-167,-211);	//10 - Carry - ****** not really carry position - temporary "rest" position ::Updated March 2 10:49 
	addPosition(84,-145,-354);	//11 - Cargo On Ground 		::Updated March 2 10:49
	addPosition(93,-128,52);	//12 - Home 				- Not Set
	addPosition(85,-106,-57);	//13 - temporary start climb
	addPosition(73,-116,-42); 	//14 - temporary mid climb
	addPosition(46,-116,-44);	//15 - temporary finish climb

    //frc::SmartDashboard::PutNumber("Elbow Slope", m_Segs[2]->getConversionSlope());
    //frc::SmartDashboard::PutNumber("Elbow Intercept", m_Segs[2]->getConversionIntercept());
}

void ManipulatorArm::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        SetDefaultCommand(new MoveArm(0));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void ManipulatorArm::Periodic() {

	frc::SmartDashboard::PutBoolean("Cargo", ifCargo());

    // Put code here to be run every loop

	//Forward Kinematics
	m_Segs[0]->updateAngles();
	
	m_Segs[1]->setPhysicalAttributes(m_Segs[0]->getEndX(),m_Segs[0]->getEndY());
	m_Segs[1]->updateAngles();
	
	m_Segs[2]->setPhysicalAttributes(m_Segs[1]->getEndX(),m_Segs[1]->getEndY());
	m_Segs[2]->updateAngles();

	m_Segs[3]->updateAngles();
	
	if (logfile.is_open())
		{
		sprintf(buf,"A,%f,%f,%f,%f,%f,%f,%i,%f\n",
			m_Segs[3]->getRelAngle(),
			m_Segs[3]->getAbsAngle(),
			m_Segs[3]->getStartX(),
			m_Segs[3]->getStartY(),
			m_Segs[3]->getEndX(),
			m_Segs[3]->getEndY(),
			m_Segs[3]->getPotentiometerReading(),
			m_Segs[3]->getSelectedSensorValue()
		);
		logfile.write(buf,strlen(buf));

		sprintf(buf,"S,%f,%f,%f,%f,%f,%f,%i,%f\n",
			m_Segs[0]->getRelAngle(),
			m_Segs[0]->getAbsAngle(),
			m_Segs[0]->getStartX(),
			m_Segs[0]->getStartY(),
			m_Segs[0]->getEndX(),
			m_Segs[0]->getEndY(),
			m_Segs[0]->getPotentiometerReading(),
			m_Segs[0]->getSelectedSensorValue()
		);
		logfile.write(buf,strlen(buf));

		sprintf(buf,"E,%f,%f,%f,%f,%f,%f,%i,%f\n",
			m_Segs[1]->getRelAngle(),
			m_Segs[1]->getAbsAngle(),
			m_Segs[1]->getStartX(),
			m_Segs[1]->getStartY(),
			m_Segs[1]->getEndX(),
			m_Segs[1]->getEndY(),
			m_Segs[1]->getPotentiometerReading(),
			m_Segs[1]->getSelectedSensorValue()
		);
		logfile.write(buf,strlen(buf));

		sprintf(buf,"W,%f,%f,%f,%f,%f,%f,%i,%f\n",
			m_Segs[2]->getRelAngle(),
			m_Segs[2]->getAbsAngle(),
			m_Segs[2]->getStartX(),
			m_Segs[2]->getStartY(),
			m_Segs[2]->getEndX(),
			m_Segs[2]->getEndY(),
			m_Segs[2]->getPotentiometerReading(),
			m_Segs[2]->getSelectedSensorValue()
		);
		logfile.write(buf,strlen(buf));
		}

    frc::SmartDashboard::PutNumber("Shoulder Angle", m_Segs[0]->getRelAngle());
	frc::SmartDashboard::PutNumber("Wrist Angle", m_Segs[2]->getRelAngle());
	frc::SmartDashboard::PutNumber("Elbow Angle", m_Segs[1]->getRelAngle());
	frc::SmartDashboard::PutNumber("Waist Angle", m_Segs[3]->getRelAngle());

	frc::SmartDashboard::PutNumber("Shoulder ABS Angle", m_Segs[0]->getAbsAngle());
	frc::SmartDashboard::PutNumber("Wrist ABS Angle", m_Segs[2]->getAbsAngle());
	frc::SmartDashboard::PutNumber("Elbow ABS Angle", m_Segs[1]->getAbsAngle());
	
	frc::SmartDashboard::PutNumber("El Start X", m_Segs[0]->getEndX());
	frc::SmartDashboard::PutNumber("El Start Y", m_Segs[0]->getEndY());
	frc::SmartDashboard::PutNumber("Wr Start X", m_Segs[1]->getEndX());
	frc::SmartDashboard::PutNumber("Wr Start Y", m_Segs[1]->getEndY());
	frc::SmartDashboard::PutNumber("End X", m_Segs[2]->getEndX());
	frc::SmartDashboard::PutNumber("End Y", m_Segs[2]->getEndY());

	
	frc::SmartDashboard::PutNumber("Shoulder Encoder", m_Segs[0]->getSelectedSensorValue());
	frc::SmartDashboard::PutNumber("Wrist Encoder", m_Segs[2]->getSelectedSensorValue());
	frc::SmartDashboard::PutNumber("Elbow Encoder", m_Segs[1]->getSelectedSensorValue());
	frc::SmartDashboard::PutNumber("Waist Encoder", m_Segs[3]->getSelectedSensorValue());

	frc::SmartDashboard::PutNumber("Shoulder Pot", m_Segs[0]->getPotentiometerReading());
	frc::SmartDashboard::PutNumber("Wrist Pot", m_Segs[2]->getPotentiometerReading());
	frc::SmartDashboard::PutNumber("Elbow Pot", m_Segs[1]->getPotentiometerReading());
	frc::SmartDashboard::PutNumber("Waist Pot", m_Segs[3]->getPotentiometerReading());
    
	frc::SmartDashboard::PutNumber("Shoulder Amps", m_Segs[0]->getCurrent());
	frc::SmartDashboard::PutNumber("Wrist Amps", m_Segs[2]->getCurrent());
	frc::SmartDashboard::PutNumber("Elbow Amps", m_Segs[1]->getCurrent());
	frc::SmartDashboard::PutNumber("Waist Amps", m_Segs[3]->getCurrent());
    
	if(m_Segs[1]->getRelAngle() > -141)
		frc::SmartDashboard::PutBoolean("Swap Enabled", true);
	else
		frc::SmartDashboard::PutBoolean("Swap Enabled", false);	

	if(m_Segs[0]->getRelAngle() < 116)
		frc::SmartDashboard::PutBoolean("Waist Allowed", true);
	else
		frc::SmartDashboard::PutBoolean("Waist Allowed", false);	

	if(isHatchMode())
		frc::SmartDashboard::PutBoolean("Hatch", true);
	else
		frc::SmartDashboard::PutBoolean("Hatch", false);
    
}

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

void ManipulatorArm::openLog(){
	time_t t=time(0);
	struct tm *now;
	if (!logfile.is_open())
		{
		now = localtime(&t);
		sprintf(buf,"/media/sdc/ARMA%02d%02d%02d%02d%02d%02d.txt",now->tm_year,now->tm_mon,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
		logfile.open(buf,std::ios::out | std::ios::binary);
		}
	else {
		now = localtime(&t);
		sprintf(buf,"/media/sdc/ARMA%02d%02d%02d%02d%02d%02d.txt",now->tm_year,now->tm_mon,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
		logfile.open(buf,std::ios::out | std::ios::binary);
	}
}
void ManipulatorArm::CloseLog() {
	if (logfile.is_open())
		logfile.close();
}

bool ManipulatorArm::moveTo(int pos) {
    frc::SmartDashboard::PutNumber("Complete", m_MotionCase);

	// printf("Got moveTo()\n\r");
	
	//Initial Check
    if(pos >= m_PosCnt) //Make sure this position exists
        return true;
	//Make sure the target wasn't suddenly changed without re-initializing the motion
	if((pos != m_TargetPosition) && (m_MotionCase != 0))
		m_MotionCase = 0;
	
	//Motion State Machine
	switch (m_MotionCase) {
		case 0:
			for(int x=0;x<3;x++) {
				m_Moves[x]->setEndAngle(m_Positions[pos][x]);
				m_Moves[x]->setStartAngle(m_Segs[x]->getRelAngle());
				m_Moves[x]->setDuration(5);
			}
			frc::SmartDashboard::PutNumber("Sh Target",m_Positions[pos][0]);
			frc::SmartDashboard::PutNumber("EL Target",m_Positions[pos][1]);
			frc::SmartDashboard::PutNumber("Wr Target",m_Positions[pos][2]);

			frc::SmartDashboard::PutNumber("Mult",m_Moves[0]->getMultiplier());

			m_TargetPosition = pos;
			
            m_MotionCase++;
			break;
		case 1:
			{
			double timer = frc::Timer::GetFPGATimestamp();
			for(int x=0;x<3;x++) 
				m_Moves[x]->setStartTime(timer);
            m_MotionCase++;
			}
			break;
		case 2:
            {
			double timer = frc::Timer::GetFPGATimestamp();
			int done_cnt = 0; //Keeps count of how many motions are complete
			int wrist_move = 2;
			if(m_Segs[1]->getRelAngle() < -141)
				wrist_move = 0;
			for(int x=0;x<3;x++) {
				if((x==2)&&(wrist_move == 0)) {
					if(m_Segs[1]->getRelAngle() > -141)
						wrist_move = 1;
					continue;
				}
				else if ((x==2)&&(wrist_move == 1)) {
					// ******* Steve added setStartAngle for test March 2 1:00 pm
					// m_Moves[x]->setStartAngle(m_Segs[x]->getRelAngle()); 
					m_Moves[x]->setDuration(m_Moves[x]->getDuration() - (timer - m_Moves[x]->getStartTime()));
					m_Moves[x]->setStartTime(timer);
					wrist_move = 2;
					continue;
				}
				//Check if this motion is complete
				if (m_Moves[x]->isComplete(timer)) {
					done_cnt++;
					continue; //If it is skip this one
				}
				//Get the next relative angle to move to
				double angle1 = m_Moves[x]->calculateNextPoint(timer);
				
				if(angle1 != -1) { //If angle1 = -1 that means the motion is complete, so this is just to make sure that isn't the case
					m_Segs[x]->set(ControlMode::Position, m_Segs[x]->convertAngleToEncoder(angle1));
				}
			}
			if(done_cnt >= 3)
				m_MotionCase++;			
            }
			break;
		case 3:
			m_CurrentPosition = pos;
            m_MotionCase = 0;
            return true;
	}
	return false;
}

// New Move-To that will position to a set of angular co-ords in the arm_points[] array.
/*
bool ManipulatorArm::moveToNew(int pos) 
	{
	double err1,err2,err3;
	printf("case:%d, pos=%d, Target=%d ",m_MotionCase,pos,m_TargetIndex);
	if((pos != m_StartIndex) && (m_MotionCase != 0))
		m_MotionCase = 0;
	switch (m_MotionCase) {
		case 0: // Start of new target position.
			m_TargetIndex = m_StartIndex;
			m_StartIndex = pos;
            m_MotionCase++;
			break;
		case 1: // Tell joints where we want them to go.
			m_Segs[0]->set(ControlMode::Position, m_Segs[0]->convertAngleToEncoder(arm_points[0][m_TargetIndex].shoulderAngle));
			m_Segs[1]->set(ControlMode::Position, m_Segs[1]->convertAngleToEncoder(arm_points[0][m_TargetIndex].elbowAngle));
			m_Segs[2]->set(ControlMode::Position, m_Segs[2]->convertAngleToEncoder(arm_points[0][m_TargetIndex].wristAngleHatch));
			m_Segs[3]->set(rev::ControlType::kPosition,m_Segs[3]->convertAngleToEncoder(0)); // Set to 0 for now.
			m_MotionCase++; // Next state will monitor destination positions to seen when we've reached target.
			break;
		case 2: // See if we've gotten close enough to where we want to be (within 1.0 degrees of target)
			err1 = (m_Segs[0]->getRelAngle() - arm_points[0][m_TargetIndex].shoulderAngle);
			if (err1 < 0)
				err1 = -err1;
			err2 = (m_Segs[1]->getRelAngle() - arm_points[0][m_TargetIndex].elbowAngle);
			if (err2 < 0)
				err2 = -err2;
			err3 = (m_Segs[2]->getRelAngle() - arm_points[0][m_TargetIndex].wristAngleHatch);
			if (err3 < 0)
				err3 = -err3;
			printf("%f,%f,%f\n\r",err1,err2,err3);
			if ((err1 < 8.0)&&(err2 < 10.0)&&(err3 < 15.0))
				{
				// printf("err1=%f,err2=%f,err3=%f\n\r",err1,err2,err3);
				if (pos > m_TargetIndex)
					{
					m_TargetIndex++;
					m_MotionCase = 1; // Back to Case 1 to set the positions.
					}
				else if (pos < m_TargetIndex)
					{
					m_TargetIndex--;
					m_MotionCase = 1; // Back to Case 1 to set the positions.
					}
				else // If we're at the pos target, we're done.
					return true;
				}
			break;
		}
	return false;
	}
	*/

// New Move-To that will position using inverse kinematics
// When running at higher speeds, we should allow for a ramp-down at the end of the
// move.  We can use m_Cycles for this.  When we get within 0.2 seconds (10 count)
// of the end, scale inches per second but then we'll need a few more cycles to
// actually get to our target.  Normally inchesPerSecondScale would be 1.0
// a slow down can be implemented by something like inchesPerSecondScale -= 0.1 or
// till inchesPerSecondScale < 0.2 but don't decrease it any further.
// could also do inchesPerSecondScale *= 0.9 or 0.8 or something for a bit different
// approach.
bool ManipulatorArm::moveToXY(double x, double y, double wristAbsTarget, double waistTarget, double inchesPerSecond)
	{
	double wristAngle,deltaWrist;
	double deltaX,deltaY,absDeltaX,absDeltaY;
	double waistDelta,waistHigh,waistLow;
	double prev_InvKinShoulderAngle,prev_InvKinElbowAngle;			
	bool doneWaist,moveallowed;
	int m_CyclesLeft;

	m_CycleCount++; // Should count from 0 to m_Cycles
	deltaX = x - m_TargetX;
	deltaY = y - m_TargetY;
	absDeltaX = fabs(deltaX);
	absDeltaY = fabs(deltaY);
	if((absDeltaX > 0.1)||(absDeltaY > 0.1)) // If we have a new x,y target, init
		m_MotionCase = 0;
	switch (m_MotionCase) {
		case 0: // Start of new target position.
			m_Segs[2]->setAbsAngleTarget(wristAbsTarget); // set the ABS Target.
			if ((fabs(m_TargetX - m_Segs[1]->getEndX()) < 1.0)&&(fabs(m_TargetY - m_Segs[1]->getEndY()) < 1.0))
				{ // if m_TargetX and m_TargetY are pretty close to where we really are, start position is m_TargetX, m_TargetY
				m_StartX = m_TargetX;
				m_StartY = m_TargetY;
				m_StartW = m_Segs[2]->getRelAngle();
				}
			else // Robot is not really close to the m_TargetX, m_TargetY location, use actual location
				{
				m_StartX = m_Segs[1]->getEndX();
				m_StartY = m_Segs[1]->getEndY();
				m_StartW = m_Segs[2]->getRelAngle();
				}
			if (absDeltaX > absDeltaY)
				{ // We've got more X to go than Y so m_XinchesPerSecond will be the larger one
				m_XinchesPerSecond = inchesPerSecond/50.0;	
				if (absDeltaX != 0)
					m_YinchesPerSecond = inchesPerSecond * (absDeltaY / absDeltaX) / 50.0;
				else
					m_YinchesPerSecond = 0;
				m_Cycles = absDeltaX / m_XinchesPerSecond; // number of times we expect to process
				// used to smoothly rotate wrist during this move.
				}
			else // We've got more Y to go than X so m_YinchesPerSecond will be the larger one.
				{
				m_YinchesPerSecond = inchesPerSecond / 50.0;	
				if (absDeltaY != 0)
					m_XinchesPerSecond = inchesPerSecond * (absDeltaX / absDeltaY) / 50.0;
				else
					m_XinchesPerSecond = 0;				
				m_Cycles = absDeltaY / m_YinchesPerSecond; // number of times we expect to process
				// used to smoothly rotate wrist during this move.
				}
			m_TargetX = x; // Establish new target.  This also ensures m_MotionCase won't get
			m_TargetY = y; // set to 0 unless the target changes.
			m_CycleCount = 0; // Used to count cycles.
			doneWaist = false; // Assume waist movement is not complete yet.
            m_MotionCase++;
			break;
		case 1: // adjust X and Y for motion path, use inverse kinematics to get angles
			// and set the positions for the axes to follow this path.
			m_CyclesLeft = m_Cycles - m_CycleCount;
			if (m_CyclesLeft < 0)
				m_CyclesLeft = 1;
			if (m_StartX < m_TargetX)
				m_StartX += m_XinchesPerSecond;
			else
				m_StartX -= m_XinchesPerSecond;
			if (m_StartY < m_TargetY)
				m_StartY += m_YinchesPerSecond;
			else
				m_StartY -= m_YinchesPerSecond;
			
			prev_InvKinShoulderAngle = m_InvKinShoulderAngle; // Keep track of shoulder angle 
			prev_InvKinElbowAngle = m_InvKinElbowAngle;
			// prior to kinematics adjustment.
			InverseKinematics(m_StartX,m_StartY - 13.7); // Do the inverse Kinematics calculation
			// This will generate m_InvKinElbowAngle and m_InvKinShoulderAngle
		
			// in the event that this new shoulder angle would limit waist movement but the
			// previous one would not, we may need to delay X,Y movement till waist has
			// reached an acceptable position.
			moveallowed = true; // Assume we're fine to complete the kinematics movement.
			if ((prev_InvKinShoulderAngle < 115.0)&&(m_InvKinShoulderAngle >= 115.0))
				{ // Waist becomes limited to -31.0 to +36.5 in range 115 to 140
				// If we're not yet in that range but waist target is in that range, wait for
				// waist to finish moving before we carry on with X,Y
				// Otherwise, we have a fault/error condition and this MoveXY needs to end.
				if ((m_TargetWaist < -31.0)||(m_TargetWaist > 36.5))
					{
					if ((waistTarget < 31.0)||(waistTarget > 36.5)) // Target won't allow this to ever happen
						return true; // no solution to this moveXY.  Exit indicating done.
					else
						moveallowed = false; // delay kinematics move till waist allows for it.					
					}
				}
			else if ((prev_InvKinShoulderAngle < 140)&&(m_InvKinShoulderAngle >= 140)&&(m_InvKinElbowAngle < 163))
				{ // This low down, we need to get between the posts with a waist angle of -4.5 to -6.5
				// Again, if waist target is not in this range, we need to end this move right where we are.
				if ((m_TargetWaist < -6.5)||(m_TargetWaist > -4.5))
					{
					if ((waistTarget < -6.5)||(waistTarget > -4.5)) // Target won't allow this to ever happen
						return true; // no solution to this moveXY.  Exit indicating done.
					else
						moveallowed = false; // delay kinematics move till waist allows for it.					
					}
				}

			//printf("sX=%7.2f sY=%7.2f SA=%7.2f EA=%7.2f m_X=%7.2f m_Y=%7.2f m_TX=%7.2f m_TY=%7.2f\n\r",m_StartX,m_StartY,m_InvKinShoulderAngle,m_InvKinElbowAngle,m_XinchesPerSecond,m_YinchesPerSecond,m_TargetX,m_TargetY);
			if (moveallowed)
				{ // movement is allowed as per waist position.
				m_Segs[0]->set(rev::ControlType::kPosition, m_Segs[0]->convertAngleToEncoder(m_InvKinShoulderAngle));
				m_Segs[1]->set(rev::ControlType::kPosition, m_Segs[1]->convertAngleToEncoder(m_InvKinElbowAngle));
				}
			else // Move is not allowed, restore m_InvKin values to what they were before.
				{
				m_InvKinShoulderAngle = prev_InvKinShoulderAngle;
				m_InvKinElbowAngle = prev_InvKinElbowAngle;
				}
			

			// If we're allowed to move the wrist, do so, based on AbsAngleTarget and relative values of shoulder and elbow.
			if(m_Segs[1]->getRelAngle() > -141) // Move gradually based on m_Cycles and m_CycleCount
				{
				wristAngle = m_Segs[2]->getAbsAngleTarget() - m_InvKinShoulderAngle - m_InvKinElbowAngle;
				// wristAngle has target.  Make sure we move there gradually
				deltaWrist = wristAngle - m_StartW;
				if ((deltaWrist < 4.0)||(m_Cycles - m_CycleCount < 5)) // We're close to actual angle, write the value.
					{
					m_StartW = wristAngle;
					}
				else // We're not that close.  Add / Sub to m_StartW based on m_Cycles
					{ 
					m_StartW += 4 * deltaWrist / m_Cycles;
					}
				// Do the actual wrist positioning.
				m_Segs[2]->set(ControlMode::Position, m_Segs[2]->convertAngleToEncoder(m_StartW));
				}

			// Waist movement happens here ...
			// if m_TargetWaist != parameter waistTarget, adjust m_TargetWaist so that it
			// makes its way to match waistTarget by the time m_CycleCount get to m_Cycles
			// we may need to delay waist movement based on shoulder and elbow positions.
			if (!doneWaist) // Only process waist stuff if there's a need to move it.
				{
				waistDelta = waistTarget - m_TargetWaist;
				if ((fabs(waistDelta) > 0.05)&&(m_CyclesLeft != 0))
					{ 
					if (m_InvKinShoulderAngle < 115.0) // waist is limited to full range +/-65.0 when shoulder < 115.0
						{
						waistLow = -65.0;
						waistHigh = 65.0;
						}
					else if (m_InvKinShoulderAngle < 140) // waist is limited to -31 to 36.5
						{
						waistLow = -31.0;
						waistHigh = 36.5;
						}
					else if (m_InvKinElbowAngle > -163)
						{
						waistLow = -31.0;
						waistHigh = 36.5;
						}	
					else
						{
						waistLow = -6.5;
						waistHigh = -4.5;
						}
					if ((waistTarget >= waistLow)&&(waistTarget <= waistHigh))
						{
						m_TargetWaist += waistDelta / m_CyclesLeft; 	
						m_Segs[3]->set(rev::ControlType::kPosition,m_Segs[3]->convertAngleToEncoder(m_TargetWaist)); // Set to 0 for now.
						}
					}
				else
					{
					doneWaist = true;
					}
				}
			

			// Adjust this to allow some waist movement as long as elbow is up above stilts
			// Then allow full waist movement when shoulder is less than 116

			// Check if we're done
			if ((fabs(m_StartX - m_TargetX) < inchesPerSecond/50)&&(fabs(m_StartY - m_TargetY) < inchesPerSecond/50))
				return true;
			break;
		}
	return false;
	}

void ManipulatorArm::addPosition(int shoulderAngle, int elbowAngle, int wristAngle) {
    m_Positions[m_PosCnt][0] = shoulderAngle;
    m_Positions[m_PosCnt][1] = elbowAngle;
    m_Positions[m_PosCnt][2] = wristAngle;

    m_PosCnt++;
}

/*
void ManipulatorArm::addPoint(int index,double shoulderAngle, double elbowAngle, double wristAngle1, double wristAngle2) {
    arm_points[index][num_arm_points[index]].shoulderAngle = shoulderAngle;
    arm_points[index][num_arm_points[index]].elbowAngle = elbowAngle;
    arm_points[index][num_arm_points[index]].wristAngleHatch = wristAngle1;
    arm_points[index][num_arm_points[index]].wristAngleCargo = wristAngle2;
	num_arm_points[index]++;
}*/

void ManipulatorArm::setShoulderandElbow(double shoulderAngle, double elbowAngle)
	{
	double wristAngle;
	m_Segs[0]->set(rev::ControlType::kPosition, m_Segs[0]->convertAngleToEncoder(shoulderAngle));
	m_Segs[1]->set(rev::ControlType::kPosition, m_Segs[1]->convertAngleToEncoder(elbowAngle));		
	m_Segs[3]->set(rev::ControlType::kPosition,m_Segs[3]->convertAngleToEncoder(0)); // Set to 0 for now.
	if(m_Segs[1]->getRelAngle() > -141)
		{
		wristAngle = m_Segs[2]->getAbsAngleTarget() - m_Segs[1]->getRelAngle() - m_Segs[0]->getRelAngle();
		m_Segs[2]->set(ControlMode::Position, m_Segs[2]->convertAngleToEncoder(wristAngle));
		}
	}
void ManipulatorArm::incAbsAngleTarget()
	{
	m_Segs[2]->setAbsAngleTarget(m_Segs[2]->getAbsAngleTarget() + 0.25);
	}
void ManipulatorArm::decAbsAngleTarget()
	{
	m_Segs[2]->setAbsAngleTarget(m_Segs[2]->getAbsAngleTarget() - 0.25);
	}
void ManipulatorArm::InverseKinematics(double x, double y)
	{
	double term1,Sh_len,El_len,Sh_ang,El_ang,Sh_deg,El_deg;
	// Formula is as follows:
	// Elbow_Angle = ACOS((x^2+y^2 - Sh_len^2 - El_len^2) / (2 * Sh_len * El_len))
	// Shoulder_Angle = ATAN(y/x) - ATAN((El_len * SIN(Elbow_Angle))/(Sh_len = El_len * COS(Elbow_Angle)))
	// For now, we're going to get x from m_PhysicalAttributes.posX for the elbow
	// and y from .poxY.  Sh_len is length in the PhysicalAttributes for the shoulder
	// El_len is length in PhysicalAttributes for the elbow.  See the forward
	// kinematics further up in this file for how to get this data.

	//x = m_Segs[1]->getEndX(); 
	//y = m_Segs[1]->getEndY() - 13.7; // Y is allowing for distance to pivot point
	Sh_len = m_Segs[0]->getLength();
	El_len = m_Segs[1]->getLength();

	term1 = (x*x + y*y - Sh_len*Sh_len - El_len * El_len)/(2 * Sh_len * El_len);
	if (term1 < -1.0) // Make sure term1 is between -1 and 1 so ACOS will work.
		term1 = -1.0;
	if (term1 > 1.0)
		term1 = 1.0;
	El_ang = -std::acos(term1); // Get elbow angle
	El_deg = El_ang * 180.0 / M_PI; // Get in degrees
	if (x > 0)
		{
		Sh_ang = std::atan(y/x) - std::atan((El_len * std::sin(El_ang))/(Sh_len + El_len * std::cos(El_ang)));
		Sh_deg = Sh_ang * 180 / M_PI;
		}
	else // Not planning to deal with conditions where x <= 0
		{
		Sh_ang = 0; 
		Sh_deg = 0;
		}
	m_InvKinShoulderAngle = Sh_deg; // Set the angle variables.
	m_InvKinElbowAngle = El_deg;
	}

void ManipulatorArm::setAbsAngleWrist(double angle) {
	m_Segs[2]->setAbsAngleTarget(angle);
}

void ManipulatorArm::moveWaist(double angle) {
	if(m_Segs[0]->getRelAngle() > 116) {
		//Potential collision!
		if(angle > 30)
			angle = 30;
		else if (angle < -30)
			angle = -30;
	}

	m_Segs[3]->set(rev::ControlType::kPosition, m_Segs[3]->convertAngleToEncoder(angle));
}

bool ManipulatorArm::isHatchMode() {
	return (m_IntakeMode == 1); // true if we expect we have a hatch in the intake
}

//Intakes
void ManipulatorArm::intakeWheelsSpin(double power){
    intakeWheels->Set(-power);
}

void ManipulatorArm::intakeWheelsOff(){
    intakeWheels->Set(0);
}

void ManipulatorArm::releaseHatch(){
    hatchServo->Set(1);
}

void ManipulatorArm::grabHatch(){
    hatchServo->Set(0);
}

void ManipulatorArm::setInCargoPosition() {
	m_InCargoPosition = true;
}
void ManipulatorArm::clearInCargoPosition() {
	m_InCargoPosition = false;
}
bool ManipulatorArm::getInCargoPosition() {
	return (m_InCargoPosition);
}
bool ManipulatorArm::ifCargo() {
	return !cargoSensor->Get();
}
bool ManipulatorArm::ifHatch() {
	return !hatchSensor->Get();
}
void ManipulatorArm::setInHatchPosition() {
	m_InHatchPosition = true;
}
void ManipulatorArm::clearInHatchPosition() {
	m_InHatchPosition = false;
}
bool ManipulatorArm::getInHatchPosition() {
	return (m_InHatchPosition);
}
void ManipulatorArm::setIntakeMode(int mode) {
	if (m_IntakeMode != mode)
		{
		m_IntakeMode = mode;
		if (mode == 1)
			m_Segs[2]->setAbsAngleTarget(-190.0);
		else if (mode == 2)
			m_Segs[2]->setAbsAngleTarget(-20.0);
		printf("Changed Intake Mode to %d, Wrist Angle = %f\n\r",mode,m_Segs[2]->getAbsAngleTarget());
		}
}
int ManipulatorArm::getIntakeMode() {
	return m_IntakeMode;
}

void ManipulatorArm::fineMotion() {
	double prev_StartX,prev_StartY;	
	double waistHigh,waistLow;
	bool moveallowed;

	double JoyX = Robot::oi->getoperator1()->GetX(); // Read values of the 4 joystick axes and work from here.
	double JoyY = -Robot::oi->getoperator1()->GetY();
	double JoyZ = Robot::oi->getoperator1()->GetRawAxis(3);
	double JoyW = -Robot::oi->getoperator1()->GetZ();

	printf("x=%f y=%f W=%f Z=%f\n\r",JoyX,JoyY,JoyW,JoyZ);
	
	//JoyX = 0; // Disable everything by setting these to 0.
	//JoyY = 0;
	//JoyZ = 0;
	//JoyW = 0;

	// Clear the Deadband
	if(fabs(JoyX) < 0.05)
		JoyX = 0;
	if(fabs(JoyY) < 0.05)
		JoyY = 0;
	if(fabs(JoyW) < 0.05)
		JoyW = 0;
	if(fabs(JoyZ) < 0.05)
		JoyZ = 0;
	if ((JoyX != 0)||(JoyY !=0)) // If we have X/Y fine motion ...
		{
		// We may want to change this to allow simultaneous X & Y movement.
		prev_StartX = m_StartX; // InvKinShoulderAngle = m_InvKinShoulderAngle; // Keep track of shoulder angle 
		prev_StartY = m_StartY; // InvKinElbowAngle = m_InvKinElbowAngle;
		if (fabs(JoyX) > fabs(JoyY)) // if X is bigger, only adjust in/out
			{
			m_StartX += JoyX/4.0;
			}
		else // JoyY is bigger, only adjust vertical position
			{
			m_StartY += JoyY/4.0;
			}
		if (m_StartX < 1.0) // Don't allow us to cross into -X territory.
			m_StartX = 1.0;
		if (m_StartX > 46.0 - 14.0) // Limit foward reach for now to just 46 - 10
			m_StartX = prev_StartX; // Eventually the -10 will be replaced by wrist calculation.
		InverseKinematics(m_StartX,m_StartY - 13.7); // Do the inverse Kinematics calculation
		// This will generate m_InvKinElbowAngle and m_InvKinShoulderAngle
	
		//printf("sX=%7.2f sY=%7.2f SA=%7.2f EA=%7.2f m_X=%7.2f m_Y=%7.2f m_TX=%7.2f m_TY=%7.2f\n\r",m_StartX,m_StartY,m_InvKinShoulderAngle,m_InvKinElbowAngle,m_XinchesPerSecond,m_YinchesPerSecond,m_TargetX,m_TargetY);
		moveallowed = true; // Assume we're fine to complete the kinematics movement.
		if ((m_InvKinShoulderAngle > 115.0)&&(m_InvKinShoulderAngle <= 140.0))
			{ // Waist becomes limited to -31.0 to +36.5 in range 115 to 140
			// If we're not yet in that range but waist target is in that range, wait for
			// waist to finish moving before we carry on with X,Y
			// Otherwise, we have a fault/error condition and this MoveXY needs to end.
			if ((m_TargetWaist < -31.0)||(m_TargetWaist > 36.5))
				{
				moveallowed = false; // delay kinematics move till waist allows for it.					
				}
			}
		else if ((m_InvKinShoulderAngle > 140)&&(m_InvKinElbowAngle < 163))
			{ // This low down, we need to get between the posts with a waist angle of -4.5 to -6.5
			// Again, if waist target is not inis range, we need to end this move right where we are.
			if ((m_TargetWaist < -6.5)||(m_TargetWaist > -4.5))
				{
				moveallowed = false; // delay kinematics move till waist allows for it.					
				}
			}
		if (moveallowed)
			{ // movement is allowed as per waist position.
			m_Segs[0]->set(rev::ControlType::kPosition, m_Segs[0]->convertAngleToEncoder(m_InvKinShoulderAngle));
			m_Segs[1]->set(rev::ControlType::kPosition, m_Segs[1]->convertAngleToEncoder(m_InvKinElbowAngle));
			}
		else // Move is not allowed, restore m_InvKin values to what they were before.
			{
			m_StartX = prev_StartX;
			m_StartY = prev_StartY;
			}

		// If we're allowed to move the wrist, do so, based on AbsAngleTarget and relative values of shoulder and elbow.
		if(m_Segs[1]->getRelAngle() > -141) // Move gradually based on m_Cycles and m_CycleCount
			{
			m_StartW = m_Segs[2]->getAbsAngleTarget() - m_InvKinShoulderAngle - m_InvKinElbowAngle;
			// Do the actual wrist positioning to keep it oriented while elbow and shoulder move.
			m_Segs[2]->set(ControlMode::Position, m_Segs[2]->convertAngleToEncoder(m_StartW));
			}
		}
	if((JoyW != 0)||(JoyZ != 0)) // If we have waist or wrist fine motion ... 
		{
		if (fabs(JoyW) > fabs(JoyZ)) // Only do 1 or the other.
			{ // Move Waist - limit based on shoulder angle, possibly on elbow angle too.  When fully retracted, need to stay at 0.
			if (m_Segs[1]->getRelAngle() < 116.0)
				{
				m_TargetWaist += JoyW / 2.0;
				if (m_InvKinShoulderAngle < 115.0) // waist is limited to full range +/-65.0 when shoulder < 115.0
					{
					waistLow = -65.0;
					waistHigh = 65.0;
					}
				else if (m_InvKinShoulderAngle < 140) // waist is limited to -31 to 36.5
					{
					waistLow = -31.0;
					waistHigh = 36.5;
					}
				else if (m_InvKinElbowAngle > -163)
					{
					waistLow = -31.0;
					waistHigh = 36.5;
					}	
				else
					{
					waistLow = -6.5;
					waistHigh = -4.5;
					}
				if (m_TargetWaist < waistLow)
					m_TargetWaist = waistLow;
				if (m_TargetWaist > waistHigh)
					m_TargetWaist = waistHigh;
				m_Segs[3]->set(rev::ControlType::kPosition,m_Segs[3]->convertAngleToEncoder(m_TargetWaist)); // Set the target waist angle.
				}
			}
		else
			{ // Move Wrist - only if elbow is up above 
			if (m_Segs[1]->getRelAngle() > -141)
				{
				m_Segs[2]->setAbsAngleTarget(m_Segs[2]->getAbsAngleTarget() + JoyZ/1.0);
				m_StartW = m_Segs[2]->getAbsAngleTarget() - m_InvKinShoulderAngle - m_InvKinElbowAngle;
				m_Segs[2]->set(ControlMode::Position, m_Segs[2]->convertAngleToEncoder(m_StartW));
				}
			}
		}	
}