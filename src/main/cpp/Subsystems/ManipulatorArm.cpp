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
#include "rev/CANSparkMax.h"
#include "Subsystems/Arm/Movement.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "../../include/Commands/MoveArm.h"

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

#define SHOULDER_ANGLE90     90
#define SHOULDER_ANGLE0      0
#define SHOULDER_ENCODER90   -1778
#define SHOULDER_ENCODER0    -7055

#define ELBOW_ANGLE150     150
#define ELBOW_ANGLE0      0
#define ELBOW_ENCODER150   -1778
#define ELBOW_ENCODER0    -7055

#define SHOULDER_POT150       604
#define SHOULDER_POT0        968

ManipulatorArm::ManipulatorArm() : frc::Subsystem("ManipulatorArm") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    waist.reset(new rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless));
    shoulder.reset(new WPI_TalonSRX(10));
    elbow.reset(new WPI_TalonSRX(9));
    wrist.reset(new WPI_TalonSRX(8));
    intakeWheels.reset(new WPI_VictorSPX(11));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    hatchServo.reset(new frc::Servo(0));
    AddChild("Servo", hatchServo);

    m_Segs[0] = new ArmSegment(shoulder,32, NULL, 0, 13.7);
    m_Segs[1] = new ArmSegment(elbow,32, m_Segs[0]);
    m_Segs[2] = new ArmSegment(wrist,14, m_Segs[1]);

    m_Moves[0] = new ArmMotion(0,0,10,0);
}

bool ManipulatorArm::Init(){

    m_Segs[0]->setSensorDirection(true);
    m_Segs[0]->setVoltageLimit(true, 12);
    m_Segs[0]->setPID(1,0,0);

    m_Segs[0]->setLimits(2619, 165, -7818, -13);

    m_Segs[1]->setSensorDirection(true);
    m_Segs[1]->setLimits(16519, -135, -123, 150);
    m_Segs[1]->setVoltageLimit(true, 12);
    m_Segs[1]->setPID(1,0,0);

    printf("%f\n\n\n\n\n\n\n",m_Segs[1]->convertAngleToEncoder(0));

    m_Segs[2]->setVoltageLimit(true, 8);
    m_Segs[2]->setLimits(-1919, 180, 60, 0);
    m_Segs[2]->setPID(1,0,0);

    //frc::SmartDashboard::PutNumber("Elbow Slope", m_Segs[2]->getConversionSlope());
    //frc::SmartDashboard::PutNumber("Elbow Intercept", m_Segs[2]->getConversionIntercept());
}

void ManipulatorArm::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        //SetDefaultCommand(new MoveArm(0));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void ManipulatorArm::Periodic() {
    // Put code here to be run every loop
    for(int x=0;x<2;x++)
        m_Segs[x]->updateAngles();
    
    frc::SmartDashboard::PutNumber("Shoulder Angle", m_Segs[0]->getRelAngle());
	frc::SmartDashboard::PutNumber("Wrist Angle", m_Segs[2]->getRelAngle());
	frc::SmartDashboard::PutNumber("Elbow Angle", m_Segs[1]->getRelAngle());
    
}

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

bool ManipulatorArm::moveTo(int pos) {
    frc::SmartDashboard::PutNumber("Complete", m_MotionCase);
    if(pos > m_PosCnt)
        return true;
	switch (m_MotionCase) {
		case 0:
            //if (m_Moves[0] != NULL)
            //    delete m_Moves[0];
            
            m_Moves[0]->setEndAngle(m_Positions[pos][1]);
            m_Moves[0]->setStartAngle(m_Segs[1]->getRelAngle());
            m_Moves[0]->setDuration(10);

            frc::SmartDashboard::PutNumber("Target", m_Positions[pos][1]);

            m_MotionCase++;
			break;
		case 1:
            m_Moves[0]->setStartTime(frc::Timer::GetFPGATimestamp());
            m_MotionCase++;
			break;
		case 2:
            {
            double angle1 = m_Moves[0]->calculateNextPoint(frc::Timer::GetFPGATimestamp());
            
            if(angle1 != -1)
                m_Segs[1]->set(ControlMode::Position, m_Segs[1]->convertAngleToEncoder(angle1));    
            //frc::SmartDashboard::PutNumber("Test", m_Segs[1]->convertAngleToEncoder(angle1));
            if(m_Moves[0]->isComplete())
                m_MotionCase++;
            }
			break;
		case 3:
            //if (m_Moves[0] != NULL)
            //    delete m_Moves[0];
            m_MotionCase = 0;
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

void ManipulatorArm::setShoulderEncoder(int val){
    m_Segs[0]->set(ControlMode::Position, val);
    //shoulder->Set(ControlMode::Position, val);
}

void ManipulatorArm::setElbowEncoder(int val){
    m_Segs[1]->set(ControlMode::Position, val);
    //elbow->Set(ControlMode::Position, val);
}

void ManipulatorArm::setWristEncoder(int val){
    m_Segs[2]->set(ControlMode::Position, val);
    //wrist->Set(ControlMode::Position, val);
}

int ManipulatorArm::getShoulderEncoder(){
    return shoulder->GetSelectedSensorPosition(0);
}
int ManipulatorArm::getShoulderPot(){
    return shoulder->GetSensorCollection().GetAnalogIn();
}

int ManipulatorArm::getWristEncoder(){
    return wrist->GetSelectedSensorPosition(0);
}
int ManipulatorArm::getWristPot(){
    return wrist->GetSensorCollection().GetAnalogIn();
}

int ManipulatorArm::getElbowEncoder(){
    return elbow->GetSelectedSensorPosition(0);
}
int ManipulatorArm::getElbowPot(){
    return elbow->GetSensorCollection().GetAnalogIn();
}

double ManipulatorArm::getWaistEncoder(){
   return waist->GetEncoder().GetPosition();
}
//double ManipulatorArm::getWaistPot(){
//   return waist->GetEncoder().GetPosition();
//}

void ManipulatorArm::setShoulderPID(double p, double i, double d){
    m_Segs[0]->setPID(p,i,d);
}

void ManipulatorArm::setWristPID(double p, double i, double d){
    wrist->Config_kP(0, p, 0);
    wrist->Config_kI(0, i, 0);
    wrist->Config_kD(0, d, 0);
}

void ManipulatorArm::setElbowPID(double p, double i, double d){
    elbow->Config_kP(0, p, 0);
    elbow->Config_kI(0, i, 0);
    elbow->Config_kD(0, d, 0);
}

void ManipulatorArm::setWaistPID(double p, double i, double d){
    waist->GetPIDController().SetP(p);
    waist->GetPIDController().SetI(i);
    waist->GetPIDController().SetD(d);
}

//Intakes
void ManipulatorArm::intakeWheelsSpin(double power){
    intakeWheels->Set(-power);
}

void ManipulatorArm::intakeWheelsOff(){
    intakeWheels->Set(0);
}

void ManipulatorArm::releaseHatch(){
    hatchServo->Set(0);
}

void ManipulatorArm::grabHatch(){
    hatchServo->Set(1);
}