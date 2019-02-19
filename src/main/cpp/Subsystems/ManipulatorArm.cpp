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
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "../../include/Commands/MoveArm.h"

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

ManipulatorArm::ManipulatorArm() : frc::Subsystem("ManipulatorArm") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    waist.reset(new rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless));
    shoulder.reset(new WPI_TalonSRX(10));
    elbow.reset(new WPI_TalonSRX(9));
    wrist.reset(new WPI_TalonSRX(8));
    intakeWheels.reset(new WPI_VictorSPX(11));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    m_Segs[0] = new ArmSegment(shoulder,32, NULL, 0, 13.7);
    m_Segs[1] = new ArmSegment(elbow,32, m_Segs[0]);
    m_Segs[2] = new ArmSegment(wrist,14, m_Segs[1]);
}

bool ManipulatorArm::Init(){

    m_Segs[0]->setSensorDirection(true);
    m_Segs[0]->setVoltageLimit(true, 8);
    m_Segs[0]->setPID(1,0,0);

    m_Segs[1]->setSensorDirection(true);
    m_Segs[1]->setVoltageLimit(true, 8);
    m_Segs[1]->setPID(1,0,0);

    m_Segs[2]->setVoltageLimit(true, 8);
    m_Segs[2]->setPID(1,0,0);
    /*shoulder->EnableCurrentLimit(false);
	shoulder->ConfigVoltageCompSaturation(8, 0);
	shoulder->EnableVoltageCompensation(true);
    shoulder->SetSensorPhase(true);
	setShoulderPID(1,0,0);
	shoulder->ConfigAllowableClosedloopError(0, 5, 0);
	shoulder->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	shoulder->SetStatusFramePeriod(StatusFrame::Status_1_General_, 10, 0);

    elbow->EnableCurrentLimit(false);
	elbow->ConfigVoltageCompSaturation(9, 0);
	elbow->EnableVoltageCompensation(true);
    elbow->SetSensorPhase(true);
	setElbowPID(1,0,0);
	elbow->ConfigAllowableClosedloopError(0, 5, 0);
	elbow->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	elbow->SetStatusFramePeriod(StatusFrame::Status_1_General_, 10, 0);

    wrist->EnableCurrentLimit(false);
	wrist->ConfigVoltageCompSaturation(5, 0);
	wrist->EnableVoltageCompensation(true);
	setWristPID(2,0,0);
	wrist->ConfigAllowableClosedloopError(0, 5, 0);
	wrist->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	wrist->SetStatusFramePeriod(StatusFrame::Status_1_General_, 10, 0);*/
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

}

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

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
    shoulder->Config_kP(0, p, 0);
    shoulder->Config_kI(0, i, 0);
    shoulder->Config_kD(0, d, 0);
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
void ManipulatorArm::spinOutward(double power){
    intakeWheels->Set(-power);
}

void ManipulatorArm::spinInward(double power){
    intakeWheels->Set(power);
}