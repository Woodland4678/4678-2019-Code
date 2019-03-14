// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

//Test 2
#include "Robot.h"

#include <frc/Commands/Scheduler.h>
#include <frc/SmartDashboard/SmartDashboard.h>

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION
std::shared_ptr<Drivetrain> Robot::drivetrain;
std::shared_ptr<Lidar> Robot::lidar;
std::shared_ptr<ManipulatorArm> Robot::manipulatorArm;
std::shared_ptr<Climber> Robot::climber;
std::unique_ptr<OI> Robot::oi;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION
AHRS * Robot::ahrs;
void Robot::RobotInit() {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    drivetrain.reset(new Drivetrain());
    lidar.reset(new Lidar());
    manipulatorArm.reset(new ManipulatorArm());
    climber.reset(new Climber());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	ahrs = new AHRS(SPI::Port::kMXP);
	// This MUST be here. If the OI creates Commands (which it very likely
	// will), constructing it during the construction of CommandBase (from
	// which commands extend), subsystems are not guaranteed to be
	// yet. Thus, their requires() statements may grab null pointers. Bad
	// news. Don't move it.
	oi.reset(new OI());
	drivetrain->configMotors();
	manipulatorArm->Init();

	// Add commands to Autonomous Sendable Chooser
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS


	chooser.SetDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
	frc::SmartDashboard::PutData("Auto Modes", &chooser);
}

/**
 * This function is called when the disabled button is hit.
 * You can use it to reset subsystems before shutting down.
 */
void Robot::DisabledInit(){
	lidar->stopLidar();
	manipulatorArm->CloseLog();
}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	autonomousCommand = chooser.GetSelected();
	if (autonomousCommand != nullptr)
		autonomousCommand->Start();
}

void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	lidar->startLidar();

	manipulatorArm->openLog();
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// these lines or comment it out.

	atshoulder = 119.0;
	atelbow = -145.0;

	if (autonomousCommand != nullptr)
		autonomousCommand->Cancel();
}

void Robot::TeleopPeriodic() {
	char buf[255];
	double JoyY(Robot::oi->getdriver()->GetRawAxis(3));

    // Deadzone check
    if (JoyY < 0.05 && JoyY > -0.05) {
        JoyY = 0.0;
    }
	else {
		if ((Robot::manipulatorArm->getInCargoPosition()) && (JoyY > 0.01))
			Robot::manipulatorArm->setIntakeMode(2); // Mode 2, we have cargo.
		if (JoyY <-0.01) // Cargo ejected
			Robot::manipulatorArm->setIntakeMode(0); // Mode 0, we just spit out the cargo.
	}
	//frc::SmartDashboard::PutNumber("Waist Encoder", manipulatorArm->getWaistPot());
	if(Robot::manipulatorArm->m_CurrentPosition == 0)
		Robot::manipulatorArm->intakeWheelsSpin(-JoyY);
	// Code added to allow use of POV to move shoulder and elbow for
	// establishing target positions with the robot driving them.

	// printf("Operator POV=%d\n\r",Robot::oi->getoperator1()->GetPOV());
	/*switch(Robot::oi->getoperator1()->GetPOV())
	{
		case 0: // Up
			atshoulder += 0.5;
			Robot::manipulatorArm->setShoulderandElbow(atshoulder,atelbow);
			sprintf(buf,"%f,%f\n\r",atshoulder,atelbow);
			frc::SmartDashboard::PutString("Shoulder+Elbow",buf);
		break;
		case 90: // Right
			atelbow += 0.5;
			Robot::manipulatorArm->setShoulderandElbow(atshoulder,atelbow);
			//printf("Right Sh=%f, El=%f\n\r",atshoulder,atelbow);
			sprintf(buf,"%f,%f\n\r",atshoulder,atelbow);
			frc::SmartDashboard::PutString("Shoulder+Elbow",buf);
		break;
		case 180: // Down
			atshoulder -= 0.5;
			Robot::manipulatorArm->setShoulderandElbow(atshoulder,atelbow);
			//printf("Down Sh=%f, El=%f\n\r",atshoulder,atelbow);
			sprintf(buf,"%f,%f\n\r",atshoulder,atelbow);
			frc::SmartDashboard::PutString("Shoulder+Elbow",buf);
		break;
		case 270: // Left
			atelbow -= 0.5;
			Robot::manipulatorArm->setShoulderandElbow(atshoulder,atelbow);
			//printf("Left Sh=%f, El=%f\n\r",atshoulder,atelbow);
			sprintf(buf,"%f,%f\n\r",atshoulder,atelbow);
			frc::SmartDashboard::PutString("Shoulder+Elbow",buf);
		break;
	}
	switch(Robot::oi->getdriver()->GetPOV())
	{
		case 0: // Up
			Robot::manipulatorArm->incAbsAngleTarget(); // m_Segs[2]->setAbsAngleTarget(Robot::manipulatorArm->m_Segs[2]->getAbsAngleTarget() + 0.25);
			Robot::manipulatorArm->setShoulderandElbow(atshoulder,atelbow);
		break;
		case 90: // Right
			Robot::manipulatorArm->setShoulderandElbow(atshoulder,atelbow);
			//printf("Right Sh=%f, El=%f\n\r",atshoulder,atelbow);
		break;
		case 180: // Down
			Robot::manipulatorArm->decAbsAngleTarget(); // m_Segs[2]->setAbsAngleTarget(Robot::manipulatorArm->m_Segs[2]->getAbsAngleTarget() - 0.25);
			Robot::manipulatorArm->setShoulderandElbow(atshoulder,atelbow);
			//printf("Down Sh=%f, El=%f\n\r",atshoulder,atelbow);
		break;
		case 270: // Left
			Robot::manipulatorArm->setShoulderandElbow(atshoulder,atelbow);
			//printf("Left Sh=%f, El=%f\n\r",atshoulder,atelbow);
		break;
	} */
	//Robot::climber->testMovement();

	frc::Scheduler::GetInstance()->Run();
}

#ifndef RUNNING_FRC_TESTS
int main(int argc, char** argv) {
    return frc::StartRobot<Robot>();
}
#endif
