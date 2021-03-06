// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/DriveRobot.h"
#include <math.h>
#include <frc/SmartDashboard/SmartDashboard.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

DriveRobot::DriveRobot(): frc::Command() {
    //m_pdp = new frc::PowerDistributionPanel(12);

        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drivetrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void DriveRobot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveRobot::Execute() {
    /*frc::SmartDashboard::PutNumber("motor 0", m_pdp->GetCurrent(0));
    frc::SmartDashboard::PutNumber("motor 1", m_pdp->GetCurrent(1));
    frc::SmartDashboard::PutNumber("motor 2", m_pdp->GetCurrent(2));
    frc::SmartDashboard::PutNumber("motor 3", m_pdp->GetCurrent(3));
    frc::SmartDashboard::PutNumber("motor 14", m_pdp->GetCurrent(14));
    frc::SmartDashboard::PutNumber("motor 15", m_pdp->GetCurrent(15));
    frc::SmartDashboard::PutNumber("pdp 5", m_pdp->GetCurrent(5));*/

    // double JoyX = Robot::oi->getdriver()->GetX();
    // double JoyY = Robot::oi->getdriver()->GetY();

// **** steve test
// TODO: If we keep this test code, clean up everything else!!!!
    double JoyX = Robot::oi->getdriver()->GetX();
    double JoyY = Robot::oi->getdriver()->GetY();
    //frc::SmartDashboard::PutNumber("Current Angle: ", Robot::ahrs->GetAngle());
    // Get X and Y values with some deadband.  Subtract/add deadband back in to ensure we
    // start at 0.0 instead of +/-0.05
    if ((JoyX > -0.05)&&(JoyX <= 0.05))
        JoyX = 0.0;
    else if (JoyX < -0.05)
        JoyX += 0.05;
    else
        JoyX -= 0.05;

    if ((JoyY > -0.05)&&(JoyY <= 0.05)) {
        JoyY = 0.0;
        if(Robot::manipulatorArm->m_FineLimitHit)
            JoyY = -0.2;
    }
    else if (JoyY < -0.05)
        JoyY += 0.05;
    else
        JoyY -= 0.05;

    // Use X^2
    double JoyX2 = JoyX * JoyX;
    double JoyY2 = JoyY * JoyY;

    // Correct the sign since x^2 is always positive.
    if (JoyX < 0)
        JoyX2 =-JoyX2;
    if (JoyY < 0)
        JoyY2 = -JoyY2;

    // Calculate Power Value.
    // Multiplier last year was 0.75, but too sensitive this year. 
    // March 9 11:00 am Start for drivers at 0.35 and adjust

    // This may make turning kinda slow.  We should really read
    // current speed and adjust the amount of JoyX2 based on 
    // the speed reading.
    double leftPower = -(JoyY2 - 0.35 * JoyX2);   
    double rightPower = -(JoyY2 + 0.35 * JoyX2);

    //frc::SmartDashboard::PutNumber("left Power: ", leftPower);
   // frc::SmartDashboard::PutNumber("right Power: ", rightPower);

   

    if(!Robot::climber->m_Climbing) {
        if (Robot::oi->getdriver()->GetRawButton(6)) {
            leftPower *= 0.30;
            rightPower *= 0.30;
        }
        Robot::drivetrain->setLeftMotor(leftPower);
        Robot::drivetrain->setRightMotor(rightPower);
    }
    // Steve test ****

    // Deadzone.
    if (fabs(JoyX) < 0.05 && fabs(JoyY) < 0.05)
        JoyX = 0.0;
    else if (JoyX > 0)
        JoyX -= 0.05;
    else
        JoyX += 0.05;

    if (fabs(JoyY) < 0.05)
        JoyY = 0.0;
    else if (JoyY > 0)
        JoyY -= 0.05;
    else
        JoyY += 0.05;
    
}

// Make this return true when this Command no longer needs to run execute()
bool DriveRobot::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void DriveRobot::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveRobot::Interrupted() {}