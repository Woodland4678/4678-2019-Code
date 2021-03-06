// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/LineUpToRocket.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

LineUpToRocket::LineUpToRocket(int side): frc::Command() {
    m_side = side;

        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drivetrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void LineUpToRocket::Initialize() {
    Robot::drivetrain->initAutoScore();
    m_state = 0;
    done = false;
}

// Called repeatedly when this Command is scheduled to run
void LineUpToRocket::Execute() {
    if(!Robot::oi->getdriver()->GetRawButton(7)&&!Robot::oi->getdriver()->GetRawButton(2)&&!Robot::oi->getdriver()->GetRawButton(3)&&!Robot::oi->getdriver()->GetRawButton(1)&&!Robot::oi->getdriver()->GetRawButton(4))
        {
        done = true;
        return;
        }
    
    switch(m_state){
        case 0:
            {
            int pov = m_side;
            if(pov == 0)
                m_target = 3; //Rocket high level autoscore
            else if(pov == 1)
                m_target = 2; //Rocket medium level autoscore
            else if(pov == 2)
                m_target = 1; //Rocket low level autoscore
            else if(pov == 3)
                m_target = 4; //Cargo ground pickup
            else 
                done = true;
            m_state = 1;
            }
            break;
        case 1:
            switch(m_target) {
                case 1:
                case 2:
                case 3:
                    done = Robot::drivetrain->autoScore(m_target);
                    break;
                case 4:
                    done = Robot::drivetrain->getNearestBall();

                    break;
            }
            break;
    }
    
}

// Make this return true when this Command no longer needs to run execute()
bool LineUpToRocket::IsFinished() {
    return done;
}

// Called once after isFinished returns true
void LineUpToRocket::End() {
    Robot::manipulatorArm->grabHatch();
    Robot::manipulatorArm->intakeWheelsSpin(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LineUpToRocket::Interrupted() {
    
}
