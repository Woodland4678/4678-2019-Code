// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Climb.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

Climb::Climb(int climbLevel): frc::Command() {
    m_climbLevel = climbLevel;
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drivetrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

    done = false;
	done2 = false;
    done3 = false;
}
// Called just before this Command runs the first time
void Climb::Initialize() {
    m_ClimbCase = 0;
    Robot::climber->m_Climbing = true;
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
    //Make sure the intakes are empty
    if(Robot::manipulatorArm->ifCargo()||Robot::manipulatorArm->ifHatch()){
        done = true;
        return;
    }
    //Make sure the climbing button is always held down
    if((!Robot::oi->getdriver()->GetRawButton(9))&&(!Robot::oi->getdriver()->GetRawButton(10))){
        //William: Small problem what to do if the arm and stilts are partly down
        done = true;
        return;
    }

    printf("Cycle: Case = %i ", m_ClimbCase);

    switch (m_ClimbCase) {
        case 0: //Move arm up above the platform
            done2 = Robot::manipulatorArm->moveToXY(25.0,40.0,-90.0,0,20.0);
            if(done2)
                m_ClimbCase = 1;
            break;
        case 1: //Move arm down on the platform
            done2 = Robot::manipulatorArm->moveToXY(33.0,30.0,-93.0,0,20.0);
            if(done2){
                m_ClimbCase = 2;
                done2 = false;
                done3 = false;
                Robot::manipulatorArm->disableWrist();
            }
            break;
        case 2://Move the robot up
            if(!done2)
                done2 = Robot::manipulatorArm->moveToXY(33.0,9,-93.0,0,10.0);
            if(!done3)
                done3 = Robot::climber->moveTo(400);
            if(done2 && done3)
                m_ClimbCase = 3;
            break;
        case 3://Move forward using intakes
            //Use lidar to detect how far away we are
            Robot::lidar->readLidar();
            m_ClimbCase = 4;
            break;
        case 4:
            if(Robot::lidar->readComplete())
                m_ClimbCase = 5;
            break;
        case 5:
            dist = Robot::lidar->climbDistance();
            printf("Dist = %i",dist);
            m_ClimbCase = 3;
            if(dist < 1500){ //Time to spin up wheel
                Robot::drivetrain->setRightMotor(0.2);
                Robot::drivetrain->setLeftMotor(0.2);
                //Robot::manipulatorArm->intakeWheelsSpin(0);
            }
            if(dist > 1000) //Spin intakes
                Robot::manipulatorArm->intakeWheelsSpin(-1);
            if(dist < 900)
                m_ClimbCase = 6;
            break;
        case 6:
            //The middle wheel is now on
            printf("STOP!\n");
            Robot::drivetrain->setRightMotor(0);
            Robot::drivetrain->setLeftMotor(0);
            Robot::manipulatorArm->intakeWheelsSpin(0);
            Robot::climber->reset();
            m_ClimbCase = 7;
            break;
        case 7:
            
            break;
    }

    printf("\n");
	
    
}

// Make this return true when this Command no longer needs to run execute()
bool Climb::IsFinished() {
    return done;
}
// Called once after isFinished returns true
void Climb::End() {
    Robot::climber->m_Climbing = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Climb::Interrupted() {

}
