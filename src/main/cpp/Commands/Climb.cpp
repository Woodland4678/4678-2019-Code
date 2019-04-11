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
#include "../../include/Subsystems/BasicUtilities.h"

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

    //Determine height
    if(m_climbLevel < 2){
        m_ClimbCase = 0;
        if(Robot::oi->getdriver()->GetRawButton(9))
            m_climbLevel = 3;
        else
            m_climbLevel = 4;
        /*switch (m_ClimbCase) {
            case 0:
                Robot::lidar->readLidar();
                m_ClimbCase = 1;
                break;
            case 1:
                if(Robot::lidar->readComplete())
                    m_ClimbCase = 2;
                break;
            case 2:
                {
                int dist = Robot::lidar->climbDistance();
                m_ClimbCase = 0;
                if(dist < 300)
                    m_climbLevel = 3;
                else
                    m_climbLevel = 4;
                }
                break;
        }*/
    }
    else {
        m_timer++;
        switch (m_ClimbCase) {
            case 0: //Move arm up above the platform
                if(m_climbLevel == 3)
                    done2 = Robot::manipulatorArm->moveToXY(A_C_START_X,A_CH_START_Y,A_C_WRIST,0,20.0); //41
                else
                    done2 = Robot::manipulatorArm->moveToXY(A_C_START_X,A_CL_START_Y,A_C_WRIST,0,20.0); //27
                if(done2)
                    m_ClimbCase = 1;
                break;
            case 1: //Move arm down on the platform
                if(m_climbLevel == 3)
                    done2 = Robot::manipulatorArm->moveToXY(A_C_START2_X,A_CH_START2_Y,A_C_WRIST2,0,20.0); //27
                else
                    done2 = Robot::manipulatorArm->moveToXY(A_C_START2_X,A_CL_START2_Y,A_C_WRIST2,0,20.0); //14, 14
                if(done2){
                    m_ClimbCase = 2;
                    done2 = false;
                    done3 = false;
                    Robot::manipulatorArm->disableWrist();
                }
                break;
            case 2://Move the robot up
                if(!done2)
                    done2 = Robot::manipulatorArm->moveToXY(A_C_X,A_C_Y,A_C_WRIST2,0,18.0);//7
                if(!done3){
                    if(m_climbLevel == 3)
                        done3 = Robot::climber->moveTo(550,-29700);
                    else
                        done3 = Robot::climber->moveTo(550,-10800);
                }
                if(done2 && done3)
                    m_ClimbCase = 3;
                m_timer = 0;
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
                printf("Dist = %i | Timer = %i\n",dist,m_timer);
                m_ClimbCase = 3;
                if(m_climbLevel == 4){
                    //if(dist < 1500){ //Time to spin up wheel
                    Robot::drivetrain->setRightMotor(0.2);
                    Robot::drivetrain->setLeftMotor(0.2);
                    //Robot::manipulatorArm->intakeWheelsSpin(0);
                    //}
                    //if(dist > 720) //Spin intakes
                        Robot::manipulatorArm->intakeWheelsSpin(-1);
                    if((m_timer > 125))
                        m_ClimbCase = 11;
                }
                else{
                    //if(dist < 1500){ //Time to spin up wheel
                        Robot::drivetrain->setRightMotor(0.1);
                        Robot::drivetrain->setLeftMotor(0.1);
                        //Robot::manipulatorArm->intakeWheelsSpin(0);
                    //}
                    //if(dist > 1000) //Spin intakes
                        Robot::manipulatorArm->intakeWheelsSpin(-1);
                    if((m_timer > 125))
                        m_ClimbCase = 11;
                }
                
                break;
            case 11:
                if(Robot::climber->unlock())
                    m_ClimbCase = 6;
                break;
            case 6:
                //The middle wheel is now on
                Robot::drivetrain->setRightMotor(0);
                Robot::drivetrain->setLeftMotor(0);
                Robot::manipulatorArm->intakeWheelsSpin(0);
                Robot::climber->reset();
                m_ClimbCase = 7;
                break;
            case 7:
                if(!done5)
                    done5 = Robot::manipulatorArm->moveToXY(A_CH_UP_X,A_C_UP_Y,A_C_WRIST2,0,20.0);//24
                else
                    done6 = Robot::manipulatorArm->moveToXY(A_CL_UP_X,A_C_UP_Y,A_C_WRIST2,0,10.0);//24
                if(done6)
                    m_ClimbCase = 8;
                break;
            case 8:
                if(Robot::climber->readTalonSRXEncoder() < 400) {
                    Robot::drivetrain->setRightMotor(0.08);
                    Robot::drivetrain->setLeftMotor(0.08);
                    Robot::lidar->readLidar();
                    m_ClimbCase = 9;
                }
                break;
            case 9:
                if(Robot::lidar->readComplete())
                    m_ClimbCase = 10;
                break;
            case 10:
                dist = Robot::lidar->climbDistance();
                printf("\nDist = %i",dist);
                if(dist < 500){
                    Robot::drivetrain->setRightMotor(0);
                    Robot::drivetrain->setLeftMotor(0);
                    done = true;
                    //done = Robot::manipulatorArm->moveToXY((7.0,29,-10.0,0,20);
                    //if(done)
                    //    Robot::climber->m_climbed = true;
                }
                else
                    m_ClimbCase = 8;
                break;
        }
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
