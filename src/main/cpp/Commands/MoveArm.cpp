// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/MoveArm.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

MoveArm::MoveArm(int btn): frc::Command() {
    m_btn = btn;
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::manipulatorArm.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void MoveArm::Initialize() {
	if (m_btn == 0) // when using default command, m_btn is set to 0.
		{
		// Any fine motion init stuff can happen here.
		}
	else
		{
		done = false;
		done2 = false;
		if (Robot::manipulatorArm->getInCargoPosition())
			{ // If we're leaving a cargo intake position, go to mode 2
			Robot::manipulatorArm->clearInCargoPosition(); // at start of any move, clear the InCargoPosition indicator.
			Robot::manipulatorArm->setIntakeMode(2); // Going to Cargo mode will set wrist to cargo rotation.
			}
		if (Robot::manipulatorArm->getInHatchPosition())
			{ // If we're leaving a hatch intake position, go to mode 1
			Robot::manipulatorArm->clearInHatchPosition(); // at start of any move, clear the InHatchPosition indicator.
			Robot::manipulatorArm->setIntakeMode(1); // Going to Hatch mode will set wrist to cargo rotation.
			}
		
		Robot::manipulatorArm->intakeWheelsOff(); // Stop intake wheels at start of any move.
		//printf("Wheels set to off\n\r");
		// Robot::manipulatorArm->grabHatch(); // Hatch clamp is engaged by default at start of any move.
		// No, hatch clamp needs to stay released till will go to carry or clearly back
		// away from a hatch placement.  Setting up for button hold to release.  When button
		// is let go, hatch clamp is re-engaged.
		}
}

// Called repeatedly when this Command is scheduled to run
void MoveArm::Execute() {
	double wristAbsAngle;
	//printf("Got m_btn = %d\n\r",m_btn);
    switch(m_btn) {
		case 0:
			if (Robot::oi->getoperator1()->GetRawButton(5))
				{
				Robot::manipulatorArm->fineMotion(); // default command, do fine motion.S
				}
			if (Robot::oi->getoperator1()->GetPOV() == 180) 
				{
				m_btn = 11;
				}

			break;
		case 1: //Carry
			wristAbsAngle = -10.0; // This is for cargo.
			if(Robot::manipulatorArm->isHatchMode())
				wristAbsAngle = -190.0;
			done = Robot::manipulatorArm->moveToXY(7.0,26.0,wristAbsAngle,0,20.0); // Move to X,Y co-ords
			Robot::manipulatorArm->m_CurrentPosition = 0;
			break;
		case 2: //Rocket Low, Cargo ship place hatch.
			if(Robot::manipulatorArm->isHatchMode())
				done = Robot::manipulatorArm->moveToXY(25.5,17,-190.0,0,20.0);
			else
				done = Robot::manipulatorArm->moveToXY(25.5,27.5,10.0,0,20.0);
			//done = Robot::manipulatorArm->moveToXY(25.5,19,20.0);
			Robot::manipulatorArm->m_CurrentPosition = 0;
			break;
		case 3: //Rocket Medium
			if(Robot::manipulatorArm->isHatchMode())
				done = Robot::manipulatorArm->moveToXY(16.0,46.0,-190,0,20.0);
			else
				done = Robot::manipulatorArm->moveToXY(16.0,55.0,10.0,0,20.0);
			Robot::manipulatorArm->m_CurrentPosition = 0;
			break;
		case 4: //Rocket High
			if(Robot::manipulatorArm->isHatchMode())
				done = Robot::manipulatorArm->moveToXY(10.0,74.0,-190,0,20.0);
			else
				{
				done = Robot::manipulatorArm->moveToXY(16.0,75.0,68.5,0,20.0);
				}
			Robot::manipulatorArm->m_CurrentPosition = 0;
			break;
		case 5: //Fine Motion
			done = Robot::manipulatorArm->moveToXY(22.0,75.0,-100,0,20.0);			
			break;
		case 6:	//Pick up - Cargo Ground Intake.
			if(!Robot::manipulatorArm->ifHatch()){
				// Set AbsTargetAngle to -28.41 (Rel is 38.83)
				if (!done2) // Stay high to ensure we don't high robot.
					{
					done2 = Robot::manipulatorArm->moveToXY(25.68,24,-28.41,0,20.0); 
					if (done2)
						{
						Robot::manipulatorArm->setInCargoPosition();
						Robot::manipulatorArm->intakeWheelsSpin(-0.7); // Wheels running.
						printf("Wheels set to run 2\n\r");
						}
					}
				else
					{
					done = Robot::manipulatorArm->moveToXY(25.68,13,-28.41,0,20.0);
					if (done)
						{
						Robot::manipulatorArm->setInCargoPosition();
						Robot::manipulatorArm->intakeWheelsSpin(-0.7); // Wheels running.
						printf("Wheels set to run 1\n\r");
						}
					}
				Robot::manipulatorArm->m_CurrentPosition = 5;
			}
			break;
		case 7: //Human Station Hatch
			if (Robot::manipulatorArm->ifCargo() == false) {
				if (!done2) // Stay high to ensure we don't hit robot.
					done2 = Robot::manipulatorArm->moveToXY(25.68,25,-190,0,20.0); 
				else
					{
					Robot::manipulatorArm->setInHatchPosition();
					done = Robot::manipulatorArm->moveToXY(25.5,17.0,-190.0,0,20.0);
					}
				Robot::manipulatorArm->m_CurrentPosition = 5;
			}
			else {
				done = true;
			}
			break;
		case 8: //Human Station Cargo
			if(!Robot::manipulatorArm->ifHatch()){
				done = Robot::manipulatorArm->moveToXY(16.0,44.0,10.0,0,20.0);
				if (done)
					{
					Robot::manipulatorArm->setInCargoPosition();
					// Start intake rollers.  Move to any other position will stop them.
					Robot::manipulatorArm->intakeWheelsSpin(-0.5); // Wheel running.
					}
				Robot::manipulatorArm->m_CurrentPosition = 5;
			}
			else {
				done = true;
			}
			
			break;
		case 9: //Swap Intake - no longer needed.
			// Robot::manipulatorArm->swapIntakes();
			done = true;
			break;
		case 10: //Cargo Ship
			if(Robot::manipulatorArm->isHatchMode())
				done = Robot::manipulatorArm->moveToXY(25.5,19.0,-190,0,20.0);
			else
				done = Robot::manipulatorArm->moveToXY(18.0,42.0,-10,0,20.0);
			Robot::manipulatorArm->m_CurrentPosition = 0;
			break;
		case 11:
			if(!done2){
				done2 = Robot::manipulatorArm->moveToXY(16.0,46.0,-190,0,20.0);
				Robot::manipulatorArm->InitCalibrate();
			}
			else 
				{
				//printf("\nHere");
				done = Robot::manipulatorArm->Calibrate();
				if(done){
					printf("\nhere");
					m_btn = 0;
				}
				}
			break;
	}
}


// Make this return true when this Command no longer needs to run execute()
bool MoveArm::IsFinished() {
    return done;
}

// Called once after isFinished returns true
void MoveArm::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveArm::Interrupted() {

}
