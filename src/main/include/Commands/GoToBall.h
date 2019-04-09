// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef GOTOBALL_H
#define GOTOBALL_H

// PROJECT INCLUDES
#include "frc/commands/Subsystem.h"
#include "Robot.h"


class GoToBall: public frc::Command {
public:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
	GoToBall();
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:

// VARIABLES
   polarPoint     m_polarBallPt;
   double            m_cartX = 0.0; 
   double            m_cartY = 0.0;
   double         m_angle = 0.0;
   int m_state = 0;
   bool m_Move1 = false;
   bool m_Move2 = false;
   bool m_Move3 = false;
   int m_scanCnt = 0;
   int m_delayState = 0;
   double m_wrist = 0;
   int cnt = 0;

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES
};

#endif
