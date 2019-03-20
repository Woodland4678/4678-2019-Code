// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/Climber.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

Climber::Climber() : frc::Subsystem("Climber") {

    stilts.reset(new WPI_TalonSRX(7));
    stilts->EnableCurrentLimit(false);
    stilts->ConfigVoltageCompSaturation(12, 0);
    stilts->EnableVoltageCompensation(true);
    stilts->SetSensorPhase(true);
    stilts->Config_kP(0, 1, 0);
    stilts->Config_kI(0, 0, 0);
    stilts->Config_kD(0, 0, 0);
    stilts->ConfigAllowableClosedloopError(0, 5, 0);
    stilts->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
    stilts->SetStatusFramePeriod(StatusFrame::Status_1_General_, 10, 0);


    int pos = readABSEncoder() % 4096;
    if(pos < 0)
        pos += 4096;
    frc::SmartDashboard::PutNumber("Position Pos ", pos);
    
    int cnt = 0;
	while((fabs(readTalonSRXEncoder() - 0) > 10.0)&&(cnt < 50)) {
		stilts->SetSelectedSensorPosition(0);
		cnt++;
	}
    
    m_StartPosition =  0;

    m_encoderPerInch = -1250;
    m_climbCase = 0;

    //stilts->Set(ControlMode::Position, m_StartPosition);
    
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}

void Climber::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Climber::Periodic() {
    // Put code here to be run every loop
    frc::SmartDashboard::PutNumber("Position Quad ", readTalonSRXEncoder());
    frc::SmartDashboard::PutNumber("Position ABS ", readABSEncoder());
    frc::SmartDashboard::PutNumber("Climber Amps ", stilts->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Climber Volts ", stilts->GetBusVoltage());
}

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

int Climber::readTalonSRXEncoder() {
    return stilts->GetSelectedSensorPosition(0);
}

int Climber::readABSEncoder() {
    return stilts->GetSensorCollection().GetPulseWidthPosition();
}

bool Climber::moveTo(int step) {
    static int pos = 0;
    pos -= step;
    if(pos > -29700) //Never go beyond this
        stilts->Set(ControlMode::Position, pos);
    
    printf("\nStilts = %i",pos);
    if(pos <= -29700)
        return true;
    return false;
}

bool Climber::moveInches(int inches, double inchesPerSecond) {
    if((m_target != inches)&&(m_climbCase != 0))
        m_climbCase = 0;
    switch(m_climbCase) {
        case 0://Initialize and pre calculations
            m_inchPerCycle = inchesPerSecond / 50.0;
            m_EncoderStep = (m_encoderPerInch) * m_inchPerCycle;
            m_currentEncoder = 0;
            m_cyclecnt = 0;

            m_cycles = fabs((inches) / m_inchPerCycle) + 1;

            m_target = inches;
            m_climbCase = 1;
            break;
        case 1: //Movement
            printf("\n%f | %f | %f | %f",inchesPerSecond, m_inchPerCycle, m_EncoderStep, m_cycles);
            m_currentEncoder += m_EncoderStep;
            stilts->Set(ControlMode::Position, m_currentEncoder);

            
            if((m_cyclecnt > m_cycles)){
                m_climbCase = 0;
                return true;
            }
            m_cyclecnt++;
            break;
    }
    return false;
}

void Climber::reset() {
    stilts->Set(ControlMode::Position, -200);
}

void Climber::testMovement(){
    //stilts->Set(ControlMode::Velocity, 0.5);
}