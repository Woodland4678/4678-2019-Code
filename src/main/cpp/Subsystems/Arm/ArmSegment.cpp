
#include "Subsystems/ManipulatorArm.h"
#include "ctre/phoenix.h"
#include "Subsystems/Arm/ArmSegment.h"

//Constructor
ArmSegment::ArmSegment(std::shared_ptr<WPI_TalonSRX> controller, double len, ArmSegment *parent, double x, double y) {
	m_physicalAttributes.posX = x;
	m_physicalAttributes.posY = y;
	m_physicalAttributes.length = len;
	
	m_Controller = controller;
	m_Parent = parent;
	
	setCurrentPeakLimit(0, false);
	setVoltageLimit(12, true);
    setSensorDirection(false);
	setPID(1,0,0);
	setCloseLoopError(5);
	setFeedbackSensorType(FeedbackDevice::QuadEncoder);
	setFramePeriod(StatusFrame::Status_1_General_, 10);
	
}

void ArmSegment::updateAngles() {
	//Angles
	m_physicalAttributes.relAngle = convertEncoderToAngle(getSelectedSensorValue());
	m_physicalAttributes.absAngle = m_calculateAbsAngle();
	
	m_physicalAttributes.endPosX = m_physicalAttributes.posX + m_physicalAttributes.length * std::cos(m_physicalAttributes.absAngle * (M_PI/180));
	m_physicalAttributes.endPosY = m_physicalAttributes.posY + m_physicalAttributes.length * std::sin(m_physicalAttributes.absAngle * (M_PI/180));
}

//Set Functions
void ArmSegment::setLimits(double encLimit1, double angLimit1, double encLimit2, double angLimit2) {
	m_ConversionValues.encLimit1 = encLimit1;
	m_ConversionValues.angLimit1 = angLimit1;
	m_ConversionValues.encLimit2 = encLimit2;
	m_ConversionValues.angLimit2 = angLimit2;
	
	//Slope Calculations
	m_ConversionValues.convM = (angLimit1 - angLimit2) / (encLimit1 - encLimit2);
	m_ConversionValues.convY = angLimit2 - encLimit2 * m_ConversionValues.convM;
	
}
void ArmSegment::setPhysicalAttributes(double x, double y, double len) {
	m_physicalAttributes.posX = x;
	m_physicalAttributes.posY = y;
	m_physicalAttributes.length = len;
}
bool ArmSegment::set(ctre::phoenix::motorcontrol::ControlMode mode, double value) {
	m_Controller->Set(mode, value);
}

//Calculations
double ArmSegment::m_calculateAbsAngle() {
	double ang = getRelAngle();
	ArmSegment *par = m_Parent;
	while(par != NULL) {
		ang += par->getRelAngle();
		par = par->getParent();
	}
	return ang;
}

//Get Functions
int ArmSegment::getQuadEncoderReading() {
	return m_Controller->GetSensorCollection().GetQuadraturePosition();
}
int ArmSegment::getAbsEncoderReading() {
	return m_Controller->GetSensorCollection().GetPulseWidthPosition();
}
int ArmSegment::getSelectedSensorValue() {
	return m_Controller->GetSelectedSensorPosition(0);
}
int ArmSegment::getPotentiometerReading() {
	return m_Controller->GetSensorCollection().GetAnalogIn();
}

double ArmSegment::getStartX() {
	return m_physicalAttributes.posX;
}
double ArmSegment::getStartY() {
	return m_physicalAttributes.posY;
}
double ArmSegment::getEndX() {
	return m_physicalAttributes.endPosX;
}
double ArmSegment::getEndY() {
	return m_physicalAttributes.endPosY;
}

ArmSegment * ArmSegment::ArmSegment::getParent() {
	return m_Parent;
}

double ArmSegment::getRelAngle() {
	return m_physicalAttributes.relAngle;
}
double ArmSegment::getAbsAngle() {
	return m_physicalAttributes.absAngle;
}

//Conversions
double ArmSegment::convertAngleToEncoder(double angle) {
	return (((angle) - (m_ConversionValues.convY))/(m_ConversionValues.convM));
}
double ArmSegment::convertEncoderToAngle(double encoder) {
	return (encoder * m_ConversionValues.convM + m_ConversionValues.convY);
}

double ArmSegment::getConversionSlope() {
	return m_ConversionValues.convM;
}
double ArmSegment::getConversionIntercept() {
	return m_ConversionValues.convY;
}

//Calibrate
bool ArmSegment::checkAngleAccuracy() {
	
}
bool ArmSegment::calibrate() {
	
}

//Initialize functions for Talon SRX
void ArmSegment::setPID(double p, double i, double d){
    m_Controller->Config_kP(0, p, 0);
    m_Controller->Config_kI(0, i, 0);
    m_Controller->Config_kD(0, d, 0);
}
void ArmSegment::setVoltageLimit(bool enable, double level){
	m_Controller->ConfigVoltageCompSaturation(level, 0);
	m_Controller->EnableVoltageCompensation(enable);
}
void ArmSegment::setCurrentPeakLimit(bool enable, double level){
	m_Controller->ConfigPeakCurrentLimit(level, 0);
	m_Controller->EnableCurrentLimit(enable);
}
void ArmSegment::setSensorDirection(bool direction){
	m_Controller->SetSensorPhase(direction);
}
void ArmSegment::setCloseLoopError(int value){
	m_Controller->ConfigAllowableClosedloopError(0, value, 0);
}
void ArmSegment::setFeedbackSensorType(ctre::phoenix::motorcontrol::FeedbackDevice device) {
	m_Controller->ConfigSelectedFeedbackSensor(device, 0, 0);
}
void ArmSegment::setFramePeriod(ctre::phoenix::motorcontrol::StatusFrame frame, int valueMS) {
	m_Controller->SetStatusFramePeriod(frame, valueMS, 0);
}
