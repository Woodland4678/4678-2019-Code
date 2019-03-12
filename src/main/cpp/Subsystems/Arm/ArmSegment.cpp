
#include "Subsystems/ManipulatorArm.h"
#include "ctre/phoenix.h"
#include "rev/CANSparkMax.h"
#include "Subsystems/Arm/ArmSegment.h"
#include "math.h"

//Constructor
ArmSegment::ArmSegment(std::shared_ptr<WPI_TalonSRX> controller, std::shared_ptr<frc::AnalogInput> pot, double len, ArmSegment *parent, double x, double y) {
	m_physicalAttributes.posX = x;
	m_physicalAttributes.posY = y;
	m_physicalAttributes.length = len;
	
	m_Controller = controller;
	m_potentiometer = pot;
	m_Parent = parent;
	
	setCurrentPeakLimit(0, false);
	setVoltageLimit(12, true);
    setSensorDirection(false);
	setPID(1,0,0);
	setCloseLoopError(5);
	setFeedbackSensorType(FeedbackDevice::QuadEncoder);
	setFramePeriod(StatusFrame::Status_1_General_, 10);
	
}

ArmSegment::ArmSegment(std::shared_ptr<rev::CANSparkMax> controller, std::shared_ptr<frc::AnalogInput> pot, double len, ArmSegment *parent, double x, double y) {
	m_Talon = false;
	m_physicalAttributes.posX = x;
	m_physicalAttributes.posY = y;
	m_physicalAttributes.length = len;
	
	m_ControllerREV = controller;
	m_potentiometer = pot;
	m_Parent = parent;
	
	setCurrentPeakLimit(0, false);
    setSensorDirection(false);
	setPID(1,0,0);
	m_ControllerREV->GetPIDController().SetOutputRange(-1,1);
}

void ArmSegment::updateAngles() {
	//Angles
	m_physicalAttributes.relAngle = convertEncoderToAngle(getSelectedSensorValue());
	m_physicalAttributes.absAngle = m_calculateAbsAngle();
	
	m_physicalAttributes.endPosX = m_physicalAttributes.posX + m_physicalAttributes.length * std::cos(m_physicalAttributes.absAngle * (M_PI/180));
	m_physicalAttributes.endPosY = m_physicalAttributes.posY + m_physicalAttributes.length * std::sin(m_physicalAttributes.absAngle * (M_PI/180));
}

bool ArmSegment::initEncoderValues() {
	if((!m_EncSet)||(!m_AngSet)||(!m_PotSet))
		return false;
	
	//Creating conversions from encoder to pot
	m_ConversionValues.convPEM = (m_ConversionValues.potLimit1 - m_ConversionValues.potLimit2) / (m_ConversionValues.encLimit1 - m_ConversionValues.encLimit2);
	m_ConversionValues.convPEY = m_ConversionValues.potLimit2 - m_ConversionValues.encLimit2 * m_ConversionValues.convPEM;

	int pot = getPotentiometerReading();
	double ang = convertPotToAngle((double)pot);
	double enc = convertAngleToEncoder(ang);

	m_Offset = getSelectedSensorValue() - enc;

	printf("\n%i | %f | %f | %f\n",pot,ang,enc,m_Offset);

	/*setSelectedSensorValue(enc);

	//Check value
	int cnt = 0;
	while((fabs(getSelectedSensorValue() - enc) > 10.0)&&(cnt < 50)) {
		setSelectedSensorValue(enc);
		cnt++;
	}
	printf(" | %i\n",cnt);
	if(cnt >= 50)
		return false;*/
	return true;
}

//Set Functions
void ArmSegment::setEncoderValues(double enc1, double ang1, double enc2, double ang2) {
	m_ConversionValues.encLimit1 = enc1;
	m_ConversionValues.angLimit1 = ang1;
	m_ConversionValues.encLimit2 = enc2;
	m_ConversionValues.angLimit2 = ang2;
	
	//Slope Calculations
	m_ConversionValues.convM = (ang1 - ang2) / (enc1 - enc2);
	m_ConversionValues.convY = ang2 - enc2 * m_ConversionValues.convM;
	
	m_EncSet = true;
	m_AngSet = true;
	
}
void ArmSegment::setPotValues(double pot1, double ang1, double pot2, double ang2) {
	m_ConversionValues.potLimit1 = pot1;
	m_ConversionValues.angLimit1 = ang1;
	m_ConversionValues.potLimit2 = pot2;
	m_ConversionValues.angLimit2 = ang2;
	
	//Slope Calculations
	m_ConversionValues.convPAM = (ang1 - ang2) / (pot1 - pot2);
	m_ConversionValues.convPAY = ang2 - pot2 * m_ConversionValues.convPAM;
	
	m_PotSet = true;
	m_AngSet = true;
}
void ArmSegment::setAbsAngleTarget(double ang1){
	m_physicalAttributes.absAngleTarget = ang1;
}
void ArmSegment::setPhysicalAttributes(double x, double y) {
	m_physicalAttributes.posX = x;
	m_physicalAttributes.posY = y;
}
void ArmSegment::set(ctre::phoenix::motorcontrol::ControlMode mode, double value) {
	m_Controller->Set(mode, value + m_Offset);
}

void ArmSegment::set(rev::ControlType mode, double value) {
	m_ControllerREV->GetPIDController().SetReference(value + m_Offset, mode);
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
double ArmSegment::getSelectedSensorValue() {
	if(m_Talon)
		return m_Controller->GetSelectedSensorPosition(0)-m_Offset;
	else
		return m_ControllerREV->GetEncoder().GetPosition()-m_Offset;
}
double ArmSegment::getCurrent() {
	if(m_Talon)
		return m_Controller->GetOutputCurrent();
	else
		return m_ControllerREV->GetOutputCurrent();
}
int ArmSegment::getPotentiometerReading() {
	return m_potentiometer->GetValue();
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
double ArmSegment::getAbsAngleTarget() {
	return m_physicalAttributes.absAngleTarget;
}
double ArmSegment::getLength() {
	return m_physicalAttributes.length;
}

//Conversions
double ArmSegment::convertAngleToEncoder(double angle) {
	return (((angle) - (m_ConversionValues.convY))/(m_ConversionValues.convM));
}
double ArmSegment::convertEncoderToAngle(double encoder) {
	return (encoder * m_ConversionValues.convM + m_ConversionValues.convY);
}

double ArmSegment::convertAngleToPot(double angle) {
	return (((angle) - (m_ConversionValues.convPAY))/(m_ConversionValues.convPAM));
}
double ArmSegment::convertPotToAngle(double pot) {
	return (pot * m_ConversionValues.convPAM + m_ConversionValues.convPAY);
}

double ArmSegment::convertPotToEncoder(double pot) {
	return (((pot) - (m_ConversionValues.convPEY))/(m_ConversionValues.convPEM));
}
double ArmSegment::convertEncoderToPot(double encoder) {
	return (encoder * m_ConversionValues.convPEM + m_ConversionValues.convPEY);
}

void ArmSegment::setConvSlope_Pot(double value) {
	m_ConversionValues.convPAM = value;
}

bool ArmSegment::setAng(double ang) {
	int enc = (int)convertAngleToEncoder(ang);
	setSelectedSensorValue(enc);

	//Check value
	int cnt = 0;
	while((getSelectedSensorValue() != enc)&&(cnt < 50)) {
		setSelectedSensorValue(enc);
		cnt++;
	}
	if(cnt >= 50)
		return false;
	return true;
}

double ArmSegment::getConversionSlope_Encoder() {
	return m_ConversionValues.convM;
}
double ArmSegment::getConversionIntercept_Encoder() {
	return m_ConversionValues.convY;
}
double ArmSegment::getConversionSlope_Pot() {
	return m_ConversionValues.convPAM;
}
double ArmSegment::getConversionIntercept_Pot() {
	return m_ConversionValues.convPAY;
}

//Calibrate
void ArmSegment::setSkipChecks(int skipAngle, int potSkip){
	m_skipPotDiff = potSkip;
	m_skipAngle = skipAngle;
}

double ArmSegment::getAngleAccuracy() {
	double enc = getSelectedSensorValue();
	int pot = getPotentiometerReading();

	double eAngle = convertEncoderToAngle(enc);
	double pAngle = convertPotToAngle(pot);

	//What is the difference between enc angle and pot angle?
	return fabs(eAngle - pAngle);
	
}
double ArmSegment::getPotAccuracy() {
	double enc = getSelectedSensorValue();
	int pot = getPotentiometerReading();

	double ePot = convertEncoderToPot(enc);
	return fabs(ePot - pot);
}

bool ArmSegment::calibrate() {
	switch(m_calCase)
		{
		case 0:
			m_CalPotCnt = 0;
			m_calCase++;
			break;
		case 1: //Get pot readings
			m_CalPots[m_CalPotCnt] = getPotentiometerReading();
			m_CalPotCnt++;
			if(m_CalPotCnt > 8)
				m_calCase++;
			break;
		case 2: {//Parse pot readings - Calibrate
			double avPot = 0;
			int h_Idx = 0, h_Val = 0, l_Idx = 0, l_Val = 9999;
			for(int x=0;x<8;x++) {
				if(m_CalPots[x] < l_Val) {
					l_Val = m_CalPots[x];
					l_Idx = x;
				}
				if(m_CalPots[x] > h_Val) {
					h_Val = m_CalPots[x];
					h_Idx = x;
				}
			}

			for(int x=0;x<8;x++) {
				if((x != h_Idx)&&(x != l_Idx))
					avPot += m_CalPots[x];
			}
			avPot /= 6;

			//printf("\nPot = %f", avPot);

			double ang = convertPotToAngle(avPot);
			double enc = convertAngleToEncoder(ang);

			m_Offset = (getSelectedSensorValue()+m_Offset) - enc;
			printf("\nOffset = %f | ang = %f | pot = %f", m_Offset, ang, avPot);
			m_calCase = 0;
			return true;
			}
			break;
		}
return false;
}

//Initialize functions for Talon SRX
void ArmSegment::setPID(double p, double i, double d){
	if(m_Talon) {
		m_Controller->Config_kP(0, p, 0);
		m_Controller->Config_kI(0, i, 0);
		m_Controller->Config_kD(0, d, 0);
	}
	else {
		m_ControllerREV->GetPIDController().SetP(p,0);
		m_ControllerREV->GetPIDController().SetI(i,0);
		m_ControllerREV->GetPIDController().SetD(d,0);
	}
}
void ArmSegment::setVoltageLimit(bool enable, double level){
	m_Controller->ConfigVoltageCompSaturation(level, 0);
	m_Controller->EnableVoltageCompensation(enable);
}
void ArmSegment::setCurrentPeakLimit(bool enable, double level){
	if(m_Talon) {
		m_Controller->ConfigPeakCurrentLimit(level, 0);
		m_Controller->EnableCurrentLimit(enable);
	}
	else {
		m_ControllerREV->SetSmartCurrentLimit((unsigned int)level);
	}
}
void ArmSegment::setSensorDirection(bool direction){
	if(m_Talon)
		m_Controller->SetSensorPhase(direction);
	else
		m_ControllerREV->SetInverted(direction);
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
void ArmSegment::setSelectedSensorValue(double value){
	if(m_Talon)
		m_Controller->SetSelectedSensorPosition((int)value, 0, 0);
	else{
		m_ControllerREV->GetEncoder().SetPosition(value);
	}
}
