#include "Subsystems/Arm/Movement.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <math.h>

ArmMotion::ArmMotion(double startAngle, double endAngle, double duration, int type) {
	m_StartAngle = startAngle;
	m_EndAngle = endAngle;
	m_Duration = duration;
	m_type = type;
	
	m_StartSet = false;
	
	m_Count = 0;
	m_ExpectedCount = duration * 50; //example: in 3 seconds at 50Hz total count should be 150
	
	m_Sharpness = 9;
	setOffset(0.25);
}

//Set Commands
void ArmMotion::setStartTime(double tim) {
	m_StartSet = true;
	m_StartTime = tim;
}
void ArmMotion::setDuration(double tim) {
	m_Duration = tim;
	m_multiplier2 = m_multiplier / (m_Duration * 1000);
}
void ArmMotion::setStartAngle(double angle) {
	m_StartAngle = angle;
}
void ArmMotion::setEndAngle(double angle) {
	m_EndAngle = angle;
}
void ArmMotion::setMoveType(int type) {
	m_type = type;
}

void ArmMotion::setAngleCompleteRange(int range) {
	m_Range = range;
}

void ArmMotion::setOffset(double value) {
	m_offset = value;
	setMultiplier(m_Sharpness);
}
void ArmMotion::setMultiplier(double sharpness) {
	m_Sharpness = sharpness;
	m_multiplier = ((m_Sharpness + m_offset) * 1000);
	//(multOffset / (time*1000))
}

//Get Functions
int ArmMotion::getCount() {
	return m_Count;
}
int ArmMotion::getCountEfficiency() {
	if(m_ExpectedCount != 0)
		return m_Count / m_ExpectedCount;
	return 0;
}
double ArmMotion::getMultiplier() {
	return m_multiplier;
}
double ArmMotion::getTimeFromAngle(double angle) {
	return invSigmod(angle);
}

//Move Functions
double ArmMotion::calculateNextPoint(double current) {
	if(!m_StartSet)
		return -1;
	m_CurrentTime = current - m_StartTime;
	//printf("%f | a = %f\n",m_CurrentTime, sigmod(m_CurrentTime));
	if(isComplete(current))
		return -1;
	if(m_type == 0)
		return sigmod(m_CurrentTime);
	else
		return linear(m_CurrentTime);
}
bool ArmMotion::isComplete(double current, double currentAngle) {
	if(currentAngle != -1) {
		if(std::abs(currentAngle - m_EndAngle) < m_Range)
			return true;
		return false;
	}
	m_CurrentTime = current - m_StartTime;
	if(m_CurrentTime > m_Duration)
		return true;
	return false;
}


//Calculations
double ArmMotion::sigmod(double tim) {
	return (((m_EndAngle-m_StartAngle)/(1+exp((-(m_multiplier2*tim))+(m_offset + 4))))+m_StartAngle);
	//		(((end-start)/(1+exp((-(mult*x))+offset)))+start);
}
double ArmMotion::invSigmod(double angle) {
	return ((std::log(((m_EndAngle-m_StartAngle)/(angle-m_StartAngle)) - 1) - (m_offset + 4))/(-m_multiplier2)); 
}
double ArmMotion::linear(double tim) {
	return (m_EndAngle - m_StartAngle) * tim / m_Duration + m_StartAngle;
}
double ArmMotion::invlinear(double angle) {
	return m_Duration * ((angle - m_StartAngle)/(m_EndAngle - m_StartAngle));
}













