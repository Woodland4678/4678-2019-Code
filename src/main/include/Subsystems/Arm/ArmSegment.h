#ifndef ARMSEGMENT_H
#define ARMSEGMENT_H

#include "frc/WPILib.h"
#include "ctre/phoenix.h"
//#include "Movement.h"

class ArmSegment {
private:
	typedef struct phyAtt {
		double posX;
		double posY;
		double endPosX;
		double endPosY;
		double length;
		
		double quad;
		double absolute;
		double potentiometer;
		
		double absAngle;
		double relAngle;
	} phytp;
	
	typedef struct convertInfo {
		double encLimit1;
		double angLimit1;
		double encLimit2;
		double angLimit2;
		
		double encHome;
		double angHome;
		
		double convM;
		double convY;
	} contp;
	
	std::shared_ptr<WPI_TalonSRX> m_Controller;
	
	phytp m_physicalAttributes;
	contp m_ConversionValues;
	ArmSegment *m_Parent = NULL;
	
	//Internal Functions
	void m_calculateConversions();
	double m_calculateAbsAngle();
	
public:
	ArmSegment(std::shared_ptr<WPI_TalonSRX> controller, double len, ArmSegment *parent = NULL, double x = 0.0, double y = 0.0);
	void updateAngles();
	
	//Set functions
	void setLimits(double encLimit1, double angLimit1, double encLimit2, double angLimit2);
	void setPhysicalAttributes(double x, double y, double len);
	bool set(ctre::phoenix::motorcontrol::ControlMode mode, double value);
	
	//Initialize Set Functions
	void setPID(double p, double i, double d);
	void setVoltageLimit(bool enable, double level);
	void setCurrentPeakLimit(bool enable, double level);
	void setSensorDirection(bool direction);
	void setCloseLoopError(int value);
	void setFeedbackSensorType(ctre::phoenix::motorcontrol::FeedbackDevice device);
	void setFramePeriod(ctre::phoenix::motorcontrol::StatusFrame frame, int valueMS);
	
	//Encoders
	int getQuadEncoderReading();
	int getAbsEncoderReading();
	int getSelectedSensorValue();
	int getPotentiometerReading();
	
	double getRelAngle();
	double getAbsAngle();
	
	double getStartX();
	double getStartY();
	double getEndX();
	double getEndY();
	
	ArmSegment * getParent();
	
	//Conversions
	double getConversionSlope();
	double getConversionIntercept();
	
	double convertAngleToEncoder(double angle);
	double convertEncoderToAngle(double encoder);
	
	//Calibrate
	bool checkAngleAccuracy();
	bool calibrate();
	

	
};



#endif //ARMSEGMENT_H