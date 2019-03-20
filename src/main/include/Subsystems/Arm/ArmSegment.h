#ifndef ARMSEGMENT_H
#define ARMSEGMENT_H

#include "frc/WPILib.h"
#include "ctre/phoenix.h"
#include "rev/CANSparkMax.h"
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

		double absAngleTarget; // What we want abs Angle to be (mostly for wrist, maybe waist eventually)

	} phytp;
	
	typedef struct convertInfo {
		double encLimit1;
		double angLimit1;
		double encLimit2;
		double angLimit2;
		
		double potLimit1;
		double potLimit2;
		
		double encHome;
		double angHome;
		
		double convM;
		double convY;
		double convPAM;
		double convPAY;
		double convPEM;
		double convPEY;
	} contp;
	
	std::shared_ptr<WPI_TalonSRX> m_Controller;
	std::shared_ptr<rev::CANSparkMax> m_ControllerREV;
	std::shared_ptr<frc::AnalogInput> m_potentiometer;
	
	phytp m_physicalAttributes;
	contp m_ConversionValues;
	ArmSegment *m_Parent = NULL;
	
	bool m_EncSet = false;
	bool m_AngSet = false;
	bool m_PotSet = false;
	
	bool m_Talon = true;

	int m_skipAngle = 10;
	int m_skipPotDiff = 200;

	int m_calCase = 0;
	int m_CalPots[8];
	int m_CalPotCnt = 0;
	int m_CalEncs[8];
	int m_CalEncCnt = 0;

	//Internal Functions
	void m_calculateConversions();
	double m_calculateAbsAngle();
	
public:
	ArmSegment(std::shared_ptr<WPI_TalonSRX> controller, std::shared_ptr<frc::AnalogInput> pot, double len, ArmSegment *parent = NULL, double x = 0.0, double y = 0.0);
	ArmSegment(std::shared_ptr<rev::CANSparkMax> controller, std::shared_ptr<frc::AnalogInput> pot, double len, ArmSegment *parent = NULL, double x = 0.0, double y = 0.0);
	void updateAngles();
	bool initEncoderValues();
	
	//Set functions
	void setEncoderValues(double enc1, double ang1, double enc2, double ang2);
	void setPotValues(double pot1, double ang1, double pot2, double ang2);
	void setPhysicalAttributes(double x, double y);
	void set(ctre::phoenix::motorcontrol::ControlMode mode, double value);
	void set(rev::ControlType mode, double value);
	
	//Initialize Set Functions
	void setPID(double p, double i, double d);
	void setVoltageLimit(bool enable, double level);
	void setCurrentPeakLimit(bool enable, double level);
	void setSensorDirection(bool direction);
	void setCloseLoopError(int value);
	void setFeedbackSensorType(ctre::phoenix::motorcontrol::FeedbackDevice device);
	void setFramePeriod(ctre::phoenix::motorcontrol::StatusFrame frame, int valueMS);
	void setSelectedSensorValue(double value);
	void setAbsAngleTarget(double ang1);

	void disableMotor();

	
	//Encoders
	int getQuadEncoderReading();
	int getAbsEncoderReading();
	double getSelectedSensorValue();
	int getPotentiometerReading();
	
	double getRelAngle();
	double getAbsAngle();
	double getLength();

	
	double getStartX();
	double getStartY();
	double getEndX();
	double getEndY();

	double getCurrent();
	double getAbsAngleTarget();

	
	ArmSegment * getParent();
	
	//Conversions
	double getConversionSlope_Encoder();
	double getConversionIntercept_Encoder();
	double getConversionSlope_Pot();
	double getConversionIntercept_Pot();

	void setConvSlope_Pot(double value);
	bool setAng(double ang);
	
	double convertAngleToEncoder(double angle);
	double convertEncoderToAngle(double encoder);
	double convertAngleToPot(double angle);
	double convertPotToAngle(double pot);
	double convertPotToEncoder(double pot);
	double convertEncoderToPot(double encoder);
	
	//Calibrate
	void setSkipChecks(int skipAngle, int potSkip);
	double getAngleAccuracy();
	double getPotAccuracy();
	bool calibrate();
	double getOffsetVal();
	
	void setBrakeMode();
	void setCoastMode();

	double m_Offset = 0;
	
};



#endif //ARMSEGMENT_H