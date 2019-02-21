#ifndef ARMMOVEMENT_H
#define ARMMOVEMENT_H

class ArmMotion {
private:
	double m_StartAngle;
	double m_EndAngle;
	double m_StartTime;
	double m_Duration;
	int m_type;
	int m_Range;
	double m_CurrentTime;
	
	double m_offset;// = 0.25;
	double m_Sharpness;
	double m_multiplier;// = (9 + (m_offset)) * 1000;
	
	int m_Count;
	int m_ExpectedCount;
	
	bool m_StartSet = false;
	
	double sigmod(double tim);
	double invSigmod(double angle);
	double linear(double tim);
	double invlinear(double angle);
	
public:
	ArmMotion(double startAngle, double endAngle, double duration, int type =0 );
	
	//Set functions
	void setStartTime(double tim);
	void setDuration(double tim);
	void setStartAngle(double angle);
	void setEndAngle(double angle);
	void setMoveType(int type);
	void setAngleCompleteRange(int range);
	
	void setOffset(double value);
	void setMultiplier(double sharpness);
	
	//Get functions
	int getCount();
	int getCountEfficiency();
	double getMultiplier();
	
	double getTimeFromAngle(double angle);
	
	//Move Functions
	double calculateNextPoint(double current);
	bool isComplete(double currentAngle = -1);
	
};

#endif //ARMMOVEMENT_H