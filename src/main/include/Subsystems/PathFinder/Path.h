/**************************************************************************************************
 * Program: Woodland 4678 Path finding calculator 
 * Purpose: Calculate the path through waypoints and the required trajectory
 * Author: William Veldhuis
 * Version: 0
 * Date: 2019
 * Changes: Initial Release
**************************************************************************************************/

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "frc/WPILib.h"

#define MAX_WAYPOINTS   10  //We don't like using malloc and other dynamic memory, this is the max 
                            //array size for the number of waypoints you can have in a single path
                            //Also the number of splines that can be used


class PathFinder {
private:
    //Basic configuation for the path finder
    typedef struct Configuration {
        double cycleTime;   //Number of seconds between cycles, normally this is just 0.02 which is = to 50Hz
        double MaxSpeed;    //The max speed of the robot
        double MaxAccel;    //The max acceleration of the robot
        double MaxJerk;
        double WheelBase;
        //Could add unit type mm / m / in / feet etc...
    }tpConfig;

    typedef struct WayPoint {
        double x;
        double y;
        double theta;
    }tpWay;

    typedef struct SplineObject {
        int s_way;
        int e_way;

        double x_offset;
        double y_offset;

        double distance;
        double theta_offset;

        double theta_S_hat;
        double theta_E_hat;

        double m_S_Hat;
        double m_E_Hat;

        double a;
        double b;
        double c;
        double d;
        double e;

        double arcLength;
    }tpSpline;

    typedef struct TrajectorySegment{
        double pos;
        double vel;
        double acc;
        double jerk; 
        double heading;
        double dt;
        double x;
        double y;
    }tpSeg;

    typedef struct TrajectorObject {
        tpSeg segments[500];
        int seg_cnt;
    }tpTrajectory;

    tpConfig m_Config;

    tpWay m_WayPoints[MAX_WAYPOINTS];
    int m_WayPoint_Cnt = 0;

    tpSpline m_Splines[MAX_WAYPOINTS - 1];
    int m_Spline_Cnt = 0;

    tpTrajectory m_Traj;
    tpTrajectory m_L_Traj;
    tpTrajectory m_R_Traj;
    int m_segmentCount = 0;

    double total_dist = 0;
    int m_traverseCount = 0;

    bool generateSpline(int idx, int way1, int way2);


public:
    PathFinder(double cycleTime, double maxSpeed, double maxAccel, double maxJerk, double wheelBase);

    bool createNewPath(); //Resets the counts
    bool addWayPoint(double x, double y, double theta);
    
    bool makePath();
    void debug();

    double spline_CalculateLength(int idx, int samples);
    double spline_derivativeAt(int idx, double percentage);
    double spline_SecondDerivativeAt(int idx, double percentage);
    double spline_getPercentageForDistance(int idx, double distance, int samples);
    bool spline_getXY(int idx, double percentage, double *outX, double *outY);
    double spline_ValueAt(int idx, double percentage);
    double angleAt(int idx, double percentage);

    bool tra_FormTrajectory(double startPower, int wayStart, double endPower, int wayEnd);
    void copySegment(int idx);

    bool traverse(int cnt, double encPrev_Right, double encPrev_Left, double *rightOut, double *leftOut);


    double angleDiffRadians(double from, double to);
};

#endif //PATHFINDER_H