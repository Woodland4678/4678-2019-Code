
#include "../../../include/Subsystems/PathFinder/Path.h"
#include "../../../include/Subsystems/LidarViewer.h"

PathFinder::PathFinder(double cycleTime, double maxSpeed, double maxAccel, double maxJerk, double wheelBase){
    m_Config.cycleTime = cycleTime;
    m_Config.MaxSpeed = maxSpeed;
    m_Config.MaxAccel = maxAccel;
    m_Config.MaxJerk = maxJerk;
    m_Config.WheelBase = wheelBase;

    m_WayPoint_Cnt = 0;
    m_Spline_Cnt = 0;
}

bool PathFinder::createNewPath(){
    m_WayPoint_Cnt = 0;
    m_Spline_Cnt = 0;
    m_segmentCount = 0;
}

bool PathFinder::addWayPoint(double x, double y, double theta){
    if(m_WayPoint_Cnt >= MAX_WAYPOINTS)
        return false; //Maximum number of waypoints reached
    m_WayPoints[m_WayPoint_Cnt].x = x;
    m_WayPoints[m_WayPoint_Cnt].y = y;
    m_WayPoints[m_WayPoint_Cnt].theta = theta * M_PI / 180; //Convert to radians
    m_WayPoint_Cnt++;
    return true;
}

bool PathFinder::makePath(){
    if(m_WayPoint_Cnt == 0)
        return false;
    total_dist = 0;
    for(int i = 0; i<m_WayPoint_Cnt-1;i++) {
        if (!generateSpline(i,i,i+1))
            return false;
        else
            m_Spline_Cnt++;
        total_dist += spline_CalculateLength(i, 10000);
    }

    m_traverseCount = 0;

    bool success = tra_FormTrajectory(0.0, 0, 0.0, m_WayPoint_Cnt-1);

    

    return success;
}

void PathFinder::debug(){
    //Display - Debug
    for(int i = 0; i<m_Spline_Cnt;i++) {
        for(double n = 0;n<=1;n+=m_Config.cycleTime){
            double x,y;
            spline_getXY(i,n,&x,&y);
            //printf("\n(%f,%f)",x,y);
            LidarViewer::Get()->addPointXY(x*1000,y*1000);
        }
    }
    for(int i=0;i<m_segmentCount;i++) {
        double x,y;
        x = m_L_Traj.segments[i].x;
        y = m_L_Traj.segments[i].y;
        LidarViewer::Get()->addPointXY(x*1000,y*1000);
        x = m_R_Traj.segments[i].x;
        y = m_R_Traj.segments[i].y;
        LidarViewer::Get()->addPointXY(x*1000,y*1000);
    }
}

//---Traversing Path---
bool PathFinder::traverse(int cnt, double encPrev_Right, double encPrev_Left, double *rightOut, double *leftOut) {
    //Calculate the target encoder positions for both left and right using segments
    if(cnt >= m_segmentCount)
        return true;

    //printf("\nCount = %i",cnt);
    m_traverseCount = cnt;
    if(m_traverseCount == 0){
        *rightOut = encPrev_Right;
        *leftOut = encPrev_Left;
        return false;
    }

    int last = m_traverseCount - 1;
    double dist = sqrt(((m_L_Traj.segments[m_traverseCount].x - m_L_Traj.segments[last].x)
        *(m_L_Traj.segments[m_traverseCount].x - m_L_Traj.segments[last].x)) 
        + ((m_L_Traj.segments[m_traverseCount].y - m_L_Traj.segments[last].y)
        *(m_L_Traj.segments[m_traverseCount].y - m_L_Traj.segments[last].y)));
    if(m_L_Traj.segments[m_traverseCount].x < 0)
        dist *= -1;
    *leftOut = encPrev_Left + (20.81 * dist);

    //printf("\nLd = %f | %f | %f",dist,encPrev_Left,*leftOut);

    dist = sqrt(((m_R_Traj.segments[m_traverseCount].x - m_R_Traj.segments[last].x)
        *(m_R_Traj.segments[m_traverseCount].x - m_R_Traj.segments[last].x)) 
        + ((m_R_Traj.segments[m_traverseCount].y - m_R_Traj.segments[last].y)
        *(m_R_Traj.segments[m_traverseCount].y - m_R_Traj.segments[last].y)));
    if(m_R_Traj.segments[m_traverseCount].x < 0)
        dist *= -1;
    *rightOut = encPrev_Right - (20.81 * dist);

    //printf("\nRd = %f | %f | %f",dist,encPrev_Right,*rightOut);

    

    return false;
}

//---Trajectories---
bool PathFinder::tra_FormTrajectory(double startPower, int wayStart, double endPower, int wayEnd){
    //lots of math
    //printf("\nTotal Dist = %f",total_dist);
    int mtime = 0;
    int f2_length = 1;
    int f1_length;
    double impulse;
    double adjust_max_power;
    //if (wayStart == 0) {
    double start_discount = 0.5 * startPower * startPower / m_Config.MaxAccel;
    double end_discount = 0.5 * endPower * endPower / m_Config.MaxAccel;
    adjust_max_power = std::min(m_Config.MaxSpeed, sqrt(m_Config.MaxAccel * total_dist - start_discount - end_discount));

    double t_rampup = (adjust_max_power - startPower) / m_Config.MaxAccel;
    double x_rampup = startPower * t_rampup + 0.5 * m_Config.MaxAccel * (t_rampup * t_rampup);

    double t_rampdown = (adjust_max_power - endPower)/m_Config.MaxAccel;
    double x_rampdown = adjust_max_power * t_rampdown - 0.5 * m_Config.MaxAccel * (t_rampdown * t_rampdown);
    double x_curise = total_dist - x_rampdown - x_rampup;

    mtime = (int)((t_rampup + t_rampdown + x_curise / adjust_max_power) / m_Config.cycleTime + 0.5);
    f1_length = (int) std::ceil((adjust_max_power / m_Config.MaxAccel) / m_Config.cycleTime );
    impulse = (total_dist / adjust_max_power) / m_Config.cycleTime - startPower / m_Config.MaxAccel / m_Config.cycleTime + start_discount + end_discount;
    //}
    /*else {
        adjust_max_power = std::min(m_Config.MaxSpeed,
            (-m_Config.MaxAccel * m_Config.MaxAccel + std::sqrt(m_Config.MaxAccel
                    * m_Config.MaxAccel * m_Config.MaxAccel * m_Config.MaxAccel
                    + 4 * m_Config.MaxJerk * m_Config.MaxJerk * m_Config.MaxAccel
                    * total_dist)) / (2 * m_Config.MaxJerk));

        // Compute the length of the linear filters and impulse.
        f1_length = (int) std::ceil((adjust_max_power / m_Config.MaxAccel) / m_Config.cycleTime);
        f2_length = (int) std::ceil((m_Config.MaxAccel / m_Config.MaxJerk) / m_Config.cycleTime);
        impulse = (total_dist / adjust_max_power) / m_Config.cycleTime;
        mtime = (int) (std::ceil(f1_length + f2_length + impulse));
    }*/

    //Create trajectory
    if(mtime > 500)
        return false;
    //The positions of each trajectory need to be initialized, this is what the following code does
    m_Traj.segments[0].pos = 0;
    m_Traj.segments[0].vel = startPower;
    m_Traj.segments[0].acc = 0;
    m_Traj.segments[0].jerk = 0;
    m_Traj.segments[0].dt = m_Config.cycleTime;

    int last = 0;

    double f1[500];
    double f2;
    f1[0] = (startPower / adjust_max_power) * f1_length;
    for(int i=0;i<mtime;i++) {
        double input = std::min(impulse, 1.0);
        if (input < 1) {
            // The impulse is over, so decelerate
            input -= 1;
            impulse = 0;
        } else {
            impulse -= input;
        }

        // Filter through F1
        double f1_last;
        if (i > 0) {
            f1_last = f1[i - 1];
        } else {
            f1_last = f1[0];
        }
        f1[i] = std::max(0.0, std::min((double)f1_length, f1_last + input));

        f2 = 0;
        // Filter through F2
        for (int j = 0; j < f2_length; ++j) {
            if (i - j < 0) {
                break;
            }

            f2 += f1[i - j];
        }

        f2 = f2 / f1_length;

        m_Traj.segments[i].vel = f2 / f2_length * adjust_max_power;
        m_Traj.segments[i].pos = (m_Traj.segments[last].vel + m_Traj.segments[i].vel) / 2.0 * m_Config.cycleTime + m_Traj.segments[last].pos;

        m_Traj.segments[i].x = m_Traj.segments[i].pos;
        m_Traj.segments[i].y = 0;

        m_Traj.segments[i].acc = (m_Traj.segments[i].vel - m_Traj.segments[last].vel) / m_Config.cycleTime;
        m_Traj.segments[i].jerk = (m_Traj.segments[i].acc - m_Traj.segments[last].acc) / m_Config.cycleTime;
        m_Traj.segments[i].dt = m_Config.cycleTime;

        last = i;
    }


    //The following code goes through the trajectory and plots segments with and x,y, and heading to the splines
    int cur_spline = 0;
    double cur_spline_start_pos = 0;
    double spline_complete = 0;
    for(int i = 0; i<mtime; ++i) {
        double cur_pos = m_Traj.segments[i].pos;
        bool spline_found = false;
        for(;(!spline_found);) {
            double cur_pos_relative = cur_pos - cur_spline_start_pos;
            if (cur_pos_relative <= m_Splines[cur_spline].arcLength) {
                //Use the current spline to plot this segment
                double percentage = spline_getPercentageForDistance(cur_spline,cur_pos_relative,10000);
                m_Traj.segments[i].heading = angleAt(cur_spline, percentage);
                spline_getXY(cur_spline, percentage, &m_Traj.segments[i].x, &m_Traj.segments[i].y);
                spline_found = true;
            }
            else if (cur_spline < m_Spline_Cnt - 1) { 
                //We need to move to the next spline
                spline_complete += m_Splines[cur_spline].arcLength;
                cur_spline_start_pos = spline_complete;
                ++cur_spline;
                //if(cur_spline == m_Spline_Cnt) //Something has gone wrong!
                //    return false;
            }
            else {
                m_Traj.segments[i].heading = angleAt(m_Spline_Cnt-1, 1.0);
                spline_getXY(m_Spline_Cnt-1, 1.0, &m_Traj.segments[i].x, &m_Traj.segments[i].y);
                spline_found = true;
            }
        }
    }

    //Calculate the left and right trajectorys
    //  Fills in the following trajectories
    //      m_L_Traj; - The trajectory for the left
    //      m_R_Traj; - The trajectory for the right
    for(int i = 0; i<mtime; ++i) {
        //Copy the segment into the left and right
        copySegment(i);
        double cos_angle = cos(m_Traj.segments[i].heading);
        double sin_angle = sin(m_Traj.segments[i].heading);

        m_L_Traj.segments[i].x = m_Traj.segments[i].x - m_Config.WheelBase / 2 * sin_angle;
        m_L_Traj.segments[i].y = m_Traj.segments[i].y + m_Config.WheelBase / 2 * cos_angle;

        m_R_Traj.segments[i].x = m_Traj.segments[i].x + m_Config.WheelBase / 2 * sin_angle;
        m_R_Traj.segments[i].y = m_Traj.segments[i].y - m_Config.WheelBase / 2 * cos_angle;
        if (i > 0) {
            double dist = sqrt(((m_L_Traj.segments[i].x - m_L_Traj.segments[i-1].x)
                *(m_L_Traj.segments[i].x - m_L_Traj.segments[i-1].x))
                +((m_L_Traj.segments[i].y - m_L_Traj.segments[i-1].y)
                *(m_L_Traj.segments[i].y - m_L_Traj.segments[i-1].y)));
            m_L_Traj.segments[i].pos = m_L_Traj.segments[i-1].pos + dist;
            m_L_Traj.segments[i].vel = dist / m_L_Traj.segments[i].dt;
            m_L_Traj.segments[i].acc = (m_L_Traj.segments[i].vel - m_L_Traj.segments[i-1].vel) / m_L_Traj.segments[i].dt;
            m_L_Traj.segments[i].jerk = (m_L_Traj.segments[i].acc - m_L_Traj.segments[i-1].acc) / m_L_Traj.segments[i].dt;

            dist = sqrt(((m_R_Traj.segments[i].x - m_R_Traj.segments[i-1].x)
                *(m_R_Traj.segments[i].x - m_R_Traj.segments[i-1].x))
                +((m_R_Traj.segments[i].y - m_R_Traj.segments[i-1].y)
                *(m_R_Traj.segments[i].y - m_R_Traj.segments[i-1].y)));
            m_R_Traj.segments[i].pos = m_R_Traj.segments[i-1].pos + dist;
            m_R_Traj.segments[i].vel = dist / m_R_Traj.segments[i].dt;
            m_R_Traj.segments[i].acc = (m_R_Traj.segments[i].vel - m_R_Traj.segments[i-1].vel) / m_R_Traj.segments[i].dt;
            m_R_Traj.segments[i].jerk = (m_R_Traj.segments[i].acc - m_R_Traj.segments[i-1].acc) / m_R_Traj.segments[i].dt;
        }
    }

    m_segmentCount = mtime;
    //printf("\nSeg Count %i",m_segmentCount);

    return true;
}

void PathFinder::copySegment(int idx) {
    m_L_Traj.segments[idx].pos = m_Traj.segments[idx].pos;
    m_L_Traj.segments[idx].acc = m_Traj.segments[idx].acc;
    m_L_Traj.segments[idx].vel = m_Traj.segments[idx].vel;
    m_L_Traj.segments[idx].jerk = m_Traj.segments[idx].jerk;
    m_L_Traj.segments[idx].heading = m_Traj.segments[idx].heading;
    m_L_Traj.segments[idx].dt = m_Traj.segments[idx].dt;
    m_L_Traj.segments[idx].x = m_Traj.segments[idx].x;
    m_L_Traj.segments[idx].y = m_Traj.segments[idx].y;

    m_R_Traj.segments[idx].pos = m_Traj.segments[idx].pos;
    m_R_Traj.segments[idx].acc = m_Traj.segments[idx].acc;
    m_R_Traj.segments[idx].vel = m_Traj.segments[idx].vel;
    m_R_Traj.segments[idx].jerk = m_Traj.segments[idx].jerk;
    m_R_Traj.segments[idx].heading = m_Traj.segments[idx].heading;
    m_R_Traj.segments[idx].dt = m_Traj.segments[idx].dt;
    m_R_Traj.segments[idx].x = m_Traj.segments[idx].x;
    m_R_Traj.segments[idx].y = m_Traj.segments[idx].y;
}

//---Splines---
bool PathFinder::generateSpline(int idx, int way1, int way2) {
    //printf("\nSpline: ");
    m_Splines[idx].arcLength = 0;

    //Calculate offsets
    m_Splines[idx].x_offset = m_WayPoints[way1].x;
    m_Splines[idx].y_offset = m_WayPoints[way1].y;

    //Calculate length
    m_Splines[idx].distance = sqrt((m_WayPoints[way2].x - m_WayPoints[way1].x)*(m_WayPoints[way2].x - m_WayPoints[way1].x)+(m_WayPoints[way2].y - m_WayPoints[way1].y)*(m_WayPoints[way2].y - m_WayPoints[way1].y));
    //printf("\n    Dist=%f",m_Splines[idx].distance);
    if(m_Splines[idx].distance == 0)
        return false;

    m_Splines[idx].theta_offset = atan2(m_WayPoints[way2].y - m_WayPoints[way1].y, m_WayPoints[way2].x - m_WayPoints[way1].x);
    m_Splines[idx].theta_S_hat =  angleDiffRadians(m_Splines[idx].theta_offset, m_WayPoints[way1].theta);
    m_Splines[idx].theta_E_hat =  angleDiffRadians(m_Splines[idx].theta_offset, m_WayPoints[way2].theta);
    //printf("\n    theta Offset=%f | theta S offset=%f theta E offset=%f| ",m_Splines[idx].theta_offset,m_Splines[idx].theta_S_hat,m_Splines[idx].theta_E_hat);
    if((fabs(m_Splines[idx].theta_S_hat - (M_PI / 2)) < 0.0001)||(fabs(m_Splines[idx].theta_E_hat - (M_PI / 2)) < 0.0001))
        return false;
    double dumbVariable = angleDiffRadians(m_Splines[idx].theta_S_hat, m_Splines[idx].theta_E_hat);
    double dumbVariable2 = (M_PI / 2);
    if(dumbVariable >= dumbVariable2)
        return false;

    //Slopes
    m_Splines[idx].m_S_Hat = tan(m_Splines[idx].theta_S_hat);
    m_Splines[idx].m_E_Hat = tan(m_Splines[idx].theta_E_hat);

    //Cofficients
    m_Splines[idx].a = -(3*(m_Splines[idx].m_S_Hat + m_Splines[idx].m_E_Hat))/(m_Splines[idx].distance * m_Splines[idx].distance * m_Splines[idx].distance * m_Splines[idx].distance);
    m_Splines[idx].b = (8*m_Splines[idx].m_S_Hat + 7 * m_Splines[idx].m_E_Hat)/(m_Splines[idx].distance * m_Splines[idx].distance * m_Splines[idx].distance);
    m_Splines[idx].c = -(6*m_Splines[idx].m_S_Hat + 4*m_Splines[idx].m_E_Hat) / (m_Splines[idx].distance * m_Splines[idx].distance);
    m_Splines[idx].d = 0;
    m_Splines[idx].e = m_Splines[idx].m_S_Hat;

    //printf("\n    %f | %f | %f | %f | %f",m_Splines[idx].a,m_Splines[idx].b,m_Splines[idx].c,m_Splines[idx].d,m_Splines[idx].e);

    return true;
}

double PathFinder::spline_CalculateLength(int idx, int samples) {
    if(m_Splines[idx].arcLength != 0)
        return m_Splines[idx].arcLength;
    if(samples == 0)
        return 0;
    double arcLength = 0;
    double t, dydt, integrand, last_integrand = sqrt(1 + spline_derivativeAt(idx, 0) * spline_derivativeAt(idx, 0)) / samples;
    for(int i = 1; i<=samples; ++i) {
        t = ((double) i) / samples;
        dydt = spline_derivativeAt(idx, t);
        integrand = sqrt(1 + dydt * dydt) / samples;
        arcLength += (integrand + last_integrand) / 2;
        last_integrand = integrand;
    }
    m_Splines[idx].arcLength = arcLength * m_Splines[idx].distance;
    
    //printf("\n   arc=%f",m_Splines[idx].arcLength);
    
    return m_Splines[idx].arcLength;
}

double PathFinder::spline_getPercentageForDistance(int idx, double distance, int samples) {
    double arcLength = 0;
    double last_arc_length = 0;
    double t = 0, dydt, integrand, last_integrand = sqrt(1 + spline_derivativeAt(idx, 0) * spline_derivativeAt(idx, 0)) / samples;
    distance /= m_Splines[idx].distance;
    for(int i = 1; i<=samples; ++i) {
        t = ((double) i) / samples;
        dydt = spline_derivativeAt(idx, t);
        integrand = sqrt(1 + dydt * dydt) / samples;
        arcLength += (integrand + last_integrand) / 2;
        if(arcLength > distance)
            break;
        last_integrand = integrand;
        last_arc_length = arcLength;
    }

    //Interpolate between samples
    double interpolate = t;
    if(arcLength != last_arc_length){
        interpolate += ((distance - last_arc_length) / (arcLength - last_arc_length) - 1) / ((double) samples);
    }
    return interpolate;
}

bool PathFinder::spline_getXY(int idx, double percentage, double *outX, double *outY) {
    percentage = std::max(std::min(percentage, 1.0), 0.0);
    double x_hat = m_Splines[idx].distance * percentage;

    double dist2 = x_hat * x_hat;
    double dist3 = x_hat * x_hat * x_hat;
    double dist4 = x_hat * x_hat * x_hat * x_hat;

    double y_hat = (m_Splines[idx].a * x_hat + m_Splines[idx].b) * dist4 
        + m_Splines[idx].c * dist3 + m_Splines[idx].d * dist2 + m_Splines[idx].e * x_hat;
    
    double cos_theta = cos(m_Splines[idx].theta_offset);
    double sin_theta = sin(m_Splines[idx].theta_offset);

    double x = x_hat * cos_theta - y_hat * sin_theta + m_Splines[idx].x_offset;
    double y = x_hat * sin_theta + y_hat * cos_theta + m_Splines[idx].y_offset;
    //printf("\nPoints=(%f,%f)",x,y);

    *outX = x;
    *outY = y;
    return true;
}

double PathFinder::spline_ValueAt(int idx, double percentage){
    percentage = std::max(std::min(percentage, 1.0), 0.0);
    double x_hat = m_Splines[idx].distance * percentage;

    double dist2 = x_hat * x_hat;
    double dist3 = x_hat * x_hat * x_hat;
    double dist4 = x_hat * x_hat * x_hat * x_hat;

    double y_hat = (m_Splines[idx].a * x_hat + m_Splines[idx].b) * dist4 
        + m_Splines[idx].c * dist3 + m_Splines[idx].d * dist2 + m_Splines[idx].e * x_hat;
    
    double cos_theta = cos(m_Splines[idx].theta_offset);
    double sin_theta = sin(m_Splines[idx].theta_offset);

    double value = x_hat * sin_theta + y_hat * cos_theta + m_Splines[idx].y_offset;
    return value;
}

double PathFinder::spline_derivativeAt(int idx, double percentage) {
    percentage = std::max(std::min(percentage, 1.0), 0.0);
    double x_hat = m_Splines[idx].distance * percentage;
    double dist2 = x_hat * x_hat;
    double dist3 = x_hat * x_hat * x_hat;
    double yp_slope = (5 * m_Splines[idx].a * x_hat + 4 * m_Splines[idx].b)
        * dist3 + 3 * m_Splines[idx].c * dist2 + 2 * m_Splines[idx].d * x_hat + m_Splines[idx].e;
    //(5 * a * dist + 4 * b) + dist * dist * dist + 3 * c * dist * dist + 2 * d * dist + e
    return yp_slope;
}

double PathFinder::spline_SecondDerivativeAt(int idx, double percentage) {
    percentage = std::max(std::min(percentage, 1.0), 0.0);
    double x_hat = m_Splines[idx].distance * percentage;
    double dist2 = x_hat * x_hat;
    double dist3 = x_hat * x_hat * x_hat;
    double yp_slope = (20 * m_Splines[idx].a * x_hat + 12 * m_Splines[idx].b)
        * dist2 + 6 * m_Splines[idx].c * x_hat + 2 * m_Splines[idx].d;
    
    return yp_slope;
}

double PathFinder::angleAt(int idx, double percentage){
    double angle = atan(spline_derivativeAt(idx, percentage)) + m_Splines[idx].theta_offset;

    while (angle >= 2.0 * M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    return angle;
}

double PathFinder::angleDiffRadians(double from, double to){
    double angle = to - from;
    while (angle >= M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
}