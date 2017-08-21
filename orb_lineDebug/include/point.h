/*************************************************************************
	> File Name: point.h
	> Author:Jun Jin 
	> Mail: jjin5@ualberta.ca
	> Created Time: Tue 21 Feb 2017 05:00:12 PM MST
 ************************************************************************/

#ifndef _POINT_H
#define _POINT_H
#endif
//point class, (x,y,theta)
class point
{
    public:
    point();
    point(double _x, double _y, double _theta);
    ~point();
    double x;
    double y;
    double theta;
};

