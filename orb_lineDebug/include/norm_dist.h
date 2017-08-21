/*************************************************************************
	> File Name: norm_dist.h
	> Author: 
	> Mail: 
	> Created Time: Mon 20 Feb 2017 09:23:05 PM MST
 ************************************************************************/

#ifndef _NORM_DIST_H
#define _NORM_DIST_H
#endif
#include <iostream>
#include<math.h>
#include<stdlib.h>
using namespace std;

class norm_dist
{
    public:
       norm_dist();
       ~norm_dist();
       double getRandX(double b);
    private:
       double st_d;
};
