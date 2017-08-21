/*************************************************************************
	> File Name: norm_dist.cpp
	> Jun Jin.: 
	> Mail:jjin5@ualberta.ca
    > ---- norm distribution functions.----
	> Created Time: Mon 20 Feb 2017 09:23:14 PM MST
 ************************************************************************/

#include<iostream>
#include<math.h>
#include<stdlib.h>
#include "../include/norm_dist.h"
using namespace std;

    norm_dist::norm_dist()
    {
    }

    norm_dist::~norm_dist(){}
    //get x=rand(-b,b)
    double norm_dist::getRandX(double b)
    {
        st_d=b;
        double sum=0;
       //run rand 12 times
       for(int i=0;i<12;i++)
      {
        double f1=(double)rand() / RAND_MAX;
        sum +=f1*2*st_d-st_d;
      }
       
       return sum/2;
    }
