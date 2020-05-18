#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED
#include "math.h"
#include "network.h"
#include <algorithm>    // std::min
#include <iostream>

using namespace std;

double distance(double lat1,double lon1, double lat2, double lon2);

int find_row_index(string name);

double distance(double lat1,double lon1, double lat2, double lon2)
{
    //calculates the great-circle distance in km between two points given the longitudes and latitudes using Havershine formula

    const double RADIUS  = 6356.752; //km
    const double PI = 3.14159265;

    lat1 *= PI/180;
    lat2 *= PI/180;
    lon1 *= PI/180;
    lon2 *= PI/180;

    double dlon = lon2-lon1;
    double dlat = lat2-lat1;

    double a = pow(sin(dlat/2),2) + (cos(lat1)*cos(lat2)*pow(sin(dlon/2),2));
    double c = 2*asin(std::min((double)1,sqrt(a)));
    
    double d = RADIUS*c;

    return d;
}

int find_row_index(string name)
{
    for(int i=0;i<network.size();i++)
    {
        if(network[i].name == name) return i;
    }
    return -1;
}



#endif
