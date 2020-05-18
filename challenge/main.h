#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#include "math.h"
#include "network.h"
#include <algorithm>    // std::min, std::reverse
#include <iostream>
#include <vector>
#include <queue>
#include "float.h"

using namespace std;

double const MAX_CHARGE = 320; //km
double const CAR_SPEED = 105; //km per hr 

double distance(double lat1,double lon1, double lat2, double lon2);
int find_row_index(string name);
void create_distance_matrix(double **matrix, int size);
void create_time_matrix(double **matrix,double **distance_m, int size);
void find_path(double **d_matrix, double ** d_c_matrix, int initial, int goal,int *path,int size);
void print_solution(int path[], int initial, int goal, double **d_matrix);
void print_matrix(double **d, int size);


double distance(double lat1,double lon1, double lat2, double lon2)
{
    //calculates the great-circle distance in km between two points given the longitudes and latitudes in degrees using Havershine formula
    //args: lat1 - latitude of first point
    //      lon1 - longitude of first point
    //      lat2 - latitute of second point
    //      lon2 - longitude of second point
    //returns: distance between two points in km

    const double RADIUS  = 6356.752; //km
    const double PI = 3.14159265;

    //convert from degrees to radians
    lat1 *= PI/180;
    lat2 *= PI/180;
    lon1 *= PI/180;
    lon2 *= PI/180;

    //Havershine formula
    double dlon = lon2-lon1;
    double dlat = lat2-lat1;
    double sin_angle = pow(sin(dlat/2),2) + (cos(lat1)*cos(lat2)*pow(sin(dlon/2),2));
    double angle = 2*asin(std::min((double)1,sqrt(sin_angle)));
    
    //calculate distance
    double d = RADIUS*angle;

    return d;
}

int find_row_index(string name)
{
    //given a name of a charger finds the index of the charger in the network array else returns -1
    //args: name - name of the super chargers
    //returns: index of first charger in network

    for(int i=0;i<network.size();i++)
    {
        if(network[i].name == name) return i;
    }
    return -1;
}


void create_distance_matrix(double **matrix, int size)
{
    // creates the distance matrix which stores the distance between two chargers in km
    // D[i][j] = distance(network[i] -> networ[j])
    // args: matrix - pointer to 2D matrix to store distances
    //       size - size of 2D matrix
    // returns: None

    for(int i=0;i<size;i++)
    {
        for(int j=i;j<size;j++)
        {
            double d = 0;
            if(i!=j)
            {
                double lon1,lon2,lat1,lat2;
                lon1 = network[i].lon;
                lat1 = network[i].lat;
                lon2 = network[j].lon;
                lat2 = network[j].lat;

                //calculate distance between the two chargers
                d = distance(lat1,lon1,lat2,lon2);

                //if distance is greater than maximum charge of car then charger is unreachable
                if(d>MAX_CHARGE) d = -1;
            } 
            matrix[i][j] = d;
            matrix[j][i] = d;
        }
    }
}

void create_time_matrix(double **matrix,double **distance_m, int size)
{
    // creates the distance matrix which stores the distance between two chargers in km
    // D[i][j] = distance(network[i] -> networ[j])
    //args: matrix - pointer to 2D matrix to store times in 
    //      distance - pointer to distance matrix
    //      size - size of 2D matrix
    //returns: None


    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            double dc = 0;
            if(i!=j)
            {
                double charge = network[i].rate;
                double d = distance_m[i][j];

                //calculate time needed to charge and travel to next point assuming zero initial charge of car
                dc = d*(1/CAR_SPEED + 1/charge);

                //if distance is negative then charger is unreachable
                if(dc<0) dc = -1;
            } 
            matrix[i][j] = dc;
        }
    }
}

typedef pair<double, int> pi; 

void find_path(double **d_matrix, double ** t_matrix, int initial, int goal,int *path,int size)
{
    //finds the shortest path between network[initial] and network[goal] chargers using modified version of Dijkstra's
    //Assumes there exists a path between initial and goal charger
    //args: d_matrix - pointer to distance matrix
    //      t_matrix - pointer to time matrix
    //      initial - index of start node
    //      goal - index of end node
    //      path - pointer to path array
    //      size - size of distance and time matrix
    //returns: Null

    //array to store if node is visited
    bool visited[size];

    //tracker of minimum time from start node
    double time_track[size];

    //priority queue to store potential connections, returns element with least time
    priority_queue<pi, vector<pi>, greater<pi> > pq;
    
    //initialize all arrays
    for(int i=0;i<size;i++)
    {
        path[i] = -1;
        visited[i] = false;
        time_track[i] = DBL_MAX;
    }

    //add start node to arrays
    pq.push({0,initial});
    time_track[initial] = 0;
    path[initial] = initial;

    //if start node is equal to final node return
    if(initial==goal) return;

    //loop through all nodes in priority queue
    while(!pq.empty())
    {
        //get least time node from queue
        pair<double,int> p = pq.top();
        pq.pop();

        int i = p.second;

        //if visited then skip        
        if(visited[i]) continue;
        visited[i] = true;

        for(int j=0;j<size;j++)
        {
            double time_i_j = t_matrix[i][j];
            if(time_i_j==-1 || time_i_j==0)continue;

            //if new time is less than minimum time then update minimum time
            if(time_i_j + time_track[i] < time_track[j])
            {
                time_track[j] = time_i_j + time_track[i];

                //add node to priority queue
                pq.push({time_track[j],j});

                //set the parent node of current node to i
                path[j] = i;
            }
        }

    }
}

void print_solution(int path[], int initial, int goal, double **d_matrix)
{
    //print the solution given the path from initial charger to goal charger
    //args: path - the path produced by find_path
    //      initial - index of start charger
    //      goal - index of goal charger
    //      d_matrix - pointer to distance matrix
    //returns: Null

    //set current node to goal
    int current = goal;
    
    //vector to store path
    vector<int> short_path;
    short_path.push_back(current);
    
    //add nodes from path to short_path that lead to goal
    while(current!=initial)
    {
        current = path[current];
        short_path.push_back(current);
    }
    
    //reverse short_path to have start node at beginning
    reverse(short_path.begin(),short_path.end());

    //set starting capacity to full charge
    double capacity = MAX_CHARGE;

    for(int i=0;i<short_path.size();i++)
    {
        //update current charger
        int current = short_path[i];
        
        //if end charger print just the charger name
        if(current == goal) cout << network[goal].name ;
        else
        {
            //get next charger in path
            int next = short_path[i+1];
            
            double rate = network[current].rate;
            
            //calculate charge required
            double charge = max((double)0,d_matrix[current][next]-capacity);

            //if start charger then just print charger name
            if(current==initial) cout << network[current].name << ", ";
            else cout << network[current].name << ", " << charge/rate << ", ";
            
            //update charge capacity of the car
            capacity = capacity - d_matrix[current][next] + charge;
        }
    }    
}

void print_matrix(double **d, int size)
{
    //print a 2D matrix with spaces between elements
    //args: d - pointer to 2D matrix
    //      size - size of 2D matrix
    //returns: Null

    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            cout << d[i][j] << " ";
        }
        cout << endl;
    }
}



#endif
