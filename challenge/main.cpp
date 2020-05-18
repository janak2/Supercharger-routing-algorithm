#include "network.h"
#include <queue>
#include "float.h"
#include <vector>
#include <algorithm>    // std::reverse
#include "main.h"

using namespace std;

//extern std::array<row, 303> network;

double MAX_CHARGE = 320; //km
double CAR_SPEED = 105; //km per hr


void create_distance_matrix(double **matrix, int size)
{
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
                d = distance(lat1,lon1,lat2,lon2);
                if(d>MAX_CHARGE) d = -1;
            } 
            matrix[i][j] = d;
            matrix[j][i] = d;
        }
    }
}
 
void create_distance_charge_matrix(double **matrix,double **distance_m, int size)
{
    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            double dc = 0;
            if(i!=j)
            {
                double charge = network[i].rate;
                double d = distance_m[i][j];
                dc = d*(1/CAR_SPEED + 1/charge);
                if(dc<0) dc = -1;
            } 
            matrix[i][j] = dc;
        }
    }
}

int find_row_index(string name)
{
    for(int i=0;i<network.size();i++)
    {
        if(network[i].name == name) return i;
    }
    return -1;
}


void find_path(double **d_matrix, double ** d_c_matrix, int initial, int goal,int *path,int size)
{
    //int path[size];
    bool visited[size];
    double min_dist[size];
    priority_queue<pair<double,int>> pq;

    
    for(int i=0;i<size;i++)
    {
        path[i] = -1;
        visited[i] = false;
        min_dist[i] = DBL_MAX;
    }

    pq.push({0,initial});
    min_dist[initial] = 0;
    path[initial] = -1;

    while(!pq.empty())
    {
        pair<double,int> p = pq.top();
        pq.pop();

        //cout << p.first << " "<< p.second << endl;

        int i = p.second;        
        if(visited[i]) continue;

        visited[i] = true;

        for(int j=0;j<size;j++)
        {
            double d = d_matrix[i][j];
            if(d!=-1 && min_dist[i]+d < min_dist[j])
            {
                min_dist[j] = min_dist[i] + d;
                pq.push({min_dist[j],j});
                path[j] = i;
            }
        }

    }
}

void print_solution(int path[], int initial, int goal, double **d_matrix)
{
    int current = goal;
    vector<int> short_path;
    short_path.push_back(current);
    
    while(current!=initial)
    {
        current = path[current];
        short_path.push_back(current);
    }

    double capacity = MAX_CHARGE;
    reverse(short_path.begin(),short_path.end());

    for(int i=0;i<short_path.size();i++)
    {
        int current = short_path[i];
        if(current ==initial) cout << "\"" << network[initial].name << ", ";
        else if(current == goal) cout  << network[goal].name  << "\"" ;
        else
        {
            int next = short_path[i+1];
            double rate = network[current].rate;
            double charge = max((double)0,d_matrix[current][next]-capacity+1);
            cout << network[current].name << ", " << charge/rate << ", ";
            capacity = capacity - d_matrix[current][next] + charge;
        }
    }    
}

void print_matrix(double **d, int size)
{
    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            cout << d[i][j] << " ";
        }
        cout << endl;
    }
}


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }
    
    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];

    const int N = network.size();
    //cout << N << endl;

    int initial_charger_index, goal_charger_index;

    initial_charger_index = find_row_index(initial_charger_name);
    goal_charger_index = find_row_index(goal_charger_name);

    //cout << initial_charger_index << " " << goal_charger_index << endl;

    double **distance_matrix; // row -> column
    distance_matrix = new double*[N];
    for(int i=0; i < N; i++) distance_matrix[i] = new double[N];

    create_distance_matrix(distance_matrix,N);
    //print_matrix(distance_matrix,30);

    double **distance_charge_matrix;  // row -> column
    distance_charge_matrix = new double*[N];
    for(int i=0; i < N; i++) distance_charge_matrix[i] = new double[N];

    create_distance_charge_matrix(distance_charge_matrix,distance_matrix,N);

    int path[N];
    find_path(distance_matrix,distance_charge_matrix,initial_charger_index,goal_charger_index,path,N);

    //for(int i=0;i<N;i++) cout << path[i] << " ";

    //cout << path[goal_charger_index]<<endl;

    print_solution(path,initial_charger_index,goal_charger_index,distance_matrix);

    return 0;
}
