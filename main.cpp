#include "main.h"

using namespace std;

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

    int initial_charger_index, goal_charger_index;

    //------------- find index of chargers in network array-------------
    initial_charger_index = find_row_index(initial_charger_name);
    goal_charger_index = find_row_index(goal_charger_name);

    //--------------create a matrix to store distance between different chargers-----------
    double **distance_matrix; // row index charger -> column index charger
    distance_matrix = new double*[N];
    for(int i=0; i < N; i++) distance_matrix[i] = new double[N];
    create_distance_matrix(distance_matrix,N);
    //print_matrix(distance_matrix,30);

    //--------------create a matrix to store travel and charge time between different chargers---------------
    double **time_matrix;  // row index charger -> column index charger
    time_matrix = new double*[N];
    for(int i=0; i < N; i++) time_matrix[i] = new double[N];
    create_time_matrix(time_matrix,distance_matrix,N);

    //--------------find the shortest path--------------------
    int path[N];
    find_path(distance_matrix,time_matrix,initial_charger_index,goal_charger_index,path,N);

    //--------------print the shortest path to output----------------------
    print_solution(path,initial_charger_index,goal_charger_index,distance_matrix);

    return 0;
}
