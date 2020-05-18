#include "main.h"

using namespace std;

int main()
{
    cout << distance(51.15,1.33,50.97,1.85) << endl;

    int initial = find_row_index("Salina_KS");
    int goal = find_row_index("Perry_OK");

    cout << initial << " " << goal << endl;
    double lat1,lat2,lon1,lon2;

    lat1 = network[initial].lat;
    lon1 = network[initial].lon;
    lat2 = network[goal].lat;
    lon2 = network[goal].lon;

    cout<< distance(lat1,lon1,lat2,lon2) << endl;

}