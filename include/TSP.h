/**
 * @file TSP.h
 * @author your name (you@domain.com)
 * @brief Travelling salesman Problem algorithm
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Vector.h>
#include <Path.h>
#include <utilities.h>
#include <GPS.h>

const int NUM_OF_DESTINATION = 4;

Path waypoints[NUM_OF_DESTINATION][NUM_OF_DESTINATION];
const Path *waypoints_ptr[NUM_OF_DESTINATION][NUM_OF_DESTINATION];
void calculate_waypoints(const double locations[NUM_OF_DESTINATION][2]) {
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            const double distance = get_gps_distance(
                locations[i][0],
                locations[i][1],
                locations[j][0],
                locations[j][1]
            );
            waypoints[i][j] = Path(i, j, distance);
            Serial.println("Distance " + String(i) + " - " + String(j) + " : " + String(waypoints[i][j].getDistance()));
        }
    } 
}

Path _path_storage[NUM_OF_DESTINATION];
Vector<Path> path(_path_storage);
int path_cost = 0;
void calculate_path(int starting_point) {
    // store all vertex apart from source vertex
    int _vertex_storage[NUM_OF_DESTINATION];
    Vector<int> vertex(_vertex_storage);
    for (int i = 0; i < NUM_OF_DESTINATION; i++) {
        if (i != starting_point) {
            vertex.push_back(i);
        }
    }

    // Serial.println(vertex.size());
    // store minimum weight Hamiltonian Cycle.
    int min_path = INT_MAX;
    int last_min_path = min_path;
    do {

        // store current Path weight(cost)
        int current_pathweight = 0;
        Path _current_path_storage[NUM_OF_DESTINATION];
        Vector<Path> current_path(_current_path_storage); 


        // compute current path weight
        int k = starting_point;
        for (size_t i = 0; i < vertex.size(); i++) {
            current_pathweight += waypoints[k][vertex[i]].getDistance();
            current_path.push_back(waypoints[k][vertex[i]]);
            k = vertex[i];
        }
        current_pathweight += waypoints[k][starting_point].getDistance();
        current_path.push_back(waypoints[k][starting_point]);

        // update minimum
        min_path = min(min_path, current_pathweight);

        if(last_min_path != min_path) {
            for(size_t i = 0; i < 4; i++) {
                path.push_back(current_path[i]);
            }
        }

        last_min_path = min_path;
        
    } while (next_permutation<VectorIterator<int>>(vertex.begin(), vertex.end()));

    path_cost = min_path;
}