#ifndef _TSP_H_
#define _TSP_H_

#include "WayPoint.h"
#include <Vector.h>
#include <utilities.h>
#include <Arduino.h>
#include <TinyGPS++.h>


#define DESTINATION_COUNT 4

class TSP {
    protected:
        WayPoint *waypoints[DESTINATION_COUNT][DESTINATION_COUNT];
        Vector<WayPoint *> path;
        double path_cost = 0;
        int starting_point;
    
    public: 
        TSP() = default;

        void calculate_waypoints(const double (*locations)[DESTINATION_COUNT][2]) {
            for(int i = 0; i < 4; i++) {
                for(int j = 0; j < 4; j++) {
                    // const double distance = TinyGPSPlus::distanceBetween(
                    //     (*locations)[i][0],(*locations)[i][1],
                    //    (*locations)[j][1],(*locations)[j][1]);
                    const double distance = random(0, 500);
                    // Serial.println(distance);
                    this->waypoints[i][j] = new WayPoint(i, j, distance);
                    // Serial.println(this->waypoints[i][j]->getName());
                }
            }
        }

        Vector<WayPoint *> calculate_path(int starting_point) {
            // store all vertex apart from source vertex
            Vector<int> vertex;
            int position = 0;
            for (int i = 0; i < DESTINATION_COUNT; i++) {
                if (i != starting_point) {
                    vertex[position++] = i;
                }
            }

            // Serial.println(vertex[2]);

            // store minimum weight Hamiltonian Cycle.
            int min_path = INT_MAX;
            int last_min_path = min_path;
            do {

                // store current Path weight(cost)
                int current_pathweight = 0;
                Vector<WayPoint *> current_path; 


                // compute current path weight
                int k = starting_point;
                for (int i = 0; i < position; i++) {
                    current_pathweight += this->waypoints[k][vertex[i]]->getDistance();
                    current_path.push_back(this->waypoints[k][vertex[i]]);
                    k = vertex[i];
                }
                current_pathweight += this->waypoints[k][starting_point]->getDistance();
                current_path.push_back(this->waypoints[k][starting_point]);

                // update minimum
                min_path = min(min_path, current_pathweight);

                if(last_min_path != min_path) {
                    for(size_t i = 0; i < 4; i++) {
                        this->path.push_back(current_path[i]);
                    }
                }

                last_min_path = min_path;

                // Serial.println(this->waypoints[1][3]->start());
                
            } while (next_permutation<VectorIterator<int>>(vertex.begin(), vertex.end()));

            this->path_cost = min_path;
            return this->path;
        }
       
       double get_path_cost() { return this->path_cost; }
};

#endif