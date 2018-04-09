/*
 * Author: Vincent Yuan
 * Create Date: Mar 26, 2018 
 * Last Edit: Mar 27, 2018 
 * File: Otto.h
 * Purpose: Holds main and functions declrataions for processing Otto challenge 
 *
 */ 

#include <iostream>
#include <fstream> // For access to file stream 
#include <cstring>
#include <cstdlib>
#include <vector>
#include <stdint.h>
#include <math.h>
#include <iomanip>

#define DEBUG false

//Waypoints are each line parsed from text file
struct waypoint{
    int x_coord = 0;
    int y_coord = 0; 
    int cost = 0; 
};

struct waypoint origin;

std::vector<waypoint *> tasks; // Vector of tasks where tasks are array of waypoints
std::vector<int> lengths; // Length of tasks 

std::ifstream map;

void read_plans(const char* filename);
void print_coord(struct waypoint point);
double calc_path(struct waypoint *task, int size);
double calc_move(struct waypoint pointA, struct waypoint pointB);

/*
 * Notes: 
 * In hindsight, probably should recursively go through with efficiency optimization
 * Or try shortest path alg?? But need multiple steps, so cannot Dijkstra?
 */
