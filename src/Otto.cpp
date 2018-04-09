/*
 * Author: Vincent Yuan
 * Create Date: Mar 26, 2018 
 * Last Edit: Mar 27, 2018 
 * File: Otto.cpp 
 * Purpose: Holds main and functions declrataions for processing Otto challenge 
 *
 */ 

#include "Otto.h"

using namespace std;

int main (int argc, const char* argv[]){
    read_plans(argv[1]);
    int tasks_count = 0;
    for (vector<waypoint*>::const_iterator i = tasks.begin(); i != tasks.end(); ++i){
        if (lengths[tasks_count] > 0){
            double score = calc_path(*i, lengths[tasks_count]);
            cout << fixed << setprecision(3) << score << endl;
        }
        tasks_count ++;
    }
    return 0;
}


/*
 * Function: calc_path
 * Purpose: Calculate score for particular task's path
 * Input: Array of waypoints, size of array
 * Output: Score of task
 * Approach:
            For everypoint, we need to consider direct distance, and summed distance
            For summed distance, we need to give 10sec for every point we cross 
            For direct distance, need to account for sum of penalties
            For pick the min of the two and call it move_cost, then minimize move_cost
            To optimize the summed distance, keep cum_score
 */
double calc_path(struct waypoint *task, int size){
    double score,min_score,move_cost,dir_score,tmp_min = 0.0;
    double cum_score,eff,min_eff,cum_pen = 0.0; //cumulitive score and penalty
    int min_index,point = 0; //smallest index 
    struct waypoint cur_point,next_point,index_point;
    while (point < size){
        //Take distance of cur_point and find min of sum of points
        cum_score = eff = min_eff = min_score =  cum_pen = 0.0;
        cur_point = task[point]; //Update current location of robot
        index_point = cur_point;//Restart index at my current location
        bool skip = false; //Do we have a skip we can perform?
        for (int next = point+1; next < size; next++){
            next_point = task[next];
            dir_score = calc_move(cur_point,next_point)+10; 
            cum_pen += next_point.cost;
            cum_score += calc_move(index_point,next_point)+10;
            dir_score += cum_pen;
            //find skip with larges efficiency, defined as percentage time saved 
            //relative to cumulutive traversal
            eff = (cum_score - dir_score)/cum_score;
            #if DEBUG
                cout << "\nCumulitive: " << cum_score << " Direct: " << dir_score;
                cout << "EFF: " << eff << " min_eff: " << min_eff << endl;
                cout << "             From: ";
                print_coord(cur_point);
                cout << " to ";  print_coord(next_point); 
            #endif
            if (min_score == 0.0){
                //init min score 
                if (cum_score <= dir_score){
                    //cumulitive is better
                    min_score = cum_score;
                    min_index = next;
                }
                else {
                    //skipping is better
                    skip = true; 
                    min_score = dir_score;
                    min_index = next;
                    min_eff = eff;
                }
            }
            else{
                //if min_score has been initialized
                if (!skip){
                    //if we are not skipping
                    if (dir_score < cum_score){
                        #if DEBUG
                            cout << "Found skip with eff: " << eff << " score of: " << min_score << endl;
                        #endif
                        //we just found first skip
                        min_score = dir_score; 
                        //smallest is now a skip
                        skip = true;
                        min_index = next; 
                        min_eff = eff;
                    } 
                }
                else {
                    //if we are skipping we want most efficient skip
                    if (dir_score < cum_score){
                        #if DEBUG
                            cout << "Found skip with eff: " << eff << " score of: " << dir_score << endl;
                        #endif
                        if (eff < min_eff){
                            //there exists a more efficient skip
                            min_score = dir_score;
                            min_index = next;
                            min_eff = eff;
                        }
                    }
                }
            }
            index_point = task[next];//Moving virtual index forward
        }     
        if (!skip){ 
            min_score = calc_move(cur_point,task[point+1])+10;
            #if DEBUG
                cout << "\n     Not skipping, using: " << min_score;
            #endif
            point ++;
        }
        else{
            #if DEBUG
                cout << "\n     skipping, using: " << min_score;
                cout << "Final eff: " << min_eff << endl;
            #endif
            point = min_index;
        }
        score += min_score;
        #if DEBUG
            cout << "\nOtto is at point: ";
            print_coord(task[point]);
        #endif
        if (point == size-1){
            return score;
        }
    }
    return score; 
}

/*
 * Function: print_coord
 * Purpose: To print coords because I'm lazy
 * Input: waypoint
 * Output: void 
 */
void print_coord(struct waypoint point){
    cout << "[" << point.x_coord << "," << point.y_coord << "]";    
} 


/*
 * Function: move_score
 * Purpose: calculate time score to travel to point
 * Output: score of moving 
 */
double calc_move(struct waypoint pointA, struct waypoint pointB){  
    int x_dis = pointB.x_coord-pointA.x_coord;
    int y_dis = pointB.y_coord-pointA.y_coord;
    return (double)sqrt(x_dis*x_dis+y_dis*y_dis)/2;
}

/* 
 * Function: read_plans
 * Purpose: Open file, parse into usable data format
 * Output: None
 * Note: Be careful to seperate the individual trials
 */
 void read_plans(const char* filename){
    map.open(filename, ifstream::in);
    int line_num = 0;
    char line[13]; //3x3digit +  2 spaces + line ending (2 for dos, 1 for unix) = 13  
    char* num;
    struct waypoint *task = NULL;
    int count = 1;
    while (map.getline(line,13)){ 
        //Retrieve data by row
        num = strtok(line," "); 
        int x_coord = atoi(num); 
        num = strtok(NULL," "); 
        if (num == NULL){ 
            if(count > 1){
                //Add our end point
                task[count].x_coord=100;
                task[count].y_coord=100;
                task[count].cost=0;
                tasks.push_back(task);
                lengths.push_back(count+1);
            }
            //Nothing after means this is size of our task's path
            count = 1; //Intialize our waypoints 
            task = new waypoint[x_coord+2];  
        }
        else{ 
            //Populate task accordingly
            double y_coord=atoi(num);
            num = strtok(NULL," ");
            if (num != NULL){
                task[count].cost = atoi(num);
                task[count].x_coord = x_coord;
                task[count].y_coord = y_coord;
                count ++;
                }
        }
    
    }
    map.close();
}
