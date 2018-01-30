#ifndef COMMONS_H
#define COMMONS_H

int constrain(int a, int b, int c);
int dead_zone(int a, int b);
float constrain_f(float a, float b, float c);
float dead_zone_f(float a, float b);

#define GRAVITY 9810
#define VEHICLE_MASS 30
#define MAX_THRUST 100 //grams
#define DEG2RAD 0.01745
#define RAD2DEG 57.3
#define MAX_VELOCITY 2
#define CIRCLING_R 0.5

#define MODE_RAW 0
#define MODE_POS 1
#define MODE_TRJ 2
#define TakingOff 0
#define Landing 1
#define Automatic 2
#define RawFlying 3
#define Idle 4
#define Hovering 5
#define Circling 6
#define Funny 22
//enum enum_TOL_cmd{TakeOff, Land, Kill}

#define RADIUS_SQUARE 0.4
#define VEHICLE_SIZE 0.15//depends on crazyflies size
#define VEHICLE_EDGE_THRESHOLD 0.065 
#define VEHICLE_DRIFT 0.3//depends on crazyflies maxium velocity
#define REVISE_WEIGHT 0.1
#define ABOUT_EDGE 0.5
#endif
