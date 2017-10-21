#ifndef CONSTANTS_H
#define CONSTANTS_H

enum class Lane {
    LEFT,
    MIDDLE,
    RIGHT
};

enum class VEHICLE_STATE {
    START,
    KEEP_LANE,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
};

const double TRACK_DISTANCE = 6945.564;
const double ROAD_WIDTH = 12.0;
const double POINTS = 50;
const double FRONT_SAFE_DISTANCE = 40.0; // m
const double BACK_SAFE_DISTANCE = -20.0; // m
const double MAX_SPEED = 49.5;
const double MIN_SPEED = 0.0;
const double LANE_WIDTH = 4; // m
const double SAFE_DISTANCE_TO_MAINTAIN = 30; // m
const double VEL_FACTOR = 0.224; // Increase or decrease velocity by this factor
const int    FARTHEST_LEFT_LANE = 0;
const int    FARTHEST_RIGHT_LANE = 2;
const int    MIDDLE_LANE = (FARTHEST_LEFT_LANE + FARTHEST_RIGHT_LANE) / 2;
const int    CAR_RADIUS = 2; // m

#endif // CONSTANTS_H
