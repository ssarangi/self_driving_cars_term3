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

#endif // CONSTANTS_H
