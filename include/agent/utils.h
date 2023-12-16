#ifndef AGENT_UTILS_H
#define AGENT_UTILS_H

#include <tuple>
#include <string>
#include <ostream>
#include <vector>

namespace agent
{

class MovementModel;
class PerceivedMap;

/**
 * Representation of a pair of coordinates.
*/
struct Position {
    double x;
    double y;

    std::string toString() const
    {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }
};

/**
 * Validate a pair of coordinates.
 * 
 * @param t_x The x coordinate.
 * @param t_y The y coordinate.
 * @return True if the coordinates are valid, false otherwise.
*/
bool validateCellCoordinates(const int& t_x, const int& t_y);

/**
 * Validate an identifier.
 * 
 * @param t_id The identifier.
 * @return True if the identifier is valid, false otherwise.
*/
bool validateCellId(const int& t_id);

/**
 * Compute an identifier from a pair of coordinates.
 * 
 * It is assumed that the coordinates are valid.
 * 
 * @param t_x The x coordinate.
 * @param t_y The y coordinate.
 * @return The identifier.
*/
int computeCellId(const int& t_x, const int& t_y);

/**
 * Compute the coordinates of a cell from its identifier.
 * 
 * It is assumed that the identifier is valid.
 * 
 * @param t_id The identifier.
 * @return The coordinates.
*/
Position computeCellCoordinates(const int& t_id);

/**
 * Returns the position of the line in the line sensor.
 * 
 * @return position of the line in the line sensor.
*/
double getLinePos();

/**
 * @brief Get the nearest cell to the given coordinates
 * 
 * @param t_x x coordinate
 * @param t_y y coordinate
 * @return int id of the nearest cell
*/
int getNearestCell(double t_x, double t_y);

/**
 * Get the coordinates of a line sensor.
 * 
 * @param t_x The x coordinate of the robot.
 * @param t_y The y coordinate of the robot.
 * @param t_degrees The orientation of the robot.
 * @param t_sensorId The sensor identifier (0..6).
 * @return The coordinates of the sensor.
*/
Position getLineSensorPosition(double t_x, double t_y, double t_degrees, int t_sensorId);

/**
 * Convert degrees to radians.
 * 
 * @param t_deg The angle in degrees.
 * @return The angle in radians.
*/
double deg2rad(double t_deg);

/**
 * Convert radians to degrees.
 * 
 * @param t_rad The angle in radians.
 * @return The angle in degrees.
*/
double rad2deg(double t_rad);

/**
 * Add two radian angles.
 * 
 * @param t_a1 The first angle.
 * @param t_a2 The second angle.
 * @return The sum of the angles.
*/
double addRad(double t_a1, double t_a2);

/**
 * Compute the distance between two points.
 * 
 * @param x1 The x coordinate of the first point.
 * @param y1 The y coordinate of the first point.
 * @param x2 The x coordinate of the second point.
 * @param y2 The y coordinate of the second point.
 * @return The distance between the two points.
*/
double distance(double x1, double y1, double x2, double y2);

}

#endif // AGENT_UTILS_H