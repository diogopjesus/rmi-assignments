#ifndef AGENT_ROBOT_H
#define AGENT_ROBOT_H

namespace agent
{

namespace robot
{

constexpr double ROBOT_DIAMETER = 1.0;
constexpr double ROBOT_RADIUS = ROBOT_DIAMETER / 2.0;
constexpr int ARENA_WIDTH_IN_UNITS = 28;
constexpr int ARENA_HEIGHT_IN_UNITS = 14;
constexpr int ARENA_WALL_IN_UNITS = 1;
constexpr int NAVIGABLE_WIDTH_IN_UNITS = ARENA_WIDTH_IN_UNITS - 2*ARENA_WALL_IN_UNITS;
constexpr int NAVIGABLE_HEIGHT_IN_UNITS = ARENA_HEIGHT_IN_UNITS - 2*ARENA_WALL_IN_UNITS;
constexpr int NAVIGABLE_WIDTH_IN_CELLS = NAVIGABLE_WIDTH_IN_UNITS / ROBOT_DIAMETER;
constexpr int NAVIGABLE_HEIGHT_IN_CELLS = NAVIGABLE_HEIGHT_IN_UNITS / ROBOT_DIAMETER;
constexpr double LINE_SENSOR_DISTANCE = 0.438; // distance of the central line sensor to the center of the robot
constexpr double LINE_SENSOR_SEPARATION = 0.08; // distance between the two line sensors

}; // namespace robot

}; // namespace agent

#endif // AGENT_ROBOT_H
