#include "agent/utils.h"
#include "agent/pose.h"
#include "agent/robot.h"
#include "robSock/RobSock.h"

#include <cmath>
#include <limits>

namespace agent{

double getLinePos()
{
    bool line[7];
    GetLineSensor(line);

    double posOverLine=0;
    int nActiveSensors=0;

    // read sensors
    for (int i = 0; i < N_LINE_ELEMENTS; i++) {
        if(line[i]){
            posOverLine += (double) (i-3);
            nActiveSensors++;
        }
    }

    // Compute the position and scale the measure for the distance between sensors.
    posOverLine = 0.08*posOverLine/nActiveSensors;

    if(nActiveSensors == 0)
        posOverLine = std::numeric_limits<double>::infinity();

    return posOverLine;
}

bool validateCellCoordinates(const int& t_x, const int& t_y)
{
    return t_x >= -24 && t_x <= 24 && (t_x % 2) == 0 
            && t_y >= -10 && t_y <= 10 && (t_y % 2) == 0;
}

bool validateCellId(const int& t_id)
{
    return t_id >= 0 && t_id < (21*49) &&           // between limits
           (t_id % 2) == 0 && (t_id / 49) % 2 == 0; // even column and even row
}

int computeCellId(const int& t_x, const int& t_y)
{
    return (t_x+24)+(t_y+10)*49;
}

Position computeCellCoordinates(const int& t_id)
{
    double x = (double)((t_id % 49)-24);
    double y = (double)((t_id / 49)-10);
    return Position{x,y};
}

int getNearestCell(double t_x, double t_y)
{
    // TODO: re-write function

    int x = floor(t_x);
    int y = floor(t_y);

    if(validateCellCoordinates(x,y))
        return computeCellId(x,y);
    else if(validateCellCoordinates(x+1,y))
        return computeCellId(x+1,y);
    else if(validateCellCoordinates(x,y+1))
        return computeCellId(x,y+1);
    else if(validateCellCoordinates(x+1,y+1))
        return computeCellId(x+1,y+1);

    return -1;    
}

Position getLineSensorPosition(double t_x, double t_y, double t_dir, int t_sensorId)
{
    using namespace robot;

    Position sensorPos{};

    // relative sensor position
    double x = LINE_SENSOR_DISTANCE;
    double y = -t_sensorId*LINE_SENSOR_SEPARATION + (N_LINE_ELEMENTS/2)*LINE_SENSOR_SEPARATION;

    // rotate sensor position
    sensorPos.x = t_x + (x*cos(t_dir) - y*sin(t_dir));
    sensorPos.y = t_y + (x*sin(t_dir) + y*cos(t_dir));

    return sensorPos;
}

double deg2rad(double t_deg)
{
    return t_deg*M_PI/180.0;
}

double rad2deg(double t_rad)
{
    return t_rad*180.0/M_PI;
}

double addRad(double t_a1, double t_a2)
{
    // [-pi, pi]
    double a = t_a1 + t_a2;
    if(a > M_PI) {
        a -= 2*M_PI;
    }
    else if(a < -M_PI) {
        a += 2*M_PI;
    }
    return a;
}

/** helper functions for checkIntersection **/

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are collinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise
// adapted from GeeksForGeeks
static int orientation(Position p, Position q, Position r) 
{ 
    int val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // collinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

// Given three collinear points p, q, r, the function checks if 
// point q lies on line segment 'pr'
// adapted from GeeksForGeeks
static bool onSegment(Position p, Position q, Position r) 
{ 
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && 
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) 
       return true; 
  
    return false; 
}

/** end of helper functions for checkIntersection **/

bool checkIntersection(std::pair<Position,Position> l1, std::pair<Position,Position> l2)
{
    Position p1 = l1.first;
    Position q1 = l1.second;
    Position p2 = l2.first;
    Position q2 = l2.second;

    // Find the four orientations needed for general and 
    // special cases
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are collinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
}

std::vector<int> getPossibleNeighbors(const int& t_id)
{
    std::vector<int> neighbors{};

    // check if the cell is valid
    if(!validateCellId(t_id)) return neighbors;

    // get cell coordinates
    Position pos = computeCellCoordinates(t_id);

    // get possible neighbors
    for(int i = 0; i < 8; i++) {
        int x = pos.x + 2*round(cos(M_PI/4*i));
        int y = pos.y +  2*round(sin(M_PI/4*i));
        neighbors.push_back(computeCellId(x, y));
    }

    return neighbors;
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

} // namespace agent
