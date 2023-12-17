#include "agent/map.h"
#include "agent/utils.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <map>
#include <queue>
#include <set>
#include <fstream>
#include <limits>
#include <iostream>

namespace agent
{

/*** MapWall implementation ***/

void MapWall::update(const int& t_id, const double& t_dir)
{
    if(!validateCellId(t_id))
        throw std::invalid_argument("Map::wall::update - t_id=" + std::to_string(t_id) + " is not a valid identifier");    

    m_corners.clear();

    // compute next cell coordinates
    double dir = t_dir;
    Position c1 = computeCellCoordinates(t_id);
    Position c2{c1.x + 2*round(cos(dir)), c1.y + 2*round(sin(dir))};

    // rotate direction by 90 degrees to compute the wall corners
    dir+=M_PI/2;
    if(dir > M_PI) dir -= 2*M_PI;
    else if(dir < -M_PI) dir += 2*M_PI;

    // add corners
    bool ret = true;
    ret &= addCorner(c1.x + PATH_WALL_WIDTH*0.5*cos(dir), c1.y + PATH_WALL_WIDTH*0.5*sin(dir));
    ret &= addCorner(c1.x - PATH_WALL_WIDTH*0.5*cos(dir), c1.y - PATH_WALL_WIDTH*0.5*sin(dir));
    ret &= addCorner(c2.x - PATH_WALL_WIDTH*0.5*cos(dir), c2.y - PATH_WALL_WIDTH*0.5*sin(dir));
    ret &= addCorner(c2.x + PATH_WALL_WIDTH*0.5*cos(dir), c2.y + PATH_WALL_WIDTH*0.5*sin(dir));

    if(!ret) throw std::logic_error("Something went wrong when updating wall");

    // cells that define the wall
    m_cells = std::make_pair(t_id, computeCellId(c2.x, c2.y));
}

bool MapWall::isInside(const double& t_x, const double& t_y) const
{
    if(m_corners.size() < 4)
        throw std::logic_error("Wall is not fully defined");

    int n = m_corners.size();
    bool inside = false;
    
    for(int i = 0, j = n-1; i < n; j = i++)
    {
        Position cornerI = m_corners[i];
        Position cornerJ = m_corners[j];
        if(((cornerI.y > t_y) != (cornerJ.y > t_y)) &&
           (t_x < (cornerJ.x - cornerI.x) * (t_y - cornerI.y) / (cornerJ.y - cornerI.y) + cornerI.x))
            inside = !inside;
    }

    return inside;
}

bool MapWall::addCorner(const double& t_x, const double& t_y)
{
    if(m_corners.size() >= 4) return false;

    // round the corners to the nearest decimal
    Position corner;
    corner.x = t_x;
    corner.y = t_y;

    m_corners.push_back(corner);
    return true;
}


/*** MapCell implementation ***/

MapCell::MapCell(int t_id)
    : m_id(t_id)
{
    if(!validateCellId(t_id))
        throw std::invalid_argument("MapCell::MapCell - t_id=" + std::to_string(t_id) + " is not a valid identifier");
}

bool MapCell::linkNeighbor(int t_id)
{
    if(!validateCellId(t_id))
        throw std::invalid_argument("MapCell::linkNeighbor - t_id=" + std::to_string(t_id) + " is not a valid identifier");

    Position c = computeCellCoordinates(m_id);
    Position n = computeCellCoordinates(t_id);

    // check if t_id is a neighbor
    int dist = sqrt(pow(c.x - n.x, 2) + pow(c.y - n.y, 2)); // euclidean distance (casted to integer)
    if(dist != 2)
        throw std::invalid_argument("MapCell::linkNeighbor - t_id=" + std::to_string(t_id) +" is not a neighbor of " + std::to_string(m_id) + " (distance=" + std::to_string(dist) + ")");

    // check if cell is already expanded or t_id is already a neighbor
    if(isExpanded() || std::find(m_neighbors.begin(), m_neighbors.end(), t_id) != m_neighbors.end())
        return false;

    // add the neighbor
    m_neighbors.push_back(t_id);

    int x = (int)c.x;
    int y = (int)c.y;
    // corner cells are expanded with 3 neighbors
    bool corner_full = (x == -24 || x == 24) && (y == -10 || y == 10) && m_neighbors.size() >= 3;
    // margin cells are expanded with 5 neighbors
    bool margin_full = (x == -24 || x == 24 || y == -10 || y == 10) && m_neighbors.size() >= 5;
    // middle cells are expanded with 8 neighbors
    bool middle_full = m_neighbors.size() >= 8;

    if(corner_full || margin_full || middle_full)
        expand();

    return true;
}

bool MapCell::unlinkNeighbor(int t_id)
{
    if(!validateCellId(t_id))
        throw std::invalid_argument("MapCell::unlinkNeighbor - t_id=" + std::to_string(t_id) + " is not a valid identifier");

    // check if t_id is a neighbor
    if(std::find(m_neighbors.begin(), m_neighbors.end(), t_id) == m_neighbors.end())
        return false;

    // remove the neighbor
    m_neighbors.erase(std::remove_if(m_neighbors.begin(), m_neighbors.end(),
                [&t_id](const int& nei) {
                    return t_id == nei;
                }
            ), m_neighbors.end()
    );

    return true;
}


/*** PerceivedMap implementation: public methods ***/

bool PerceivedMap::addCell(int t_id)
{
    // Validate coordinates
    if(!validateCellId(t_id)) 
        throw std::invalid_argument("PerceivedMap::addCell - t_id=" + std::to_string(t_id) +" is not a valid identifier");

    // Check if cell is already in map
    for(MapCell& l_c : m_list)
    {
        if(l_c.getId() == t_id)
        {
            return false;
        }
    }

    // Add cell to map
    MapCell c {t_id};
    m_list.push_back(c);
    
    return true;
}

void PerceivedMap::unlinkNeighbor(int t_id1, int t_id2)
{
    // Validate coordinates
    if(!validateCellId(t_id1)) 
        throw std::invalid_argument("PerceivedMap::removeCell - t_id1=" + std::to_string(t_id1) +" is not a valid identifier");

    if(!validateCellId(t_id2)) 
        throw std::invalid_argument("PerceivedMap::removeCell - t_id2=" + std::to_string(t_id2) +" is not a valid identifier");

    // Check if cell is in map
    if(!cellInLocalMap(t_id1) || !cellInLocalMap(t_id2))
        return;
    
    getCell(t_id1).unlinkNeighbor(t_id2);
}

bool PerceivedMap::linkNeighbor(int t_id1, int t_id2)
{
    // Validate identifiers
    if(!validateCellId(t_id1))
        throw std::invalid_argument("PerceivedMap::linkNeighbors - " + std::to_string(t_id1) + " is not a valid identifier");
    if(!validateCellId(t_id2))
        throw std::invalid_argument("PerceivedMap::linkNeighbors - " + std::to_string(t_id2) + " is not a valid identifier");
    
    if(!cellInLocalMap(t_id1))
        throw std::invalid_argument("PerceivedMap::linkNeighbors - " + std::to_string(t_id1) + " is not in map");
    if(!cellInLocalMap(t_id2))
        throw std::invalid_argument("PerceivedMap::linkNeighbors - " + std::to_string(t_id2) + " is not in map");
    
    return getCell(t_id1).linkNeighbor(t_id2);
}

bool PerceivedMap::isNeighbor(const int& t_id1, const int& t_id2)
{
    // Validate identifiers
    if(!validateCellId(t_id1))
        throw std::invalid_argument("PerceivedMap::areNeighbors - " + std::to_string(t_id1) + " is not a valid identifier");
    if(!validateCellId(t_id2))
        throw std::invalid_argument("PerceivedMap::areNeighbors - " + std::to_string(t_id2) + " is not a valid identifier");

    // Check if cells are in map
    if(!cellInLocalMap(t_id1) || !cellInLocalMap(t_id2))
        return false;

    for(int nei : getCell(t_id1).getNeighbors())
        if(nei == t_id2)
            return true;
    
    return false;
}

void PerceivedMap::expandCell(const int& t_id)
{
    // Validate identifier
    if(!validateCellId(t_id))
        throw std::invalid_argument("PerceivedMap::expandCell - " + std::to_string(t_id) + " is not a valid identifier");

    // Check if cell is in map
    if(!cellInLocalMap(t_id))
        throw std::invalid_argument("PerceivedMap::expandCell - " + std::to_string(t_id) + " is not in map");
    
    // Expand cell
    getCell(t_id).expand();
}

bool PerceivedMap::cellIsExpanded(const int& t_id)
{
    // Validate identifier
    if(!validateCellId(t_id))
        throw std::invalid_argument("PerceivedMap::cellIsExpanded - " + std::to_string(t_id) + " is not a valid identifier");

    // Check if cell is in map
    if(!cellInLocalMap(t_id))
        throw std::invalid_argument("PerceivedMap::cellIsExpanded - " + std::to_string(t_id) + " is not in map");

    return getCell(t_id).isExpanded();
}

int PerceivedMap::getNextCell(const int& id)
{
    // Validate identifier
    if(!validateCellId(id))
        throw std::invalid_argument("PerceivedMap::getNextCell - " + std::to_string(id) + " is not a valid identifier");

    // Check if cell is in map
    if(!cellInLocalMap(id))
        throw std::invalid_argument("PerceivedMap::getNextCell - " + std::to_string(id) + " is not in map");

    // Check if path is empty and compute it if necessary
    if(m_path.size() <= 1 || m_path.back() != id)
    {
        // Compute path
        if(!computePathWithoutGoal(id))
        {
            // double check if path is empty
            for(MapCell& c : m_list)
            {
                if(!c.isExpanded() && c.getNeighbors().size() > 0) {
                    // print neighbors
                    std::cout << "Neighbors of " << computeCellCoordinates(c.getId()).toString() << ": ";
                    for(const int& n : c.getNeighbors())
                        std::cout << computeCellCoordinates(n).toString() << " ";
                    std::cout << std::endl;

                    throw std::logic_error("Something went wrong! Map marked as fully expanded but cell " + computeCellCoordinates(c.getId()).toString() + " is not expanded");
                }
            }

            m_complete = m_list.size() > 1;

            // return to starting point
            if(!computePath(id, m_list[0].getId()))
                throw std::logic_error("Something went wrong! Map fully expanded but no path was computed");
            
            return m_path.back();
        }
    }

    // Get next cell
    // TODO: check if this pop first then return next is correct.
    //       The assumption here is that when reconstructing the path,
    //       the first element (last in the vector) is the current cell,
    //       and not the next one.
    if(m_path.size() > 1) m_path.pop_back();
    m_next = m_path.back();
    return m_next;
}

bool PerceivedMap::isComplete()
{
    if(!m_complete) {
        for(const MapCell& c : m_list)
            if(!c.isExpanded()) return false;
    
        m_complete |= m_list.size() > 0;
    }

    return m_complete;
}

void PerceivedMap::writeToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints)
{
    // TODO: check if map is complete, check if checkpoints are valid, check if fname is valid.

    writeMapToFile(t_fname, t_checkpoints);
    writePathToFile(t_fname, t_checkpoints);
}

// TODO: remove this function
std::string PerceivedMap::toString() const
{
    std::string text{};
    for(const MapCell& c : m_list)
    {
        text += "(" + std::to_string(
                (int)(computeCellCoordinates(c.getId()).x)
            ) + "," + std::to_string(
                (int)(computeCellCoordinates(c.getId()).y)
        ) + ") : ";
        for(const int& n : c.getNeighbors())
        {
            text += "(" + std::to_string(
                    (int)(computeCellCoordinates(n).x)
                ) + "," + std::to_string(
                    (int)(computeCellCoordinates(n).y)
                ) + ") ";
        }
        text += "\n";
    }
    return text;
}


/*** PerceivedMap implementation: private methods ***/

MapCell& PerceivedMap::getCell(const int& t_id)
{
    // it is assumed that cell is in local map
    return *find_if(m_list.begin(), m_list.end(),
                [&t_id](const MapCell c) {
                    return t_id == c.getId();
                }
            );
}

bool PerceivedMap::cellInLocalMap(const int& t_id) const
{
    return find_if(m_list.begin(), m_list.end(),
                [&t_id](const MapCell c) {
                    return t_id == c.getId();
                }
            ) != m_list.end();
}

double PerceivedMap::distanceBetweenCells(const int& t_id1, const int& t_id2)
{
    std::set<int> openSet{t_id1};

    std::map<int,int> cameFrom;
    
    std::map<int,double> gScore;
    gScore[t_id1] = 0;

    std::map<int,double> fScore;
    fScore[t_id1] = computeHeuristic(t_id1, t_id2);

    while(!openSet.empty())
    {
        int current = *openSet.begin();
        int key = 0;

        for(const int& id : openSet)
            if(fScore[id] < fScore[current])
                current = id;

        if(current == t_id2)
        {
            int dist = 0;
            while(cameFrom.find(current) != cameFrom.end())
            {
                Position p1 = computeCellCoordinates(current);
                Position p2 = computeCellCoordinates(cameFrom[current]);
                dist+=distance(p1.x, p1.y, p2.x, p2.y);
                current = cameFrom[current];
            }
            return dist;
        }

        openSet.erase(current);
        for(const int& t_neighbor : getCell(current).getNeighbors())
        {
            double tentative_gScore = gScore.at(current) + computeEdgeWeight(current, t_neighbor);

            try { gScore.at(t_neighbor); }
            catch(std::out_of_range& e) { gScore[t_neighbor] = std::numeric_limits<int>::max(); }

            if(tentative_gScore < gScore.at(t_neighbor))
            {
                cameFrom[t_neighbor] = current;
                gScore[t_neighbor] = tentative_gScore;
                fScore[t_neighbor] = gScore.at(t_neighbor) + computeHeuristic(t_neighbor, t_id2);
                openSet.insert(t_neighbor);
            }
        }
    }

    return std::numeric_limits<int>::max();
}

bool PerceivedMap::computePathWithoutGoal(const int& t_start)
{
    std::map<int,bool> visited;
    visited[t_start] = true;

    std::queue<int> queue;
    queue.push(t_start);
    
    std::map<int,int> cameFrom;

    while(!queue.empty())
    {
        int v = queue.front();
        queue.pop();

        if (!cellIsExpanded(v))
        {
            reconstructPath(cameFrom, v);
            return true;
        }

        for(const int& nei : getCell(v).getNeighbors())
        {
            if(!visited[nei])
            {
                visited[nei] = true;
                cameFrom[nei] = v;
                queue.push(nei);
            }
        }
    }

    return false;
}

bool PerceivedMap::computePath(const int& t_start, const int& t_goal)
{
    std::set<int> openSet{t_start};

    std::map<int,int> cameFrom;
    
    std::map<int,double> gScore;
    gScore[t_start] = 0;

    std::map<int,double> fScore;
    fScore[t_start] = computeHeuristic(t_start, t_goal);

    while(!openSet.empty())
    {
        int current = *openSet.begin();
        int key = 0;
        for(const int& id : openSet)
        {
            if(fScore[id] < fScore[current])
            {
                current = id;
            }
        }

        if(current == t_goal)
        {    
            reconstructPath(cameFrom, current);
            return true;
        }

        openSet.erase(current);
        for(const int& t_neighbor : getCell(current).getNeighbors())
        {
            double tentative_gScore = gScore.at(current) + computeEdgeWeight(current, t_neighbor);

            try { gScore.at(t_neighbor); }
            catch(std::out_of_range& e) { gScore[t_neighbor] = std::numeric_limits<double>::max(); }

            if(tentative_gScore < gScore.at(t_neighbor))
            {
                cameFrom[t_neighbor] = current;
                gScore[t_neighbor] = tentative_gScore;
                fScore[t_neighbor] = gScore.at(t_neighbor) + computeHeuristic(t_neighbor, t_goal);
                openSet.insert(t_neighbor);
            }
        }
    }

    return false;
}

double PerceivedMap::computeHeuristic(const int& t_start, const int& t_goal) const
{
    Position p1 = computeCellCoordinates(t_start);
    Position p2 = computeCellCoordinates(t_goal);
    return distance(p1.x, p1.y, p2.x, p2.y);
}

double PerceivedMap::computeEdgeWeight(const int& t_start, const int& t_goal) const
{
    Position p1 = computeCellCoordinates(t_start);
    Position p2 = computeCellCoordinates(t_goal);
    return distance(p1.x, p1.y, p2.x, p2.y);
}

void PerceivedMap::reconstructPath(std::map<int,int>& t_cameFrom, const int& t_current)
{
    m_path.clear();
    m_path.push_back(t_current);

    int current = t_current;
    while(t_cameFrom.find(current) != t_cameFrom.end())
    {
        current = t_cameFrom[current];
        m_path.push_back(current);
    }
}

void PerceivedMap::writeMapToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints) const
{
    char map[21][49];
    for(int i = 0; i < 21; i++)
    {
        for(int j = 0; j < 49; j++)
        {
            map[i][j] = ' ';
        }
    }

    // starting point must be first in t_checkpoints
    for(int i = 0; i < t_checkpoints.size(); i++)
    {
        Position c = computeCellCoordinates(t_checkpoints[i]);
        map[10 - (int)c.y][24 + (int)c.x] = '0' + i;
    }

    // draw the map representation
    for(const MapCell& cell : m_list)
    {
        Position c = computeCellCoordinates(cell.getId());
        int row = 10 - (int)c.y; // 10 - y
        int column = 24 + (int)c.x; // 24 + x

        for(const int& nid : cell.getNeighbors())
        {
            Position n = computeCellCoordinates(nid);

            if(c.x < n.x)
            {
                if(c.y < n.y) map[row - 1][column + 1] = '/';
                else if(c.y > n.y) map[row + 1][column + 1] = '\\';
                else map[row][column + 1] = '-';
            }
            else if (c.x > n.x)
            {
                if(c.y < n.y) map[row - 1][column - 1] = '\\';
                else if(c.y > n.y) map[row + 1][column - 1] = '/';
                else map[row][column - 1] = '-';
            }
            else
            {
                if(c.y < n.y) map[row - 1][column] = '|';
                else if(c.y > n.y) map[row + 1][column] = '|';
            }
        }
    }

    // convert to string
    std::string text{};
    for(int i = 0; i < 21; i++)
    {
        for(int j = 0; j < 49; j++)
            text += map[i][j];

        text += '\n';
    }
    text.pop_back(); // remove last \n

    // write to file
    std::ofstream file(t_fname + ".map");
    file << text;
    file.close();
}

// TODO: rewrite function to a more readable version
void PerceivedMap::writePathToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints)
{
    if(t_checkpoints.empty()) return;

    // store distances between checkpoints
    double cp_distances[t_checkpoints.size()][t_checkpoints.size()];
    for(int i = 0; i < t_checkpoints.size()-1; i++) {
        for(int j = i+1; j < t_checkpoints.size(); j++) {
            double dist = distanceBetweenCells(t_checkpoints[i], t_checkpoints[j]);
            cp_distances[i][j] = dist;
            cp_distances[j][i] = dist;
        }
    }

    std::vector<int> path, best_path; // stored by index of t_checkpoints
    double bestDist = std::numeric_limits<double>::max();
    for(int i=0; i < t_checkpoints.size(); i++) path.push_back(i);
    path.push_back(0);

    // find best path (brute force through all possible combinations).
    if(path.size() > 2)
    {
        do {
            double dist = 0.0;

            for(int i = 0; i < path.size()-1; i++) {
                dist += cp_distances[path[i]][path[i+1]];
            }

            if(dist < bestDist)
            {
                bestDist = dist;
                best_path = path;
            }
            
        } while(std::next_permutation(path.begin()+1, path.begin()+t_checkpoints.size()));
    }

    // lambda that updates the text with the computed path
    auto l_addPathToText = [this](std::string& text)
    {
        // path iterator
        std::vector<int>::reverse_iterator it = m_path.rbegin();
        it++; // skip first element (current cell)
        for(; it != m_path.rend(); ++it)
        {
            Position c = computeCellCoordinates(*it);
            text += std::to_string((int)c.x) + " " + std::to_string((int)c.y) + '\n';
        }
    };

    int first = t_checkpoints[0];
    int previous = first;
    std::string text;
    
    // write first cell
    Position fc = computeCellCoordinates(first);
    text += std::to_string((int)fc.x) + " " + std::to_string((int)fc.y) + '\n';
    
    // write the rest of the path
    for(int i = 1; i < best_path.size(); i++)
    {
        int idx = best_path[i];
        int current = t_checkpoints[idx];
        computePath(previous, current);
        l_addPathToText(text);
        previous = current;
    }

    text.pop_back(); // remove last '\n' character

    std::ofstream file(t_fname + ".path");
    file << text;
    file.close();
}

} // namespace agent
