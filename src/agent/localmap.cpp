/* localmap.cpp
 */

#include "agent/localmap.h"

namespace agent
{

// Cell implementation

bool LocalMap::Cell::addNeighbor(int t_id)
{
    if(isExpanded()) return false;

    for(const int& nei : m_neighbors)
    {
        if(nei == t_id) return false;
    }

    m_neighbors.push_back(t_id);

    if(m_neighbors.size() >= 8) expand(); 

    return true;
}


// LocalMap implementation

int LocalMap::addCell(int t_x, int t_y)
{
    if((t_x % 2) != 0 || (t_y % 2) != 0) throw std::invalid_argument("t_x and t_y are not valid coordinates");

    // check if cell is already in map
    for(Cell& c : m_list)
    {
        if(c.getX() == t_x && c.getY() == t_y) return c.getId();
    }
    
    Cell c {t_x, t_y};
    m_list.push_back(c);
    return c.getId();
}

int LocalMap::addCell(int t_x, int t_y, int t_neighbor)
{
    int id = addCell(t_x, t_y);
    addCellNeighbor(id, t_neighbor);
    return id;
}

bool LocalMap::addCellNeighbor(int t_id, int t_neighbor)
{
    Coord cid = getCellCoord(t_id);
    Coord cnei = getCellCoord(t_neighbor);
    if ((cnei.first - cid.first)%2 != 0 || (cnei.second - cid.second)%2 != 0)
    {
        throw std::invalid_argument("Cells are not neighbors");
    }

    bool ret = false;
    for(Cell& c : m_list)
    {
        if(c.getId() == t_id)
        {        
            ret = c.addNeighbor(t_neighbor);
        }
    }

    for(Cell& c : m_list)
    {
        if(c.getId() == t_neighbor)
        {
            return ret || c.addNeighbor(t_id);
        }
    }

    throw std::invalid_argument("Cell not found");
}

void LocalMap::expandCell(const int& t_id)
{
    for(Cell& c : m_list)
    {
        if(c.getId() == t_id) c.expand();
    }
    throw std::invalid_argument("Cell not found");
}

bool LocalMap::cellInPerceivedMap(const int& t_id) const
{
    for(const Cell& c : m_list)
    {
        if(c.getId() == t_id) return true;
    }
    return false;
}

bool LocalMap::cellIsExpanded(const int& t_id) const
{
    for(const Cell& c : m_list)
    {
        if(c.getId() == t_id) return c.isExpanded();
    }
    throw std::invalid_argument("Cell not found");
}

const Coord LocalMap::getNextCell(const int& t_id)
{
    if(!computePathWithoutGoal(t_id))
    {
        for(Cell& c : m_list)
        {
            if(!c.isExpanded())
            {
                throw std::logic_error("Something went wrong! Map not fully expanded and no path was computed");
            }
        }

        m_complete = true;
        return getCellCoord(t_id);
    }

    return getNextInPath();
}

bool LocalMap::isComplete()
{
    if(!m_complete)
    {
        for(const Cell& c : m_list)
        {
            if(!c.isExpanded()) return false;
        }
        m_complete = true;
    }
    return m_complete;
}

void LocalMap::writeToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints)
{
    writeMapToFile(t_fname, t_checkpoints);
    writePathToFile(t_fname, t_checkpoints);
}


const Coord LocalMap::getCellCoord(const int& t_id) const
{
    for(const Cell& c : m_list)
    {
        if(c.getId() == t_id) return std::make_pair(c.getX(), c.getY());
    }
    throw std::invalid_argument("Cell not found");
}

const Neighbors LocalMap::getCellNeighbors(const int& t_id) const
{
    for(const Cell& c : m_list)
    {
        if(c.getId() == t_id) return c.getNeighbors();
    }
    throw std::invalid_argument("Cell not found");
}

bool LocalMap::computePath(const int& t_start, const int& t_goal)
{
    std::set<int> openSet{t_start};

    std::map<int,int> cameFrom;
    
    std::map<int,int> gScore;
    gScore[t_start] = 0;

    std::map<int,int> fScore;
    fScore[t_start] = computeHeuristic(t_start, t_goal);


    while(!openSet.empty())
    {
        int current = *openSet.begin();
        
        if(current == t_goal) 
        {    
            reconstructPath(cameFrom, current);
            return true;
        }

        for(const int& t_neighbor : getCellNeighbors(current))
        {
            int tentative_gScore = gScore.at(current) + computeEdgeWeight(current, t_neighbor);

            try { gScore.at(t_neighbor); }
            catch(std::out_of_range& e) { gScore[t_neighbor] = std::numeric_limits<int>::max(); }

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

float LocalMap::computeHeuristic(const int& t_start, const int& t_goal) const
{
    Coord a = getCellCoord(t_start);
    Coord b = getCellCoord(t_goal);

    return sqrt( pow(a.first - b.first, 2) + pow(a.second - b.second, 2) );
}

float LocalMap::computeEdgeWeight(const int& t_start, const int& t_goal) const
{
    return 1.0f; // All edges have the same weight
}

bool LocalMap::computePathWithoutGoal(const int& t_start)
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

        for(const int& nei : getCellNeighbors(v))
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

void LocalMap::reconstructPath(std::map<int,int> t_cameFrom, int current)
{
    m_path.clear();
    m_path.push_back(current);

    while(t_cameFrom.find(current) != t_cameFrom.end())
    {
        current = t_cameFrom[current];
        m_path.push_back(current);
    }
}

const Coord LocalMap::getNextInPath()
{
    if(m_path.empty()) throw std::invalid_argument("Path is empty");
    
    Coord ret = getCellCoord(m_path.back());
    m_path.pop_back();
    return ret;
}

void LocalMap::writeMapToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints) const
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
    int checkNum = 0;
    for(const int& cid : t_checkpoints)
    {
        Coord coord = getCellCoord(cid);
        map[coord.first + 10][coord.second + 24] = '0' + checkNum;
    }

    for(const Cell& c : m_list)
    {        
        int cx = c.getX() + 10;
        int cy = c.getY() + 24;

        for(const int& nei : c.getNeighbors())
        {
            Coord neiCoord = getCellCoord(nei);
            int diffX = neiCoord.first - c.getX();
            int diffY = neiCoord.second - c.getY();

            if(diffX > 0)
            {
                if(diffY < 0) map[cx + 1][cy + 1] = '/';
                else if(diffY > 0) map[cx + 1][cy - 1] = '\\';
                else map[cx + 1][cy] = '-';
            }
            else if (diffX < 0)
            {
                if(diffY < 0) map[cx - 1][cy + 1] = '\\';
                else if(diffY > 0) map[cx - 1][cy - 1] = '/';
                else map[cx - 1][cy] = '-';
            }
            else
            {
                if(diffY < 0) map[cx][cy + 1] = '|';
                else if(diffY > 0) map[cx][cy - 1] = '|';
            }
        }
    }

    // convert to string
    std::string text{};
    for(int i = 0; i < 21; i++)
    {
        text += std::string(map[i]) + '\n';
    }

    std::ofstream file(t_fname + ".map");
    file << text;
    file.close();
}

void LocalMap::writePathToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints)
{
    std::string text{};
    
    int first = t_checkpoints.at(0);
    int previous = first;

    auto l_updateText = [this](std::string& text)
    {
        for(auto i = m_path.rbegin(); i != m_path.rend(); ++i)
        {
            Coord coord = getCellCoord(*i);
            text += std::to_string(coord.first) + " " + std::to_string(coord.second) + '\n';
        }
    };

    for(const int& cid : t_checkpoints)
    {
        if (cid == previous) continue;
        computePath(previous, cid);
        l_updateText(text);
        previous = cid;
    }

    computePath(previous, first);
    l_updateText(text);

    std::ofstream file(t_fname + ".path");
    file << text;
    file.close();
}

}; // namespace agent
