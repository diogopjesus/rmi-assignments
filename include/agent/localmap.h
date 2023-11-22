/* localmap.h
 */

#ifndef AGENT_LOCALMAP_H
#define AGENT_LOCALMAP_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <queue>
#include <fstream>
#include <limits>

namespace agent
{

typedef std::pair<int,int> Coord;
typedef std::vector<int> Path;
typedef std::vector<int> Neighbors;

class LocalMap
{
    // Cell class definition

    class Cell
    {
    public:
        explicit Cell(Coord t_c) : Cell(t_c.first, t_c.second) {}
        Cell(int t_x, int t_y)
            : m_x(t_x), m_y(t_y),
              m_id(computeId(t_x,t_y))
            {
                if(m_x < -10 || m_x > 10) throw std::invalid_argument("t_x is out of range");
                if(m_y < -24 || m_y > 24) throw std::invalid_argument("t_y is out of range");
            }
        virtual ~Cell() = default;

        inline const int& getId() const { return m_id; }
        inline const int& getX() const { return m_x; }
        inline const int& getY() const { return m_y; }
        inline const Neighbors& getNeighbors() const { return m_neighbors; }
        inline bool isExpanded() const { return m_expanded; }

        inline void expand() { m_expanded = true; }
        
        bool addNeighbor(int t_id);

        inline bool operator==(const Cell& t_c) const { return m_x == t_c.m_x && m_y == t_c.m_y && m_id == t_c.m_id; }

    private:
        inline static int computeId(const int& t_x, const int& t_y) { return (t_y + 24) * 21 + t_x + 10; } // (t_x + 10) * 49 + t_y + 24

        const int m_id;
        const int m_x, m_y;
        Neighbors m_neighbors; // neighbors stored by their identifier
        bool m_expanded{false};
    };


// LocalMap class definition

public:
    LocalMap() = default;
    virtual ~LocalMap() = default;

    int addCell(int t_x, int t_y);
    int addCell(int t_x, int t_y, int t_neighbor);
    bool addCellNeighbor(int t_id, int t_neighbor);
    
    void expandCell(const int& t_id);

    bool cellInPerceivedMap(const int& t_id) const;
    bool cellIsExpanded(const int& t_id) const;
    
    const Coord getNextCell(const int& t_id);

    bool isComplete();

    void writeToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints);

    static constexpr int MAX_NEIGHBORS = 8;

private:
    const Coord getCellCoord(const int& t_id) const;
    const Neighbors getCellNeighbors(const int& t_id) const;
    
    bool computePath(const int& t_start, const int& t_goal); // used in writePathToFile (A*)
    float computeHeuristic(const int& t_start, const int& t_goal) const;
    float computeEdgeWeight(const int& t_start, const int& t_goal) const;

    bool computePathWithoutGoal(const int& t_start); // used to find next cell to expand (bfs)
    
    void reconstructPath(std::map<int,int> t_cameFrom, int t_current);
    const Coord getNextInPath();

    void writeMapToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints) const;
    void writePathToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints);

    std::vector<Cell> m_list;
    Path m_path;
    bool m_complete{false};

};

} // namespace agent

#endif // AGENT_LOCALMAP_H
