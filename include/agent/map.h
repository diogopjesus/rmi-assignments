#ifndef AGENT_LOCALMAP_H
#define AGENT_LOCALMAP_H

#include "agent/utils.h"

#include <vector>
#include <cassert>
#include <map>
#include <string>

namespace agent
{

/**
 * @class MapWall
 * @brief A wall constructed as in the ciberRato environment.
 * A wall is considered a rectangle that connects 2 cells.
 * This class serves the purpose of checking if a point is inside a wall.
 * The points in cause are line sensor coordinates. This will help detect
 * if the line sensor is inside a wall or not.
*/
class MapWall
{
public:
    MapWall() = default;
    virtual ~MapWall() = default;

    /**
     * Re-center wall to a cell and compute its coordinates.
     * 
     * @param t_id The identifier of the cell.
     * @param t_dir The direction of the wall (in radians).
     * @param isRight True if the angle is multiple of 90, false otherwise.
    */
    void update(const int& t_id, const double& t_dir);

    /**
     * Check if a point is inside the wall.
     * 
     * @param t_x The x coordinate of the point.
     * @param t_y The y coordinate of the point.
     * @return True if the point is inside the wall, false otherwise.
    */
    bool isInside(const double& t_x, const double& t_y) const;

    /**
     * Get the cells that define the wall.
     * 
     * @return A pair of cells.
    */
    inline const std::pair<int,int>& getCells() const { return m_cells; }

    static constexpr double PATH_WALL_WIDTH{0.2};

private:
    /**
     * Add corner to the corner list.
     * 
     * @param t_x The x coordinate of the corner.
     * @param t_y The y coordinate of the corner.
     * @return True if the corner was added, false otherwise
    */
    bool addCorner(const double& t_x, const double& t_y);

    std::vector<Position> m_corners;
    std::pair<int,int> m_cells;
};

/**
 * @class MapCell
 * @brief Representation of a cell in the map of a ciberRato environment.
 * Used internally in the PerceivedMap class.
*/
class MapCell
{
public:
    MapCell(int t_id);
    virtual ~MapCell() = default;

    inline const int& getId() const { return m_id; }
    inline const std::vector<int>& getNeighbors() const { return m_neighbors; }
    inline bool isExpanded() const { return m_expanded; }

    /**
     * Link a neighbor to the cell.
     * 
     * @param t_id The identifier of the neighbor.
     * @return True if the neighbor was added, false otherwise.
    */
    bool linkNeighbor(int t_id);

    /**
     * Unlink a neighbor from the cell.
     * 
     * @param t_id The identifier of the neighbor.
     * @return True if the neighbor was removed, false otherwise.
    */
    bool unlinkNeighbor(int t_id);

    /**
     * Set cell expansion.
     * 
     * A cell is expanded all neighbors are added to the list of neighbors.
     * A cell can be unexpanded because it might detect some neighbors that are not connected.
     * This way, the cell can be expanded again when the connection is verified.
     * 
     * @return True if the cell was expanded, false otherwise.
    */
    inline void setExpanded(bool t_expanded) { m_expanded = t_expanded; } 

    /**
     * 
    */

private:
    int m_id;
    std::vector<int> m_neighbors; // stored by their identifier
    bool m_expanded{false};
};


/**
 * @class LocalMap
 * @brief A representation of the map perceived by the agent.
 * Private methods do not check if the parameters are valid.
*/
class PerceivedMap
{
public:
    PerceivedMap() = default;
    virtual ~PerceivedMap() = default;

    /**
     * Reset the map.
    */
    void reset();

    /**
     * Add a cell to the map.
     * 
     * @param t_id The identifier of the cell.
     * @return True if the cell was added, false otherwise.
    */
    bool addCell(int t_id);

    /**
     * Unlink a cell as neighbor of another cell.
     * 
     * @param t_id1 The identifier of the cell.
     * @param t_id2 The identifier of the neighbor.
    */
    void unlinkNeighbor(int t_id1, int t_id2);

    /**
     * Get cell neighbors.
     * 
     * @param t_id The identifier of the cell.
     * @return A vector with the identifiers of the neighbors.
    */
    inline std::vector<int> getNeighbors(const int& t_id) { return getCell(t_id).getNeighbors(); }

    /**
     * Link a cell as neighbor of another cell.
     * 
     * Do not link the neighbor to the cell. This must be done separately.
     * This function should be called when the cell is being traversed.
     * 
     * @param t_id2 The identifier of the cell.
     * @param t_id2 The identifier of the neighbor.
     * @return True if the neighbor was added, false otherwise.
    */
    bool linkNeighbor(int t_id1, int t_id2);

    /**
     * Check if a cell is neighbor of another.
     * 
     * @param t_id1 The identifier of the cell.
     * @param t_id2 The identifier of the neighbor.
     * @return True if is neighbor, false otherwise.
    */
    bool isNeighbor(const int& t_id1, const int& t_id2);

    /**
     * Set a cell expansion.
     * 
     * @param t_id The identifier of the cell.
     * @param t_expanded True if the cell is expanded, false otherwise.
    */
    void setCellExpanded(const int& t_id, bool t_expanded);

    /**
     * Check if a cell is expanded.
     * 
     * @param t_id The identifier of the cell.
     * @return True if the cell is expanded, false otherwise.
    */
    bool cellIsExpanded(const int& t_id);

    /**
     * Get the coordinates of the next cell to traverse.
     * 
     * @param t_id The identifier of the cell from which to get the next cell.
     * @return The identifier of the next cell. If the map is complete, returns t_id.
    */
    int getNextCell(const int& t_id);

    /**
     * Check if the map is complete.
     * 
     * A map is complete if all cells are expanded.
     * 
     * @return True if the map is complete, false otherwise.
    */
    bool isComplete();

    /**
     * Write the map to a file.
     * 
     * @param t_fname The filename.
     * @param t_checkpoints The list of checkpoints by id.
    */
    void writeToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints);

    /**
     * Write the map to a string.
     * 
     * @return The string.
    */
    std::string toString() const;

private:
    /**
     * Get the cell corresponding to an identifier.
     * 
     * This function must be private to ensure that 
     * no cell is changed outside of the class.
     * 
     * @param t_id The identifier of the cell.
    */
    MapCell& getCell(const int& t_id);

    /**
     * Check if a cell is in the map.
     * 
     * @param t_id The identifier of the cell.
     * @return True if the cell is in the map, false otherwise.
    */
    bool cellInLocalMap(const int& t_id) const;

    /**
     * Compute the distance between two cells.
     * 
     * Adapted from the A* algorithm.
     * 
     * @param t_id1 The identifier of the first cell.
     * @param t_id2 The identifier of the second cell.
     * @return The distance between the cells.
    */
    double distanceBetweenCells(const int& t_id1, const int& t_id2);

    /**
     * Compute shortest path to an open cell.
     * 
     * Used to find the next cell to expand.
     * Uses breadth-first search.
     * 
     * @param t_start The identifier of the starting cell.
     * @return True if a path was found, false otherwise.
    */
    bool computePathWithoutGoal(const int& t_start);

    /**
     * Compute the shortest path between two cells.
     * 
     * Uses A* algorithm.
     * Used to find shortest path in writePathToFile.
     * 
     * @param t_start The identifier of the starting cell.
     * @param t_goal The identifier of the goal cell.
     * @return True if a path was found, false otherwise.
    */
    bool computePath(const int& t_start, const int& t_goal);
    
    /**
     * Compute the heuristic between two cells.
     * 
     * Used in A* algorithm.
     * 
     * In this case, the heuristic is the euclidean distance between the two cells.
     * 
     * @param t_start The identifier of the starting cell.
     * @param t_goal The identifier of the goal cell.
     * @return The heuristic.
    */
    double computeHeuristic(const int& t_start, const int& t_goal) const;
    
    /**
     * Compute the weight of an edge between two cells.
     * 
     * Used in A* algorithm.
     * 
     * In this case, all edges have the same weight.
     * 
     * @param t_start The identifier of the starting cell.
     * @param t_goal The identifier of the goal cell.
     * @return The weight of the edge.
    */
    double computeEdgeWeight(const int& t_start, const int& t_goal) const;

    /**
     * Reconstruct the path traversed by the algorithm.
     * 
     * The path is stored in reverse order (start position is the last element).
     * Stores the path inside the class.
     * 
     * @param t_cameFrom The map of predecessors.
     * @param t_current The identifier of the current cell.
    */
    void reconstructPath(std::map<int,int>& t_cameFrom, const int& t_current);

    /**
     * Write the map to a file.
     * 
     * @param t_fname The filename.
     * @param t_checkpoints The list of checkpoints.
    */
    void writeMapToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints) const;

    /**
     * Write the path to a file.
     * 
     * @param t_fname The filename.
     * @param t_checkpoints The list of checkpoints.
    */
    void writePathToFile(const std::string& t_fname, const std::vector<int>& t_checkpoints);

    std::vector<MapCell> m_list;
    std::vector<int> m_path;
    int m_next{-1};
    bool m_complete{false};
};

} // namespace agent

#endif // AGENT_LOCALMAP_H
