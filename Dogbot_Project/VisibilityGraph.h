#pragma once
#include <Aria.h>
#include <vector>
#include <ArMapObject.h>
#include <unordered_map>

// Helper struct to store and access the graph nodes
struct Node
{
    float position[2];
    // Vector of new objects to know the neighbor coordinates and index
    std::vector<Node> neighbors;
    // Index only filled for neighbor nodes
    int index;
    // Stores the travelcosts for all neighbor indices
    std::map<int, float> travelcosts;
};

class VisibilityGraph
{
public:
    /**
 * Central method to compute the graph
 * @param map The map to compute the graph for
 * @param robotPos The start position
 */
    static std::vector<Node> ComputeGraph(ArMap *map, ArPose robotPos);

private:
    static std::vector<Node> ComputeNeighbors(ArPose pos, std::vector<ArLineSegment> *lines, std::map<int, float> &travelcosts, std::vector<ArPose> customNodes);
    static std::vector<Node> m_graph;
    static std::map<ArPose, int> m_indexMap;
    static std::map<Node, float[2]> m_nodePositions;
};

/**
 * Utility functions to compute the new, safer positions of each node on the map
 */
// Moves a point on the Y axis
inline ArPose computeNewPosY(float x, float y, int direction)
{
    float newY = y + 500 * direction;
    return ArPose(x, newY, 0);
}
// Moves a point on the X axis
inline ArPose computeNewPosX(float x, float y, int direction)
{
    float newX = x + 500 * direction;
    return ArPose(newX, y, 0);
}
// Moves a point diagonally to the upper left sector and lower right
inline ArPose computeNewEdgePosYLeftRight(float x, float y, int direction)
{
    float newX = x - 500 * direction;
    float newY = y + 500 * direction;
    return ArPose(newX, newY, 0);
}
// Moves a point diagonally to upper right and lower left sector
inline ArPose computeNewEdgePosRightLeft(float x, float y, int direction)
{
    float newX = x + 500 * direction;
    float newY = y + 500 * direction;
    return ArPose(newX, newY, 0);
}
// Encapsules all calls for to compute the four new positions for an edge node
inline void computeEdgeNode(std::vector<ArPose> &customNodes, ArPose node)
{
    customNodes.push_back(computeNewEdgePosRightLeft(node.getX(), node.getY(), 1));
    customNodes.push_back(computeNewEdgePosRightLeft(node.getX(), node.getY(), -1));
    customNodes.push_back(computeNewEdgePosYLeftRight(node.getX(), node.getY(), 1));
    customNodes.push_back(computeNewEdgePosYLeftRight(node.getX(), node.getY(), -1));
}

inline std::vector<Node> VisibilityGraph::ComputeGraph(ArMap *map, ArPose robotPos)
{
    if (!map)
    {
        ArLog::log(ArLog::Terse, "Warning a map is needed for Graph!");
    }
    std::vector<ArLineSegment> *lines = map->getLines();
    std::vector<Node> m_graph;

    // Get our start
    ArPose startPose = robotPos;
    // Always has the index 0
    m_indexMap[startPose] = 0;

    // Get our goal
    std::list<ArMapObject *> goals = map->findMapObjectsOfType("Goal", true);
    std::list<ArMapObject *>::iterator it = goals.begin();
    ArMapObject *mapObject = *it;
    ArPose endPose = mapObject->getPose();
    m_indexMap[endPose] = lines->size();

    // Helper variables
    std::vector<ArPose> customNodes;
    std::vector<ArPose> edgeNodes;
    std::map<ArPose, int> edgeMap;

    customNodes.push_back(startPose);
    VisibilityGraph::m_indexMap[startPose] = 0;

    // Determine whether a point on the map is an edge or normal node
    // Checks if a node appears as the start/end node of multiple lines
    for (int i = 0; i < lines->size(); ++i)
    {
        ArLineSegment line = lines->at(i);
        ArPose start(line.getX1(), line.getY1(), 0), end(line.getX2(), line.getY2(), 0);

        if (edgeMap.count(start) == 0)
        {
            edgeMap[start] = 1;
        }
        else
        {
            edgeNodes.push_back(start);
        }
        if (edgeMap.count(end) == 0)
            edgeMap[end] = 1;
        else
        {
            edgeNodes.push_back(end);
        }
    }

    // Compute the nodes while checking for duplicates
    // Checking duplicates isn't necessary as of now
    bool startEdge = false, endEdge = false;
    bool computeStart = true, computeEnd = true;
    int offset = 1;
    for (int i = 0; i < lines->size(); ++i)
    {
        ArLineSegment line = lines->at(i);
        ArPose start(line.getX1(), line.getY1(), 0), end(line.getX2(), line.getY2(), 0);

        // Did we already computed the points?
        if (VisibilityGraph::m_indexMap.count(start) != 0)
        {
            computeStart = false;
        }
        if (VisibilityGraph::m_indexMap.count(end) != 0)
        {
            computeEnd = false;
        }
        if (!computeStart && !computeEnd)
        {
            continue;
        }

        // Check if we have an edge position and compute it if true
        for (int a = 0; a < edgeNodes.size(); ++a)
        {
            if (start == edgeNodes[a] && computeStart)
            {
                computeEdgeNode(customNodes, start);
                startEdge = true;
            }
            if (end == edgeNodes[a] && computeEnd)
            {
                computeEdgeNode(customNodes, end);
                endEdge = true;
            }
        }

        // No edge position
        float yDiff = line.getY1() - line.getY2();
        ArPose newStart, newEnd;
        // Does the line run in the Y or X direction?
        if (yDiff < 5)
        {
            if (start.getY() < end.getY())
            {
                newStart = computeNewPosY(start.getX(), start.getY(), -1);
                newEnd = computeNewPosY(end.getX(), end.getY(), 1);
            }
            else
            {
                newStart = computeNewPosY(start.getX(), start.getY(), 1);
                newEnd = computeNewPosY(end.getX(), end.getY(), -1);
            }
        }
        // If Y difference is bigger than 5, the line runs in X direction
        else
        {

            if (start.getX() < end.getX())
            {
                newStart = computeNewPosX(start.getX(), start.getY(), -1);
                newEnd = computeNewPosX(end.getX(), end.getY(), 1);
            }
            else
            {
                newStart = computeNewPosX(start.getX(), start.getY(), 1);
                newEnd = computeNewPosX(end.getX(), end.getY(), -1);
            }
        }
        if (!startEdge)
        {
            customNodes.push_back(newStart);
        }
        if (!endEdge)
        {
            customNodes.push_back(newEnd);
        }

        endEdge = false;
        startEdge = false;
        computeEnd = true;
        computeStart = true;
    }

    // Remove all duplicates by iterating through the vector yet again
    std::map<ArPose, bool> duplicateMap;
    std::vector<ArPose> fixedNodes;
    for (int i = 0; i < customNodes.size(); ++i)
    {
        ArPose tmp = customNodes.at(i);
        if (duplicateMap.count(tmp) != 0)
            continue;
        else
        {
            duplicateMap[tmp] = true;
            fixedNodes.push_back(tmp);
        }
    }
    // And add the endNode which has to be in the vector even if its a duplicate
    endPose.setX(endPose.getX());
    fixedNodes.push_back(endPose);

    // Copy the vector and enter the indices for all nodes
    customNodes.assign(fixedNodes.begin(), fixedNodes.end());
    for (int i = 0; i < customNodes.size(); ++i)
    {
        ArPose start = customNodes.at(i);
        VisibilityGraph::m_indexMap[start] = i;
    }

    // Fill the startNode with data
    Node startNode;
    startNode.position[0] = customNodes.at(0).getX();
    startNode.position[1] = customNodes.at(0).getY();
    startNode.neighbors = ComputeNeighbors(customNodes.at(0), lines, startNode.travelcosts, customNodes);
    m_graph.push_back(startNode);

    // Fill all nodes except the endNode with data
    for (int i = 1; i < customNodes.size() - 1; ++i)
    {
        Node currentNode1;
        currentNode1.position[0] = customNodes.at(i).getX();
        currentNode1.position[1] = customNodes.at(i).getY();
        // Move points out of line so the Nodes dont sit directly in them

        currentNode1.neighbors = ComputeNeighbors(customNodes.at(i), lines, currentNode1.travelcosts, customNodes);

        m_graph.push_back(currentNode1);
    }

    Node endNode;
    ArPose endPos = customNodes.at(customNodes.size() - 1);
    endNode.position[0] = endPos.getX();
    endNode.position[1] = endPos.getY();

    endNode.neighbors = ComputeNeighbors(endPos, lines, endNode.travelcosts, customNodes);
    m_graph.push_back(endNode);

    return m_graph;
}

inline std::vector<Node> VisibilityGraph::ComputeNeighbors(ArPose pos, std::vector<ArLineSegment> *lines, std::map<int, float> &travelcosts, std::vector<ArPose> customNodes)
{
    std::map<ArPose, bool> neighborMap;
    bool intersect = false, intersect2 = false;
    std::vector<Node> neighbors;

    int offset = 1;

    // Shoot a ray to all start and end points of the other lines
    for (int z = 0; z < customNodes.size(); ++z)
    {
        // Grab the next position
        ArPose neighborPos = customNodes.at(z);
        if (neighborPos != pos)
        {
            ArLineSegment currentLine(pos.getX(), pos.getY(), neighborPos.getX(), neighborPos.getY());

            intersect = false;
            ArPose perp;

            // Check if the rays collide with any other lines
            for (int test = 0; test < lines->size(); ++test)
            {
                ArLineSegment possIntersect = lines->at(test);

                // Does the first or second ray intersect with any line?
                if (possIntersect.intersects(&currentLine, &perp))
                {
                    intersect = true;
                }
            }

            // Add the neighbor if the line is intersection free
            if (!intersect)
            {
                Node neighbor;
                neighbor.position[0] = currentLine.getX2();
                neighbor.position[1] = currentLine.getY2();

                int index = VisibilityGraph::m_indexMap[neighborPos];
                neighbor.index = index;
                float dist = sqrt((pos.getX() - currentLine.getX2()) * (pos.getX() - currentLine.getX2()) + (pos.getY() - currentLine.getY2()) * (pos.getY() - currentLine.getY2()));
                travelcosts[index] = dist;

                neighbors.push_back(neighbor);
            }
        }
    }
    return neighbors;
}