#include "dijkstra.h"

//Convert VG in a Form Dijkstras algorithm can use
vector<unordered_map<int, int>> dijkstra::graphconversion(vector<Node> v)
{
    vector<unordered_map<int, int>> G;
    // Go through all nodes in the VG
    for (int i = 0; i < v.size(); i++)
    {
        //A nodes connections will be represented as Maps
        Node tmp = v[i];
        unordered_map<int, int> tmpmap;
        //For each of its neighbours create pairs (index, cost) an put them in a map
        for (int j = 0; j < tmp.neighbors.size(); j++)
        {
            int tmpIndex = tmp.neighbors[j].index;
            tmpmap[tmpIndex] = tmp.travelcosts[tmpIndex];
        }
        G.push_back(tmpmap);
    }

    return G;
}
//Find the shortest path from s to t by going recursively from finish back to start
//s is the start, v is the furthest node in the path, and p is the list of predecessors
deque<int> dijkstra::shortest_path(int s, int v, int p[])
{
    deque<int> shortest_path;
    shortest_path.push_front(v);
    while (p[v] != -1)
    {
        v = p[v];
        shortest_path.push_front(v);
    }
    return shortest_path;
}
//Dijkstras Algorithm modified to solve the shortest path problem
deque<int> dijkstra::dijkstra_algorithm(vector<unordered_map<int, int>> G, int s, int t, int size)
{
    int m = size;

    int paths[m]; // holds the lowest possible distance to reach a node
    int pre[m];   // hold the predeccessor of a node on its shortest path
    //initializing both lists with -1 to represent infinity/None.
    for (int i = m - 1; i >= 0; i--)
        paths[i] = -1;

    for (int i = m - 1; i >= 0; i--)
        pre[i] = -1;

    paths[s] = 0; //Set distance from start node to itself to 0

    vector<int> v; //holds all nodes that are not declared permanent
    for (int i = 0; i < m; i++)
        v.push_back(i);
    while (!v.empty()) //The Algorithm terminates when all nodes are declared permanent
    {
        vector<tuple<int, int>> costlist; // Holds the currently known lowest distance to every node still in v
        costlist.clear();
        for (int u : v)
        {
            if (paths[u] != -1)
            {
                costlist.push_back(make_tuple(paths[u], u));
            }
        }
        //Find the next node to explore by selecting the one with the currently lowest distnace to s
        int min = -1;
        for (int i = 0; i < costlist.size(); i++)
        {
            if (get<0>(costlist[i]) < min || min == -1)
            {
                min = get<1>(costlist[i]);
            }
        }
        //If the graph is not connected we break when we have fully explored the connected component
        if (min == -1)
        {
            std::cout << "connected component has been fully explored" << "\n";
            break;
        }

        v.erase(std::remove(v.begin(), v.end(), min), v.end()); // Delete the selected Node from v to mark it as permanent. From: https://stackoverflow.com/a/3385251
        std::cout << "Del: " << min << "\n";
        //Branches from the selected Node to all it's neighbors and trys to find new, shorter paths
        for (std::pair<int, int> u : G[min])
        {
            int alt = paths[min] + G[min][u.first];
            if (paths[u.first] == -1 || alt < paths[u.first]) // If a shorter path has been found
            {
                paths[u.first] = alt; // Update the neighbors costs
                pre[u.first] = min;   // Set the selected node as its predecessor
            }
        }
    }
    //Now find the shortest path from s to t
    deque<int> shortestpath = shortest_path(s, t, pre);
    for (int i = 0; i < shortestpath.size(); i++)
    {
        cout << "v:" << shortestpath[i] << "\n";
    }

    return shortestpath;
}
