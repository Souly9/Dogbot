#include <unordered_map>
#include <map>
#include <iostream>
#include <list>
#include <vector>
#include <algorithm>
#include <optional>
#include <deque>
#include <Aria.h>
#include "VisibilityGraph.h"

using namespace std;

class dijkstra
{
    public:
        static vector<unordered_map<int, int>> graphconversion(vector<Node> v);
        static deque<int> shortest_path(int s, int v, int p[]);
        static deque<int> dijkstra_algorithm(vector<unordered_map<int, int>> G, int s, int e, int size);
};