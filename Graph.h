#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "MinHeap.h"
#include <vector>
#include <list>
#include <iostream>
#include <map>
#include <set>
#include <unordered_set>

using namespace std;

class Graph {
    struct Edge {
        int dest;
        int capacity;
        int duration;
    };

    struct Node {
        list<Edge> adj;
        bool visited;
        double dist;
        int pred;
    };

    int n;              // Graph size (vertices are numbered from 1 to n)
    bool hasDir;        // false: undirect; true: directed
    vector<Node> nodes; // The list of nodes being represented


public:

    Graph()= default;
    ///
    /// \param nodes max size of the graph
    /// \param dir if the graph has direction
    ///
    explicit Graph(int nodes, bool dir = false);
    ///
    /// \param codeID a unordered_map with the code of the stop and its ID
    ///
    void addEdge(int source, int destination, int capacity, int duration);
    /// size of the graph
    /// \return
    int size() const {return n;}

    void bfs(int v);

    void dijkstra(int source);

    double prim(int source);
};

#endif