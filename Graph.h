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
        bool removed = false;
    };

    struct Node {

        list<Edge> adj;
        bool visited;
        string name;
        string zone;
        double latitude;
        double longitude;
        double dist;
        int pred;
        string code;
        string lineUsed;
        bool removed = false;
    };

    int n;              // Graph size (vertices are numbered from 1 to n)
    bool hasDir;        // false: undirect; true: directed
    vector<Node> nodes; // The list of nodes being represented
    unordered_map<string, int> codeID; // codigo da parada e seu id
    int dest,src;
    bool night = false;


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
    void setCodeIDInfos(unordered_map<string, int> &codeID);
    /// change the time from day to night and vice-versa
    void changeTime();
    ///
    /// \param id id for the stop
    /// \param name name of the stop
    /// \param zone zone of the stop
    /// \param latitude latitude of the stop
    /// \param longitude longitude of the stop
    /// \param code code of the stop
    ///
    void setNodeInfo(int id, const string &name, const string &zone, double latitude, double longitude, string code);
    ///
    /// \param source source stop
    /// \param destination destination stop
    /// \param capacity capacity of passengers
    /// \param duration duration in hours
    ///
    void addEdge(int source, int destination, int capacity, int duration);
    /// size of the graph
    /// \return
    int size() const {return n;}
    ///
    /// \param x the latitude of the local
    /// \param y the longitude of the local
    /// \param distance max distance to walk
    ///
    void localByCoordinates(double x, double y, double distance);
    ///
    /// get the coordinates of the node
    /// \param node the ID of the node
    /// \return
    pair<double,double> getCoordinates(int node);
    ///
    /// generate the possible feet paths to all nodes
    /// complexity: O(n²)
    /// \param distance max distance to walk
    ///
    void generatePossibleFeetPaths(double distance);
    ///
    /// \param name code of the stop
    /// \param distance max distance to walk
    ///
    void localByName(string &name, double distance);
    ///
    /// calculate, using BFS, the path with less stops
    /// complexity:  O(|V| + |E|), where V is the quantity of nodes and E the edges
    /// \param v the node from which the algorithm will run
    ///
    void lessStops(int v);
    ///
    /// \param x latitude from the local
    /// \param y longitude from the local
    /// \param distance max distance to walk
    ///
    void destByCoordinates(double x, double y, double distance);
    ///
    /// \param name code of the line
    /// \param distance max distance to walk
    ///
    void destByName(string &name, double distance);
    ///
    /// create walking paths to the specified node
    /// \param i index of the node
    /// \param d max distance to walk
    ///
    void addCoordinatesEdge(int i, double d);
    ///
    /// calculate, using dijkstra and minHeap, the path with less zones
    /// complexity:   O(|E| log |V|), where V is the quantity of nodes and E the edges
    /// \param s the node from which the algorithm will run
    ///
    void lessZones(int s);
    ///
    /// calculate, using dijkstra and minHeap, the path with less distance
    /// complexity:   O(|E| log |V|), where V is the quantity of nodes and E the edges
    /// \param source the node from which the algorithm will run
    ///
    void lessDistance(int source);
    ///
    /// calculate, using dijkstra and minHeap, the path with less lines passed
    /// complexity:   O(|E| log |V|), where V is the quantity of nodes and E the edges
    /// \param source the node from which the algorithm will run
    ///
    void lessLines(int source);
    ///
    /// uses the lessStops function to calculate the less stops routes
    /// prints the output for the user
    /// complexity: lessStops() + O(n) to show the output
    ///
    void lessStopsPath();
    ///
    /// uses the lessZones function to calculate the less zones routes
    /// prints the output for the user
    /// complexity: lessZones() + O(n) to show the output
    ///
    void lessZonesPath();
    ///
    /// uses the lessDistance function to calculate the less distance routes
    /// prints the output for the user
    /// complexity: lessDistance() + O(n) to show the output
    ///
    void lessDistancePath();
    ///
    /// uses the lessLines function to calculate the less lines routes
    /// prints the output for the user
    /// complexity: lessLines() + O(n) to show the output
    ///
    void lessLinesPath();
    ///
    /// calculate, using Prim and minHeap, the MST from GDM1 and shows the output to the user
    /// complexity: O(|E| log |V|), where V is the quantity of nodes and E the edges
    /// \return
    double primForGDM1();
    ///
    /// remove a stop from the paths by putting the flag removed of the stop to true
    /// \param name code of the stop to be removed
    ///
    void removeStop(string &name);
    ///
    /// remove a line from the paths by putting the flag removed of the line to true
    /// complexity: on average is O(|V| + |E|), worst case is O(|V| + |E| + stringSet find(linear))
    /// \param stringSet set of the lines code to be removed
    ///
    void removeLine(unordered_set<string> stringSet);
    ///
    /// the algorithm that calculates the distance between 2 points
    /// \param lat1 latitude from source
    /// \param lon1 longitude from source
    /// \param lat2 latitude from destination
    /// \param lon2 longitude from destination
    /// \return
    double applyHaversine(double lat1, double lon1, double lat2, double lon2);

};

#endif