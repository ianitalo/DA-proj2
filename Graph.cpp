#include "graph.h"
#include <climits>
#include <string>
#include <cmath>
#include <queue>

#include <unordered_set>
using namespace std;

#define INF (INT_MAX/2)

Graph::Graph(int num, bool dir) : n(num), hasDir(dir), nodes(num+1) {
}

void Graph::addEdge(int source, int destination, string line, double distance, bool isNight) {
    if (source < 0 || source > n - 1 || destination < 0 || destination > n - 1){
        cout << "Can't add the edge." << endl;
        return;
    }
    nodes[source].adj.push_back({destination, distance, line, isNight});
    if (!hasDir) nodes[destination].adj.push_back({source, distance});
}


pair<double, double> Graph::getCoordinates(int node) {
    return {nodes[node].latitude, nodes[node].longitude};
}

void Graph::setNodeInfo(int id, const string &name, const string &zone, double latitude, double longitude, string code) {
    nodes[id].name = name;
    nodes[id].zone = zone;
    nodes[id].latitude = latitude;
    nodes[id].longitude = longitude;
    nodes[id].code = code;
}

void Graph::setCodeIDInfos(unordered_map<string, int> &codeIDs){
    this->codeID = codeIDs;

}

double Graph::applyHaversine(double lat1, double lon1, double lat2, double lon2){
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;

    double a = pow(sin(dLat / 2), 2) +
               pow(sin(dLon / 2), 2) *
               cos(lat1) * cos(lat2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c * 1000; //retorna em metros
}

void Graph::generatePossibleFeetPaths(double distance)
{
    for(int i = 0; i < n - 1; i++){
        for(int j = i + 1; j < n; j++){
            auto pair1 = getCoordinates(i);
            auto pair2 = getCoordinates(j);
            double d = applyHaversine(
                    pair1.first, pair1.second,
                    pair2.first, pair2.second);

            if(d <= distance)
                addEdge(i, j, "feet", d,night);
        }
    }
}

void Graph::addCoordinatesEdge(int v, double distance)
{
    auto pair1 = getCoordinates(v);
    for(int i = 0; i < n -1; i++)
    {
        auto pair2 = getCoordinates(i);
        double d = applyHaversine(
                pair1.first, pair1.second,
                pair2.first, pair2.second);
        if(d <= distance && v != i)
        {
            addEdge(i, v, "feet", d,night);
            addEdge(v, i, "feet", d,night);
        }
    }
}

void Graph::localByCoordinates(double x, double y, double distance) {
    cout << "These are the stops you can walk to: "<< endl;
    src = n - 2;
    setNodeInfo(src,"origin","",x,y,"ORIG");
    generatePossibleFeetPaths(distance);
    addCoordinatesEdge(src,distance);
    for(auto &a: nodes[src].adj)
    {
        cout << nodes[a.dest].code  << " " << nodes[a.dest].name  << endl;
    }
}

void Graph::localByName(string &name, double distance)
{
    if(codeID.find(name) == codeID.end()) {
        cerr << "name of the stop is invalid!" << endl;
        exit(0);
    }
    src = n - 2;
    setNodeInfo(src,"origin","",nodes[codeID[name]].latitude,nodes[codeID[name]].longitude,"ORIG");
    generatePossibleFeetPaths(distance);
    addCoordinatesEdge(src,distance);
}

void Graph::destByCoordinates(double x, double y, double distance) {

    dest = n - 1;
    setNodeInfo(dest,"destination","",x,y,"DEST");
    addCoordinatesEdge(dest,distance);
}

void Graph::destByName(string &name, double distance)
{
    if(codeID.find(name) == codeID.end()) {
        cerr << "name of the stop is invalid!" << endl;
        exit(0);
    }
    dest = n - 1;
    setNodeInfo(dest,"destination","",nodes[codeID[name]].latitude,nodes[codeID[name]].longitude,"DEST");
    addCoordinatesEdge(dest,distance);

}

double Graph::primForGDM1() {
    int source = codeID["25A1"];
    MinHeap<int, double> heap(n, -1);
    for(int i = 0; i < n; i++){
        nodes[i].dist = INF;
        nodes[i].pred = -1;
        heap.insert(i, nodes[i].dist);
    }
    nodes[source].dist = 0;
    heap.decreaseKey(source, 0);

    list<int> stops;

    while(heap.getSize() > 0){
        int smallest = heap.removeMin();
        if(nodes[smallest].zone != "GDM1") continue;
        stops.push_back(smallest);
        for(auto &edge : nodes[smallest].adj){
            if(heap.hasKey(edge.dest)
               && edge.weight < nodes[edge.dest].dist){
                nodes[edge.dest].pred = smallest;
                nodes[edge.dest].dist = edge.weight;
                heap.decreaseKey(edge.dest, edge.weight);
            }
        }
    }

    double result = 0;
    cout << "CODE  |  NAME  |  ZONE" << endl;
    for(int i : stops)
    {
        cout << nodes[i].code << " " << nodes[i].name << " " << nodes[i].zone << endl;
        result += nodes[i].dist;
    }
    cout << "distancia total: " << result << endl;
    return result;
}

void Graph::lessStopsPath()
{
    lessStops(src);
    list<int> path;
    if(!nodes[dest].visited) return;
    path.push_back(dest);
    int v = dest;
    while(v != src)
    {
        v = nodes[v].pred;
        path.push_front(v);
    }

    cout << "CODE  |  NAME  |  LINE USED" << endl;

    for(int i : path)
    {
        cout << nodes[i].code << " " << nodes[i].name << " " << nodes[i].lineUsed << endl;
    }
}

void Graph::lessStops(int v) {
// initialize all nodes as unvisited
    for (int i=0; i<n; i++) nodes[i].visited = false;
    queue <int> q; // queue of unvisited nodes
    q.push(v);
    nodes[v].pred = v;
    nodes[v].visited = true ;
    while (!q.empty ()) { // while there are still unprocessed nodes
        int u = q.front(); q.pop(); // remove first element of q
        for (auto &e : nodes[u].adj) {
            if(((night && e.isNight) || (!night && !e.isNight) || (e.line == "feet")) && !e.removed) {
                int w = e.dest;
                if (!nodes[w].visited && !nodes[w].removed) { // new node!
                    q.push(w);
                    nodes[w].visited = true;
                    nodes[w].pred = u;
                    nodes[w].lineUsed = e.line;
                }
            }
        }
    }
}


void Graph::lessDistancePath()
{
    lessDistance(src);
    list<int> path;
    if (nodes[dest].dist == INF) return;
    cout << "distancia total: " << nodes[dest].dist << endl;
    path.push_back(dest);
    int v = dest;
    while (v != src) {
        v = nodes[v].pred;
        path.push_front(v);
    }

    cout << "CODE  |  NAME  |  LINE USED  | DISTANCE" << endl;

    for(int i : path)
    {
        cout << nodes[i].code << " " << nodes[i].name << " " << nodes[i].lineUsed << " " << nodes[i].dist<< endl;
    }
}

void Graph::lessDistance(int source) {
    MinHeap<int, double> queue(n, -1);
    for (int v=0; v<n; v++) {
        nodes[v].dist = INF;
        queue.insert(v, INF); //priority queue
        nodes[v].visited = false;
    }
    nodes[source].dist = 0;
    queue.decreaseKey(source, 0);
    nodes[source].pred = source;
    while (queue.getSize() > 0) {
        int smallest = queue.removeMin();
        nodes[smallest].visited = true;
        for (auto &edge : nodes[smallest].adj) {
            if(((night && edge.isNight) || (!night && !edge.isNight) || (edge.line == "feet")) && !edge.removed) {
                int v = edge.dest;
                double w = edge.weight;
                if (!nodes[v].visited && nodes[smallest].dist + w < nodes[v].dist && !nodes[v].removed) {
                    nodes[v].dist = nodes[smallest].dist + w;
                    queue.decreaseKey(v, nodes[v].dist);
                    nodes[v].pred = smallest;
                    nodes[v].lineUsed = edge.line;
                }
            }
        }
    }
}

void Graph::lessZonesPath()
{
    lessZones(src);
    list<int> path;
    if(!nodes[dest].visited) return;
    path.push_back(dest);
    cout << nodes[dest].dist - 1 << " zone(s) passed" << endl;
    int v = dest;
    while(v != src)
    {
        v = nodes[v].pred;
        path.push_front(v);
    }

    cout << "CODE  |  NAME  | ZONE" << endl;

    for(int i : path)
    {
        cout << nodes[i].code << " " << nodes[i].name << " " << nodes[i].zone << endl;
    }
}

void Graph::lessZones(int s)
{
    MinHeap<int, double> q(n, -1);
    for (int v=0; v<n; v++) {
        nodes[v].dist = INF;
        q.insert(v, nodes[v].dist); //priority queue
        nodes[v].visited = false;
    }
    nodes[s].dist = 0;
    q.decreaseKey(s, 0);
    nodes[s].pred = s;
    string lastZone;
    while (q.getSize()>0) {
        int smallest = q.removeMin();
        nodes[smallest].visited = true;
        if(nodes[smallest].zone != " ") lastZone = nodes[smallest].zone;
        for (auto &edge : nodes[smallest].adj) {
            if(((night && edge.isNight) || (!night && !edge.isNight) || (edge.line == "feet")) && !edge.removed) {
                int v = edge.dest;
                if (nodes[v].zone == lastZone)
                    edge.weight = 0;

                else
                    edge.weight = 1;

                double w = edge.weight;
                if (!nodes[v].visited && nodes[smallest].dist + w < nodes[v].dist && !nodes[v].removed) {
                    nodes[v].dist = nodes[smallest].dist + w;
                    q.decreaseKey(v, nodes[v].dist);
                    nodes[v].pred = smallest;
                }
            }
        }
    }
}


void Graph::lessLinesPath() {
    lessLines(src);
    list<int> path;
    if (!nodes[dest].visited) return;
    path.push_back(dest);
    cout << nodes[dest].dist - 2 << " line(s) passed" << endl;
    int v = dest;
    while (v != src) {
        v = nodes[v].pred;
        path.push_front(v);
    }

    cout << "CODE  |  NAME  | LINE USED" << endl;

    for (int i: path) {
        cout << nodes[i].code << " " << nodes[i].name << " " << nodes[i].lineUsed << endl;
    }
}

void Graph::lessLines(int source)
{
    MinHeap<int, double> queue(n, -1);
    for (int v=0; v<n; v++) {
        nodes[v].dist = INF;
        queue.insert(v, nodes[v].dist); //priority queue
        nodes[v].visited = false;
    }
    nodes[source].dist = 0;
    queue.decreaseKey(source, 0);
    nodes[source].pred = source;
    int counter = 0;
    while (queue.getSize() > 0) {
        int smallest = queue.removeMin();
        nodes[smallest].visited = true;
        for (auto &edge : nodes[smallest].adj) {
            if(((night && edge.isNight) || (!night && !edge.isNight) || (edge.line == "feet")) && !edge.removed) {
                int v = edge.dest;
                if (edge.line == "feet" && counter != 0 && nodes[v].name != "destination")
                    edge.weight = INF;
                else if(edge.line != nodes[smallest].lineUsed)
                    edge.weight = 1;
                else edge.weight = 0;

                double w = edge.weight;
                if (!nodes[v].visited && nodes[smallest].dist + w < nodes[v].dist && !nodes[v].removed) {
                    nodes[v].dist = nodes[smallest].dist + w;
                    queue.decreaseKey(v, nodes[v].dist);
                    nodes[v].pred = smallest;
                    nodes[v].lineUsed = edge.line;
                }
            }
        }
        counter = 1;
    }
}

void Graph::changeTime() {
    if(night) {
        night = false;
        cout << "time changed to day" << endl;
        return;
    }
    night = true;
    cout << "time changed to night" << endl;
}

void Graph::removeStop(string &name)
{
    nodes[codeID[name]].removed = true;
    nodes[codeID[name]].adj.clear();
    nodes[codeID[name]].longitude = 0;
    nodes[codeID[name]].latitude = 0;
    nodes[codeID[name]].zone = "";
    nodes[codeID[name]].name = "REMOVED";
}

void Graph::removeLine(unordered_set <string> stringSet)
{
    for(auto& node : nodes)
    {
        for(auto& edge : node.adj)
        {
            if(stringSet.find(edge.line) != stringSet.end()) {
                edge.removed = true;
            }
        }
    }
}