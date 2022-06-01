#include "Graph.h"
#include <climits>
#include <string>
#include <cmath>
#include <queue>

#include <unordered_set>
using namespace std;

#define INF (INT_MAX/2)

Graph::Graph(int num, bool dir) : n(num), hasDir(dir), nodes(num+1) {
}

void Graph::addEdge(int source, int destination, int capacity, int duration) {
    if (source < 0 || source > n - 1 || destination < 0 || destination > n - 1){
        cout << source << destination << endl;
        cout << "Can't add the edge." << endl;
        return;
    }
    nodes[source].adj.push_back({destination, capacity, duration});
    if (!hasDir) nodes[destination].adj.push_back({source, capacity, duration});
}

void Graph::bfs(int v) {
// initialize all nodes as unvisited
    for (int i=0; i<n; i++) nodes[i].visited = false;
    queue <int> q; // queue of unvisited nodes
    q.push(v);
    nodes[v].pred = v;
    nodes[v].visited = true ;
    while (!q.empty ()) { // while there are still unprocessed nodes
        int u = q.front(); q.pop(); // remove first element of q
        for (auto &e : nodes[u].adj) {
            int w = e.dest;
            if (!nodes[w].visited) { // new node!
                q.push(w);
                nodes[w].visited = true;
                nodes[w].pred = u;
            }
        }
    }
}
//ta fazendo o caminho com menor duração no momento
void Graph::dijkstra(int source) {
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

                int v = edge.dest;
                double w = edge.duration;
                if (!nodes[v].visited && nodes[smallest].dist + w < nodes[v].dist) {
                    nodes[v].dist = nodes[smallest].dist + w;
                    queue.decreaseKey(v, nodes[v].dist);
                    nodes[v].pred = smallest;
                }

        }
    }
}



double Graph::prim(int source) {
    MinHeap<int, double> heap(n, -1);
    for(int i = 0; i < n; i++){
        nodes[i].dist = INF;
        nodes[i].pred = -1;
        heap.insert(i, nodes[i].dist);
    }
    nodes[source].dist = 0;
    heap.decreaseKey(source, 0);

    while(heap.getSize() > 0){
        int smallest = heap.removeMin();
        for(auto &edge : nodes[smallest].adj){
            if(heap.hasKey(edge.dest)
               && edge.duration < nodes[edge.dest].dist){
                nodes[edge.dest].pred = smallest;
                nodes[edge.dest].dist = edge.duration;
                heap.decreaseKey(edge.dest, edge.duration);
            }
        }
    }
}
/*

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
 */