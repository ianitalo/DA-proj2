#include "Graph.h"
#include <climits>
#include <string>
#include <cmath>
#include "maxHeap.h"
#include <queue>

#include <unordered_set>
using namespace std;

#define INF (INT_MAX/2)

Graph::Graph(int num, bool dir) {
    n = num;
    hasDir = dir;
    nodes = vector<Node>(num+1);
    getInfo(initial_node,final_node);
    cout << "what is the group size? " << endl; cin >> total_group_size; cout << endl;
    if(!cin.good())exit(1);
    cout << "what is the possible group increment? " << endl; cin >> total_group_increment; cout << endl;
    if(!cin.good())exit(1);
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

void Graph::getInfo(int& source,int& destination)
{
    cout << "from what stop is the group leaving? " << endl; cin >> source; cout << endl;
    if(!cin.good()) exit(1);
    cout << "to what stop is the group arriving? " << endl; cin >> destination; cout << endl;
    if(!cin.good()) exit(1);
    if(source > n - 1 || destination < 0)
    {
        cerr << "non-existing stop choice!" << endl;
        exit(1);
    }
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

void Graph::prim(int source) {
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

void Graph::dijkstra1(int source,Graph &rgraph,int parent[]) {
    PriorityQueue queue;
    for (int v=1; v<=n; v++) {
        rgraph.nodes[v].visited = false;
        rgraph.nodes[v].nodeCapacity = 0;
        queue.push(v, 0); //priority queue
    }
    rgraph.nodes[source].nodeCapacity = INF;
    queue.increaseKey(source, INF);
    parent[source] = -1;
    //nodes[source].pred = source;
    while (!queue.empty()) {
        int smallest = queue.top(); queue.pop();
        rgraph.nodes[smallest].visited = true;
        for (auto &edge : rgraph.nodes[smallest].adj) {
            int v = edge.dest;
            int w = edge.capacity;
            if (!rgraph.nodes[v].visited && min(rgraph.nodes[smallest].nodeCapacity,w) > rgraph.nodes[v].nodeCapacity) {
                rgraph.nodes[v].nodeCapacity = min(rgraph.nodes[smallest].nodeCapacity,w);
                queue.increaseKey(v, nodes[v].nodeCapacity);
                //rgraph.nodes[v].pred = smallest;
                parent[v] = smallest;
            }

        }
    }
}


void Graph::problema_1_1()
{
    Graph rGraph = *this;
    int parent[n];
    dijkstra1(initial_node,rGraph,parent);
    cout << "path:" << endl;
    for (int v = final_node; v != initial_node; v = parent[v]) {
        cout << v << endl;
    }
    cout << "with " << rGraph.nodes[final_node].nodeCapacity << " people" << endl;
}

void Graph::problema_2_1()
{
    Graph rGraph = *this;
    list<string> paths = fordFulkerson2_1(initial_node,final_node,total_group_size,&rGraph);
    if(!paths.empty())
    {
        cout << "possible path: " << endl;
        for(string &s: paths)
        {
            cout << s << endl;
        }
    }
    else
    {
        cout << "no possible path with this number of people!" << endl;
    }
}

//ta fazendo o caminho com menor duração no momento
bool Graph::dijkstra2(int source,int dest,Graph &rgraph,int parent[]) {
    MinHeap<int, double> queue(n, -1);
    for (int v=0; v<n; v++) {
        rgraph.nodes[v].dist = INF;
        queue.insert(v, INF); //priority queue
        rgraph.nodes[v].visited = false;
    }
    rgraph.nodes[source].dist = 0;
    queue.decreaseKey(source, 0);
    parent[source] = -1;
    //nodes[source].pred = source;
    while (queue.getSize() > 0) {
        int smallest = queue.removeMin();
        rgraph.nodes[smallest].visited = true;
        for (auto &edge : rgraph.nodes[smallest].adj) {
            int v = edge.dest;
            double w = edge.duration;
            edge.departure = (int)rgraph.nodes[smallest].dist;
            if (!rgraph.nodes[v].visited && rgraph.nodes[smallest].dist + w < rgraph.nodes[v].dist && edge.capacity > 0) {
                rgraph.nodes[v].dist = rgraph.nodes[smallest].dist + w;
                queue.decreaseKey(v, nodes[v].dist);
                //rgraph.nodes[v].pred = smallest;
                parent[v] = smallest;
                if(v == dest) return true;
            }

        }
    }
    return false;
}

bool Graph::bfs2(int source,int dest,Graph &rgraph,int parent[]) {
// initialize all nodes as unvisited
    for (int i=0; i<n; i++) rgraph.nodes[i].visited = false;
    queue <int> q; // queue of unvisited nodes
    q.push(source);
    parent[source] = -1;
    rgraph.nodes[source].visited = true ;
    while (!q.empty ()) { // while there are still unprocessed nodes
        int u = q.front(); q.pop(); // remove first element of q
        for (auto &e : rgraph.nodes[u].adj) {
            int w = e.dest;
            if (!rgraph.nodes[w].visited && e.capacity > 0) {
                if (w == dest) {
                    parent[w] = u;
                    return true;
                }
                // new node!
                q.push(w);
                rgraph.nodes[w].visited = true;
                parent[w] = u;
            }
        }
    }
    return false;
}

Edge* Graph::getEdge(Graph& rGraph,int src, int dest)
{
    for(auto &a : rGraph.nodes[src].adj)
    {
        if(a.dest == dest)
        {
            return &a;
        }
    }
    //se o edge n existe, cria-o para a rede residual
    Edge* edge = new Edge{dest,0,0};
    rGraph.nodes[src].adj.push_back(*edge);
    return edge;
}

list<string> Graph::fordFulkerson2_1(int s, int t,int group_size,Graph* rGraph)
{
    int u, v;
    //residual graph

    int parent[n]; // This array is filled by BFS and to
    // store path
    list<string> paths_used;

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by BFS. Or we can say find the
        // maximum flow through the path found.
        int path_flow = INT_MAX;
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edge = getEdge(*rGraph,u,v);
            path_flow = min(path_flow, edge->capacity);
        }
        if(path_flow > group_size) path_flow = group_size;
        // update residual capacities of the edges and
        // reverse edges along the path
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edgeUV = getEdge(*rGraph,u,v);
            edgeUV->capacity -= path_flow;
            Edge* edgeVU = getEdge(*rGraph,v,u);
            edgeVU->capacity += path_flow;
            paths_used.push_front("from path " + to_string(u) + " to " + to_string(v) + " with " + to_string(path_flow) +" people");
        }
        group_size -= path_flow;
        cout << "path flow: " << path_flow << endl;
        if(group_size <= 0)
        {
            return paths_used;
        }
    }
    return {};
}

void Graph::problema_2_2()
{
    Graph rGraph = *this;
    list<string> paths = fordFulkerson2_2(initial_node,final_node,total_group_size,&rGraph,total_group_increment);
    if(!paths.empty())
    {
        cout << "possible path: " << endl;
        for(string &s: paths)
        {
            cout << s << endl;
        }
    }
    else
    {
        cout << "no possible path with this number of people!" << endl;
    }
}

list<string> Graph::fordFulkerson2_2(int s, int t,int group_size,Graph* rGraph,int group_increment)
{
    int u, v;
    //residual graph
    int total_group_path = group_size + group_increment;

    int parent[n]; // This array is filled by BFS and to
    // store path
    list<string> paths_used;

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by BFS. Or we can say find the
        // maximum flow through the path found.
        int path_flow = INT_MAX;
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edge = getEdge(*rGraph,u,v);
            path_flow = min(path_flow, edge->capacity);
        }
        if(path_flow > total_group_path) path_flow = total_group_path;

        // update residual capacities of the edges and
        // reverse edges along the path
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edgeUV = getEdge(*rGraph,u,v);
            edgeUV->capacity -= path_flow;
            Edge* edgeVU = getEdge(*rGraph,v,u);
            edgeVU->capacity += path_flow;
            paths_used.push_front("from path " + to_string(u) + " to " + to_string(v) + " with " + to_string(path_flow) +" people");
        }
        total_group_path -= path_flow;
        cout << "path flow: " << path_flow << endl;
        if(total_group_path <= 0)
        {
            return paths_used;
        }
    }

    if( group_increment >= total_group_path )
    {
        cout << "its only possible to increment " <<  group_increment - total_group_path << " to this trip" << endl;
        return paths_used;
    }
    return {};
}

void Graph::problema_2_3()
{
    Graph rGraph = *this;
    list<string> paths;
    int max_size = fordFulkerson2_3(initial_node,final_node,&rGraph,&paths);
    if(!paths.empty())
    {
        cout << "the max dimension of the group is " << max_size << endl;
        cout << "possible path: " << endl;
        for(string &s: paths)
        {
            cout << s << endl;
        }
    }
    else
    {
        cout << "no possible path!" << endl;
    }
}

int Graph::fordFulkerson2_3(int s, int t,Graph* rGraph,list<string>* paths_used)
{
    int u, v;
    //residual graph

    int parent[n]; // This array is filled by BFS and to
    // store path

    int max_flow = 0; // There is no flow initially

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by BFS. Or we can say find the
        // maximum flow through the path found.
        int path_flow = INT_MAX;
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edge = getEdge(*rGraph,u,v);
            path_flow = min(path_flow, edge->capacity);
        }

        // update residual capacities of the edges and
        // reverse edges along the path
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edgeUV = getEdge(*rGraph,u,v);
            edgeUV->capacity -= path_flow;
            Edge* edgeVU = getEdge(*rGraph,v,u);
            edgeVU->capacity += path_flow;
            paths_used->push_front("from path " + to_string(u) + " to " + to_string(v) + " with " + to_string(path_flow) +" people");
        }

        // Add path flow to overall flow
        max_flow += path_flow;
    }

    // Return the overall flow
    return max_flow;
}

void Graph::problema_2_4()
{
    Graph rGraph = *this;
    list<string> paths;
    int time = fordFulkerson2_4(initial_node,final_node,total_group_size,&rGraph,&paths);
    if(!paths.empty())
    {
        cout << "possible path: " << endl;
        for(string &s: paths)
        {

        }
        cout << "all group will be on destination in " << time << " minutes" << endl;
    }
    else
    {
        cout << "no possible path with this number of people!" << endl;
    }
}

int Graph::fordFulkerson2_4(int s, int t,int group_size,Graph* rGraph,list<string>* paths_used)
{
    int u, v;
    //residual graph

    int parent[n]; // This array is filled by BFS and to
    // store path

    int max_time = 0;

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by BFS. Or we can say find the
        // maximum flow through the path found.
        // max_time = max_time > rGraph->nodes[t].dist ? max_time : (int)rGraph->nodes[t].dist;
        int path_flow = INT_MAX;
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edge = getEdge(*rGraph,u,v);
            path_flow = min(path_flow, edge->capacity);
        }

        if(path_flow > group_size) path_flow = group_size;
        // update residual capacities of the edges and
        // reverse edges along the path
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edgeUV = getEdge(*rGraph,u,v);
            edgeUV->capacity -= path_flow;
            Edge* edgeVU = getEdge(*rGraph,v,u);
            edgeVU->capacity += path_flow;
            paths_used->push_front("from path " + to_string(u) + " to " + to_string(v) + " with " + to_string(path_flow) +" people");
        }
        group_size -= path_flow;
        cout << "path flow: " << path_flow << endl;
        if(group_size <= 0)
        {
            return (int)rGraph->nodes[t].dist;
        }

    }

    return -1;
}

void Graph::problema_2_5()
{
    Graph rGraph = *this;
    Edge oldEdge;
    list<FlowInfo> paths = fordFulkerson2_5(initial_node,final_node,total_group_size,&rGraph);
    if(!paths.empty())
    {
        cout << "possible path: " << endl;
        for(FlowInfo &s: paths)
        {
            int waitTime;
            Edge* edge = getEdge(rGraph,s.from,s.to);
            if(s.from == initial_node)
            {
                waitTime = 0;
            }
            else
            {
                waitTime = edge->departure - (oldEdge.departure + oldEdge.duration);
            }
            if(waitTime <= 0)
                cout << "from path " + to_string(s.from) + " to " + to_string(s.to) + " with " + to_string(s.flow) +" people at " +
                       to_string(s.distance_from) << " total distance " << " and traveling for " << edge->duration << " minutes " << endl;
            else  {
                cout << "after waiting for " << waitTime << " minutes at " << s.from << ", go to " << s.to << " with "
                     << edge->totalPeople << " people at " << edge->departure << " minutes and traveling for "
                     << edge->duration << endl;
            }
            oldEdge = *edge;
        }
        cout << "and end at " << final_node << " at minute " << rGraph.nodes[final_node].dist << endl;
    }
    else
    {
        cout << "no possible path with this number of people!" << endl;
    }
}


list<FlowInfo> Graph::fordFulkerson2_5(int s, int t, int group_size, Graph* rGraph)
{
    int u, v;
    //residual graph

    int parent[n]; // This array is filled by BFS and to
    // store path
    list<FlowInfo> paths_used;

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by BFS. Or we can say find the
        // maximum flow through the path found.
        int path_flow = INT_MAX;
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edge = getEdge(*rGraph,u,v);
            path_flow = min(path_flow, edge->capacity);
        }
        if(path_flow > group_size) path_flow = group_size;
        // update residual capacities of the edges and
        // reverse edges along the path
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edgeUV = getEdge(*rGraph,u,v);
            string path;
            edgeUV->capacity -= path_flow;
            edgeUV->totalPeople += path_flow;
            Edge* edgeVU = getEdge(*rGraph,v,u);
            edgeVU->capacity += path_flow;
            paths_used.push_front({u,v,path_flow,(int)rGraph->nodes[u].dist,(int)rGraph->nodes[v].dist});
        }
        group_size -= path_flow;
        cout << "path flow: " << path_flow << endl;
        if(group_size <= 0)
        {
            return paths_used;
        }
    }
    return {};
}

// Returns the maximum flow from s to t in the given graph
/*int Graph::fordFulkerson(int s, int t,)
{
    int u, v;
    //residual graph
    Graph rGraph = *this;

    int parent[n]; // This array is filled by BFS and to
    // store path

    int max_flow = 0; // There is no flow initially

    // Augment the flow while there is path from source to
    // sink
    while (bfs2(s, t,rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by BFS. Or we can say find the
        // maximum flow through the path found.
        int path_flow = INT_MAX;
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edge = getEdge(rGraph,u,v);
            path_flow = min(path_flow, edge->capacity);
        }

        // update residual capacities of the edges and
        // reverse edges along the path
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            Edge* edgeUV = getEdge(rGraph,u,v);
            edgeUV->capacity -= path_flow;
            Edge* edgeVU = getEdge(rGraph,v,u);
            edgeVU->capacity += path_flow;
        }

        // Add path flow to overall flow
        max_flow += path_flow;
    }

    // Return the overall flow
    return max_flow;
}*/
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