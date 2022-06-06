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
    for (int v=0; v<=n; v++) {
        rgraph.nodes[v].visited = false;
        rgraph.nodes[v].nodeCapacity = 0;
        queue.push(v, 0); //priority queue
    }
    rgraph.nodes[source].nodeCapacity = INF;
    queue.increaseKey(source, INF);
    parent[source] = -1;
    //nodes[source].pred = source;
    while (!queue.empty()) {
        int biggest = queue.top(); queue.pop();
        //cout << rgraph.nodes[biggest].dist << " "  << rgraph.nodes[biggest].nodeCapacity << endl;
        rgraph.nodes[biggest].visited = true;
        for (auto &edge : rgraph.nodes[biggest].adj) {
            int v = edge.dest;
            int w = edge.capacity;
            if (!rgraph.nodes[v].visited && min(rgraph.nodes[biggest].nodeCapacity, w) > rgraph.nodes[v].nodeCapacity) {
                rgraph.nodes[v].nodeCapacity = min(rgraph.nodes[biggest].nodeCapacity, w);
                queue.increaseKey(v, rgraph.nodes[v].nodeCapacity);
                //rgraph.nodes[v].pred = biggest;
                parent[v] = biggest;
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
    if(!rGraph.nodes[final_node].nodeCapacity) {cerr << "impossible path" << endl;
        exit(1);}
    for (int v = final_node; v != initial_node; v = parent[v]) {
        cout << "from node " << parent[v] << " to " << v  << endl;
    }
    cout << "with " << rGraph.nodes[final_node].nodeCapacity << " people" << endl;
}

void Graph::problema_1_2()
{
    Graph rGraph = *this;
    int parent[n];
    int bestParent[n];
    dijkstra1_2(initial_node,rGraph,bestParent,INT_MAX);
    cout << "the path with maximum number of people is: " << endl;
    for (int v = final_node; v != initial_node; v = bestParent[v]) {
        cout << "from node " << bestParent[v] << " to " << v  << endl;
    }
    int bestGroup = rGraph.nodes[final_node].nodeCapacity;
    cout << "with " << rGraph.nodes[final_node].nodeCapacity << " people" << endl;
    int maxGroup = rGraph.nodes[final_node].nodeCapacity;
    int maxScore = rGraph.nodes[final_node].nodeCapacity - (int)rGraph.nodes[final_node].dist;
    bool first = true;
    for(int i = 0; i < maxGroup; i++)
    {
        dijkstra1_2(initial_node,rGraph,parent,i);
        if(!rGraph.nodes[final_node].visited) continue;
        if(first) {
            cout << "the path with minimum transhipment is: " << endl;
            for (int v = final_node; v != initial_node; v = parent[v]) {
                cout << "from node " << parent[v] << " to " << v  << endl;
            }
            cout << "with " << rGraph.nodes[final_node].nodeCapacity << " people" << endl;
            first = false;
        }
        int score = rGraph.nodes[final_node].nodeCapacity - (int)rGraph.nodes[final_node].dist;
        if(score > maxScore) {maxScore = score; for(int j = 0; j < n;j++ ){bestParent[j] = parent[j];} bestGroup = rGraph.nodes[final_node].nodeCapacity;}
    }

    if(!first) {
        cout << "the best path considering both variables is: " << endl;
        for (int v = final_node; v != initial_node; v = bestParent[v]) {
            cout << "from node " << parent[v] << " to " << v << endl;
        }
        cout << "with " << bestGroup << " people" << endl;
    }
    else
    {
        cout << "and there's no path with minus transhipment" << endl;
    }
}

void Graph::dijkstra1_2(int source,Graph &rgraph,int parent[],int maxCapacity) {
    PriorityQueue queue;
    for (int v=0; v<=n; v++) {
        rgraph.nodes[v].visited = false;
        rgraph.nodes[v].nodeCapacity = 0;
        rgraph.nodes[v].dist = INF;
        queue.push(v, 0); //priority queue
    }
    rgraph.nodes[source].nodeCapacity = INF;
    rgraph.nodes[source].dist = 1;
    queue.increaseKey(source, INF);
    parent[source] = -1;
    //nodes[source].pred = source;
    while (!queue.empty()) {
        int biggest = queue.top(); queue.pop();
        if(rgraph.nodes[biggest].dist > maxCapacity) continue;
        rgraph.nodes[biggest].visited = true;
        for (auto &edge : rgraph.nodes[biggest].adj) {
            int v = edge.dest;
            int w = edge.capacity;
            if (!rgraph.nodes[v].visited && min(rgraph.nodes[biggest].nodeCapacity, w) > rgraph.nodes[v].nodeCapacity) {
                rgraph.nodes[v].nodeCapacity = min(rgraph.nodes[biggest].nodeCapacity, w);
                rgraph.nodes[v].dist = rgraph.nodes[biggest].dist + 1;
                queue.increaseKey(v, rgraph.nodes[v].nodeCapacity);
                //rgraph.nodes[v].pred = biggest;
                parent[v] = biggest;
            }

        }
    }
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


    int parent[n]; // This array is filled by dijkstra2 and to
    // store path
    list<string> paths_used;

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by dijkstra2. Or we can say find the
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

    int total_group_path = group_size + group_increment;

    int parent[n]; // This array is filled by dijkstra2 and to
    // store path
    list<string> paths_used;

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by dijkstra2. Or we can say find the
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


    int parent[n]; // This array is filled by dijkstra2 and to
    // store path

    int max_flow = 0; // There is no flow initially

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by dijkstra2. Or we can say find the
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
            cout << s << endl;
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


    int parent[n]; // This array is filled by dijkstra2 and to
    // store path

    int max_time = 0;

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by dijkstra2. Or we can say find the
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

    int parent[n]; // This array is filled to
    // store path
    list<FlowInfo> paths_used;

    // Augment the flow while there is path from source to
    // sink
    while (dijkstra2(s, t,*rGraph, parent)) {
        // Find minimum residual capacity of the edges along
        // the path filled by dijkstra2. Or we can say find the
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