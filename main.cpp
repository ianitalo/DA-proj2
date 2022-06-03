#include <iostream>
#include "Graph.h"
#include <fstream>
#include <string>
#include <vector>

using namespace std;

Graph loadGraph(string filename_no){
    fstream f;
    string nodes_number;
    string edges_number;

    string entry_node, exit_node, capacity, duration;

    if(stoi(filename_no) > 9){
        f.open("..\\Tests_B\\in"+filename_no+"_b.txt");
    }else {
        f.open("..\\Tests_B\\in0" + filename_no + "_b.txt");
    }

    getline(f, nodes_number, ' ');
    getline(f, edges_number);

    Graph graph(stoi(nodes_number) +1, true);

    for (int i = 0; i < stoi(edges_number); ++i) {
        getline(f,entry_node,' ');
        getline(f,exit_node,' ');
        getline(f,capacity,' ');
        getline(f, duration, '\n');
        graph.addEdge(stoi(entry_node),stoi(exit_node), stoi(capacity), stoi(duration));
    }
    return graph;

};


int main() {

    Graph graph = loadGraph("1");
    graph.problema_2_1();
    cout << "------------------" << endl;
    graph.problema_2_2();
    cout << "------------------" << endl;
    graph.problema_2_3();
    return 0;
}
