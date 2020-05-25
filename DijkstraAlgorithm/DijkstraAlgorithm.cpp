// DijkstraAlgorithm.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <fstream>

using namespace std;

// Create custom type of pair of ints for an Edge
typedef pair<int, int> Edge;

// Adds an Edge connection between vertices, also keeps track of its weight
void addEdge(vector<pair<int, int>> G[], int u, int v, int weight) {
    G[u].push_back(make_pair(v, weight));
    G[v].push_back(make_pair(u, weight));
}

// Prints the path from the specified parent vector 
void printPath(vector<int>& parent, int i) {
    // Need to make the integer values of Graph back into chars so we add 65 ('A' in ASCII) back to the values before printing
    if (parent[i] == -1) {
        cout << "The path is: " << char(i + 65) << " ";
        return;
    }

    printPath(parent, parent[i]);
    cout << char(i + 65) << " ";
}

// Uses Dijkstra's Algorithm with a graph of vectors of pairs that takes as input the total number of vertices and the source vertex to start path traversal
void findShortestPath(vector<pair<int, int>> G[], int V, int src) {
    // Create the minHeap from STL properties of priority queue, uses comparator to determine ordering within "tree"
    priority_queue<Edge,  vector<Edge>, greater<Edge>> minHeap;
    // Declare distance vector and set everything to "infinity"
    vector<int> dist(V, INT_MAX);
    // Create the parent vector for the shortest path
    vector<int> parent(V, 0);
    // Add the source to the min heap with 0 as its starting location in the graph
    minHeap.push(make_pair(0, src));
    // Declare the parent source vertex slot to be -1 (used to find path later)
    parent[src] = -1;
    // Set the distance source vertex slot to be 0 distance
    dist[src] = 0;

    // Loop until the heap is empty
    while (!minHeap.empty()) {
        // Declare the min value of the heap as u
        int u = minHeap.top().second;
        // Remove from the heap for processing
        minHeap.pop(); 

        // For all the neighbors of the current edge connection
        for (Edge x : G[u]) {
            int v = x.first;
            int weight = x.second;
            // Check if that distance can join the "tree"
            if (dist[v] > dist[u] + weight) { 
                dist[v] = dist[u] + weight;
                // Set the current vertex to what the parent vertex is
                parent[v] = u;
                // Update the heap for next loop
                minHeap.push(make_pair(dist[v], v));
            }
        }
    }
    // The weight of the path from A->B will be in index 1 of the distance array
    int weight = dist[1];
    cout << "The weight of the shortest path is: " << weight << endl;
    // Go through the parent array for B slot to print the correct path from A->B
    printPath(parent, 1);
}

// Makes a graph with the specified file input 
void makeGraph(vector<pair<int, int>>G[], ifstream &file, char &src, char &dst, int &wt) {
    while (!file.eof()) {
        file >> src;
        file >> dst;
        file >> wt;

        // Subtract ASCII value from letters for Graph processing into array
        src = src - 65;
        dst = dst - 65;
        addEdge(G, src, dst, wt);
    }
    file.close();
}

int main() {
    string filename, dummyV;
    ifstream file;
    int weight;
    char source, destination;

    cout << "Please enter the filename you want to text (please include file extension): ";
    getline(cin, filename);

    file.open(filename);
    if (file.fail()) {
        cout << "Couldn't open text file, try again";
        exit(1);
    }

    // Read the first line in the text file so we can process remaining lines for Graph in makeGraph function
    getline(file, dummyV);

    if (filename._Equal("Case1.txt")) {
        const int V = 10;
        vector<pair<int, int>> G[V];
        makeGraph(G, file, source, destination, weight);
        findShortestPath(G, V, 0);
    }

    else if (filename._Equal("Case2.txt")) {
       const int V = 15;
       vector<pair<int, int>> G[V];
       makeGraph(G, file, source, destination, weight);
       findShortestPath(G, V, 0);
    }

    else if (filename._Equal("Case3.txt")) {
        const int V = 20;
        vector<pair<int, int>> G[V];
        makeGraph(G, file, source, destination, weight);
        findShortestPath(G, V, 0);
    }
    return 0;
}



