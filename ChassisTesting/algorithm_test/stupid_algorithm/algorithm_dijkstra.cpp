#include <bits/stdc++.h>
using namespace std;
#include <iostream>
// A C++ program for Dijkstra's
// single source shortest path
// algorithm. The program is for
// adjacency matrix representation
// of the graph.

int NO_PARENT = -1;


// Function to print shortest path
// from source to currentVertex
// using parents array
void printPath(int currentVertex, vector<int> parents)
{

// Base case : Source node has
// been processed
if (currentVertex == NO_PARENT) {
    return;
}
printPath(parents[currentVertex], parents);

cout << currentVertex +1 << " ";
}

// A utility function to print
// the constructed distances
// array and shortest paths
void printSolution(int startVertex, vector<int> distances,
vector<int> parents, int end)
{
int nVertices = distances.size();
cout << "Vertex\t Distance\tPath";

for (int vertexIndex = 0; vertexIndex < nVertices;
     vertexIndex++) {
         if (vertexIndex == end){

                cout << "\n" << startVertex << " -> ";
                cout << vertexIndex+1 << " \t\t ";
                cout << distances[vertexIndex] << "\t\t";
                printPath(vertexIndex, parents);                

         }        
}
}

// Function that implements Dijkstra's
// single source shortest path
// algorithm for a graph represented
// using adjacency matrix
// representation

void dijkstra(vector<vector<int> > adjacencyMatrix,
int startVertex, int end)
{
end -= 1 ;
startVertex -= 1;
int nVertices = adjacencyMatrix[0].size();

// shortestDistances[i] will hold the
// shortest distance from src to i
vector<int> shortestDistances(nVertices);

// added[i] will true if vertex i is
// included / in shortest path tree
// or shortest distance from src to
// i is finalized

vector<bool> added(nVertices);

// Initialize all distances as
// INFINITE and added[] as false
for (int vertexIndex = 0; vertexIndex < nVertices;
     vertexIndex++) {
    shortestDistances[vertexIndex] = INT_MAX;
    added[vertexIndex] = false;
}

// Distance of source vertex from
// itself is always 0
shortestDistances[startVertex] = 0;

// Parent array to store shortest
// path tree
vector<int> parents(nVertices);

// The starting vertex does not
// have a parent
parents[startVertex] = NO_PARENT;

// Find shortest path for all
// vertices
for (int i = 1; i < nVertices; i++) {

    // Pick the minimum distance vertex
    // from the set of vertices not yet
    // processed. nearestVertex is
    // always equal to startNode in
    // first iteration.
    int nearestVertex = -1;
    int shortestDistance = INT_MAX;
    for (int vertexIndex = 0; vertexIndex < nVertices;
         vertexIndex++) {
        if (!added[vertexIndex]
            && shortestDistances[vertexIndex]
                   < shortestDistance) {
            nearestVertex = vertexIndex;
            shortestDistance
                = shortestDistances[vertexIndex];
        }
    }

    // Mark the picked vertex as
    // processed
    added[nearestVertex] = true;

    // Update dist value of the
    // adjacent vertices of the
    // picked vertex.
    for (int vertexIndex = 0; vertexIndex < nVertices;
         vertexIndex++) {
        int edgeDistance
            = adjacencyMatrix[nearestVertex]
                             [vertexIndex];

        if (edgeDistance > 0
            && ((shortestDistance + edgeDistance)
                < shortestDistances[vertexIndex])) {
            parents[vertexIndex] = nearestVertex;
            shortestDistances[vertexIndex]
                = shortestDistance + edgeDistance;
        }
    }
}

printSolution(startVertex+1, shortestDistances, parents, end);
}


// Driver Code
int main()
{
vector<vector<int> > adjacencyMatrix
= {
                {  0, 66, 39,  0, 86,  0,  0,  0,  0,  0,   0,   0,  0,   0 }, //1
                { 66,  0, 46,  0,  0, 30,  0,  0,  0,  0,   0,   0,  0,   0 },
                { 39, 46,  0, 23,  0,  0,  0,  0,  0,  0,   0,   0,  0,   0 }, //3
                {  0,  0, 23,  0, 24,  0, 38,  0,  0,  0,   0,   0,  0,   0 },
                { 86,  0,  0, 24,  0,  0,  0,  0,  0,  0,   0,   0, 73,   0 }, //5
                {  0, 30,  0,  0,  0,  0,  0,  0,  0,  0,  35,  70,  0,   0 },
                {  0,  0,  0, 38,  0,  0,  0, 23, 14,  0,   0,   0,  0,   0 }, //7
                {  0,  0,  0,  0,  0,  0, 23,  0,  0,  0,   0,   0,  0,   0 },
                {  0,  0,  0,  0,  0,  0, 14,  0,  0, 23,   0,   0,  0,  52 },  //9 
                {  0,  0,  0,  0,  0,  0,  0,  0, 23,  0,   0,   0,  0,   0 },   //10
                {  0,  0,  0,  0,  0,  35,  0,  0,  0, 0,   0,   35, 0, 100 },
                {  0,  0,  0,  0,  0, 70,  0,  0,  0,  0,   35,  0,  102,  0},
                {  0,  0,  0,  0, 73,  0,  0,  0,  0,  0,   0, 102,  0,  94 },    //13
                {  0,  0,  0,  0,  0,  0,  0,  0, 52,  0, 100,   0, 94,   0 }};

dijkstra(adjacencyMatrix, 12, 1); // 10 to 3 

return 0;

}