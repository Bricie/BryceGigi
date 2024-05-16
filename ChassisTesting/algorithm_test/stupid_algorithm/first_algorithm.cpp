#include <iostream>
#define MAX_PATH_SIZE 100
using namespace std;

int path[MAX_PATH_SIZE];
int pathSize = 0;
const int V = 14;

int minDistance(int dist[], bool sptSet[])
{
    int min = INT_MAX, min_index;

    for (int v = 0; v < V; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}

void printPath(int parent[], int j)
{
    if (parent[j] == -1)
    {
        path[pathSize++] = j;
        return;
    }

    printPath(parent, parent[j]);
    path[pathSize++] = j;
}

void dijkstra(int graph[][V], int start, int end)
{
    int dist[V];
    bool sptSet[V];
    int parent[V];

    for (int i = 0; i < V; i++)
    {
        parent[start] = -1;
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }

    dist[start] = 0;

    for (int count = 0; count < V - 1; count++)
    {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

        for (int v = 0; v < V; v++)
            if (!sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v])
            {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            }
    }

    pathSize = 0;

    printPath(parent, end);

    cout << "Shortest path from ";
    cout << (start);
    cout << ("to");
    cout << (end);
    cout << (":   \n");

    for (int i = 0; i < pathSize; i++)
    {
        cout << (path[i]);
        cout << (",  ");
    }
}

int graph[V][V] = {
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
                {  0,  0,  0,  0,  0,  0,  0,  0,  0, 52,   0,   35, 0, 100 },
                {  0,  0,  0,  0,  0, 70,  0,  0,  0,  0,   35,  0,  102,  0},
                {  0,  0,  0,  0, 73,  0,  0,  0,  0,  0,   0, 102,  0,  94 },    //13
                {  0,  0,  0,  0,  0,  0,  0,  0, 52,  0, 100,   0, 94,   0 }};
    int main (){
        cout << ("DijkstraTask");
        dijkstra(graph, 10, 14);
    }

