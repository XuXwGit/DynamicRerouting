#include "RoutingAlgorithms.h"
#include <queue>
#include <set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <random>

// Represents an item in the priority queue for A* algorithm.
struct PriorityQueueItem
{
    int node; // The current node.
    int dist; // The distance from the start node.
    int freq; // The frequency used.
    // Overload the less-than operator for priority queue ordering.
    bool operator<(const PriorityQueueItem &other) const
    {
        return dist > other.dist; // Min-heap based on distance.
    }
};

// Default routing algorithm implementation.
vector<int> DefaultRouting::findPath(const Service &service, int freq)
{
    int start = service.s; // Start node of the service.
    int end = service.t;   // End node of the service.

    int lastInPathV = start; // Last node in the current path.
    vector<int> edgesPath;   // Path edges.
    set<int> nodesInPath;    // Nodes in the current path to avoid cycles.
    nodesInPath.insert(start);

    // Iterate over all edges in the graph.
    for (Edge edge : localGraph->edges)
    {
        // Skip edges not available for the service at the given frequency.
        if (!localGraph->checkEdgeFreqServiceAvailable(edge.id, freq, service.id))
        {
            continue;
        }

        // Try to extend the path.
        if (edge.v == lastInPathV || edge.u == lastInPathV)
        {
            int other = edge.v == lastInPathV ? edge.u : edge.v;
            if (nodesInPath.count(other))
                continue; // Avoid cycles.
            lastInPathV = other;
            nodesInPath.insert(other);
            edgesPath.push_back(edge.id);
        }

        // If the path reaches the end node.
        if (lastInPathV == end)
        {
            return edgesPath; // Return the complete path.
        }
    }

    // Return an empty path if no route is found.
    return {};
}

// Dijkstra routing algorithm implementation.
vector<int> DijkstraRouting::findPath(const Service &service, int freq)
{
    // Use the graph's Dijkstra method to find the shortest path.
    auto [nodePath, edgePath] = localGraph->dijkstra(service, freq);
    return edgePath; // Return the path edges.
}

// A* routing algorithm implementation.
vector<int> AStarRouting::findPath(const Service &service, int freq)
{
    int start = service.s; // Start node.
    int end = service.t;   // End node.

    priority_queue<PriorityQueueItem> pq;         // Priority queue for A*.
    vector<int> dist(localGraph->N + 1, INT_MAX); // Distance from start.
    vector<int> prev(localGraph->N + 1, -1);      // Previous node in the path.
    vector<int> usedEdge(localGraph->N + 1, -1);  // Edge used to reach the node.

    dist[start] = 0;           // Distance to start node is 0.
    pq.push({start, 0, freq}); // Push start node to the queue.

    while (!pq.empty())
    {
        PriorityQueueItem curr = pq.top();
        pq.pop();

        if (curr.node == end)
            break; // Path found.

        // Explore neighbors.
        for (const auto &[neighbor, edge_id] : localGraph->adj.at(curr.node))
        {
            // Skip unavailable edges.
            if (!localGraph->checkEdgeFreqServiceAvailable(edge_id, freq, service.id))
            {
                continue;
            }

            int weight = localGraph->edgeUsedFreqs[edge_id]; // Edge weight.
            int heuristic = 0;                               // Simple heuristic (set to 0).
            // Update distance and predecessors.
            if (dist[neighbor] > dist[curr.node] + weight + heuristic)
            {
                dist[neighbor] = dist[curr.node] + weight;
                prev[neighbor] = curr.node;
                usedEdge[neighbor] = edge_id;
                pq.push({neighbor, dist[neighbor] + heuristic, freq});
            }
        }
    }

    // Reconstruct the path.
    vector<int> path;
    for (int v = end; v != -1; v = prev[v])
    {
        if (usedEdge[v] != -1)
            path.push_back(usedEdge[v]);
    }
    reverse(path.begin(), path.end());
    return path;
}

// Single base routing algorithm implementation.
vector<Route> SingleBaseRouting::getNewRoutes(
    const unordered_map<int, Service> &interruptedServices, vector<Route> &currentRoutes)
{
    // Attempt to reroute each interrupted service.
    for (auto &[service_id, service] : interruptedServices)
    {
        for (int freq = 1; freq <= localGraph->W; ++freq)
        {
            vector<int> newPath = this->findPath(service, freq);
            if (!newPath.empty())
            {
                currentRoutes[service_id - 1].p = newPath;
                currentRoutes[service_id - 1].w = freq;
                for (int edgeId : currentRoutes[service_id - 1].p)
                {
                    localGraph->curReservedFrequenciesMask[edgeId] |= (1LL << freq);
                }
                break; // Found a new path, stop frequency iteration.
            }
        }
    }

    return currentRoutes;
}

// Dijkstra routing algorithm's getNewRoutes implementation.
vector<Route> DijkstraRouting::getNewRoutes(
    const unordered_map<int, Service> &interruptedServices, vector<Route> &currentRoutes)
{
    // Task queue for services with remaining frequency attempts.
    std::queue<std::pair<int, vector<int>>> taskQueue;
    std::vector<int> frequencies;
    for (int i = 1; i <= localGraph->W; ++i)
    {
        frequencies.push_back(i);
    }
    // Initialize the task queue.
    for (const auto &[service_id, service] : interruptedServices)
    {
        taskQueue.push({service_id, frequencies});
    }

    // Process the task queue.
    while (!taskQueue.empty())
    {
        auto [service_id, attempts] = taskQueue.front();
        taskQueue.pop();

        const Service &service = interruptedServices.at(service_id);

        int countTry = 0;
        for (auto iter = attempts.begin(); iter != attempts.end(); countTry++)
        {
            int freq = *iter;
            vector<int> newPath = this->findPath(service, freq);
            if (!newPath.empty())
            {
                currentRoutes[service_id - 1].p = newPath;
                currentRoutes[service_id - 1].w = freq;
                for (int edgeId : currentRoutes[service_id - 1].p)
                {
                    localGraph->curReservedFrequenciesMask[edgeId] |= (1LL << freq);
                }
                break; // Found a new path, stop frequency iteration.
            }
            else
            {
                iter = attempts.erase(iter);
            }
        }
    }

    return currentRoutes;
}

// ILP-based routing algorithm implementation.
vector<Route> ILPBasedRouting::getNewRoutes(
    const unordered_map<int, Service> &interruptedServices, vector<Route> &currentRoutes)
{
    // Load interrupted services into the model.
    model->clearServices();
    for (const auto &[id, service] : interruptedServices)
    {
        model->addService(service);
    }

    // Solve the model.
    vector<Route> newRoutes = currentRoutes;
    if (!model->solve(newRoutes))
    {
        std::cerr << "ILP failed to find a solution." << endl;
        return currentRoutes; // Return the original routes if rerouting fails.
    }

    // Update the current routes with the new routes.
    for (const auto &[id, service] : interruptedServices)
    {
        currentRoutes[id - 1] = newRoutes[id - 1];

        // Update the frequency reservation in the graph.
        for (int edgeId : currentRoutes[id - 1].p)
        {
            localGraph->curReservedFrequenciesMask[edgeId] |= (1LL << currentRoutes[id - 1].w);
        }
    }

    return currentRoutes; // Return the updated routes.
}

// ILP-based routing algorithm's updateGraph implementation.
void ILPBasedRouting::updateGraph(const Graph &globalGraph)
{
    this->localGraph->updateFromGlobalGraph(globalGraph);
}

// ILP-based routing algorithm's findPath implementation.
vector<int> ILPBasedRouting::findPath(const Service &service, int freq)
{
    // ILP methods does not support individual service path finding; return an empty path.
    cerr << "ILPBasedRouting does not support findPath for individual services." << endl;
    return {};
}