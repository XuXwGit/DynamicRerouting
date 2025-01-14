#ifndef ROUTE_MANAGER_H
#define ROUTE_MANAGER_H

#include "Graph.h"
#include "RoutingAlgorithms.h"

#include <iostream>
#include <algorithm>

class RouteManager
{
private:
    Graph graph;
    vector<Service> services;
    vector<Route> currentRoutes;
    vector<RoutingAlgorithm*> algorithms;
    vector<ReroutingAlgorithmType> enabledAlgorithms;
public:
    RouteManager(int n, int m, int w, int k, vector<ReroutingAlgorithmType> enabledAlgorithms);
    ~RouteManager();

    void init(vector<Edge> initEdges, vector<Service> initServices);
    void clearInterruptedRoutes(int deletedEdgeId);
    std::pair<int, vector<Route>> handleRequest(int deletedEdgeId);

    bool checkRoutesValid(const vector<Route> &routes) const;
    bool checkReRoutingConstrs(const unordered_map<int, Service> &interruptedServices, const vector<Route> &currentRoutes) const;
};

#endif // ROUTE_MANAGER_H
