#include "rerouting.h"
#include <queue>
#include <vector>
#include <set>
#include <climits>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <iostream>
#include "RouteManager.h"

using namespace std;

RouteManager *routeManager = nullptr;

/**
 * Called once before edge removal requests for initialization
 *
 * @param n quantity of nodes
 * @param m quantity of edges
 * @param w quantity of frequency
 * @param k quantity of services
 * @param initEdges edges
 * @param initServices services
 */
void init(int n, int m, int w, int k, vector<Edge> initEdges, vector<Service> initServices)
{
    // 选择 Sample / Dijkstra / A* / Arc-IP 算法
    // DIJKSTRA /A / ARC_IP / PATH_IP / LAGRANGE_DECOMPOSE
    routeManager = new RouteManager(n, m, w, k, {DIJKSTRA, A_STAR, ARC_IP, PATH_IP, LAGRANGE_DECOMPOSE});
    routeManager->init(initEdges, initServices);
}

/**
 * Called for each edge removal request
 *
 * @param r edge id to remove
 * @return vector<Route> current routes for each service
 */
vector<Route> request(int deletedEdgeId)
{
    auto [x1, res] = routeManager->handleRequest(deletedEdgeId);
    return res;
}
