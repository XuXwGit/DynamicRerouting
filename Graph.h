#ifndef GRAPH_H
#define GRAPH_H

#include "Rerouting.h"

#include <vector>
#include <cstring>
#include <queue>
#include <set>
#include <string>
#include <sstream>
#include <unordered_set>
#include <unordered_map>

using namespace std;

static const int MAX_EDGE_QUANTITY = 40;
static const int MAX_FREQ_QUANTITY = 32;

// 自定义比较器
struct PathComparator
{
    bool operator()(const vector<int> &a, const vector<int> &b) const
    {
        return a.size() > b.size(); // 按路径长度升序排列
    }
};

struct RouteComparator
{
    bool operator()(const Route &a, const Route &b) const
    {
        if (a.p.size() != b.p.size())
            return a.p.size() > b.p.size();
        return a.w > b.w; // 路径长度优先，波长次优
    }
};

class Graph
{
public:
    int N, M;                           // 节点数和边数
    int W;                                      // 波长数
    vector<Edge> edges;                 // 边集合
    vector<int> edgeUsedFreqs;              // 每条边已使用的波长数量
    unordered_map<int, unordered_map<int, int>> adj;      // 邻接表 key : nodeID, value : {key :nodeID, value: edgeID}
    vector<long long> curReservedFrequenciesMask;         // 当前路径波长占用
    vector<long long> initReservedFrequenciesMask;     // 初始路径波长占用
    set<int> deletedEdges;              // 已删除的边
    int belongingService[MAX_EDGE_QUANTITY + 1][MAX_FREQ_QUANTITY + 1]; // 每条边的波长归属服务

    unordered_map<int, set<vector<int>, PathComparator>> candidateServicePaths;
    unordered_map<int, set<Route, RouteComparator>> candidateServiceRoutes;

    Graph(int n, int m, int w);
    Graph(int n, int m, int w, vector<Edge> initEdges);

    void addEdge(const Edge &edge);
    void reset();

    void generateCandidatePaths(const Service &service);
    void generateCandidateRoutes(const Service &service);

    bool checkEdgeFreqAvailable(int edgeId, int freq) const;
    bool checkEdgeFreqServiceAvailable(int edgeId, int freq, int serviceId) const;
    bool checkPathConnected(const vector<int> &path) const;
    bool checkRouteValid(const Route &route) const;

    std::pair<std::vector<int>, std::vector<int>> dijkstra(const Service &service, int freq) const;
    std::pair<std::vector<int>, std::vector<int>> dijkstra(int s, int t, const unordered_set<int> &invalidEdges) const;
    std::vector<std::vector<int>> YenKShortestPaths(int s, int t, int k);
    std::vector<std::vector<int>> bfsKShortestPaths(int s, int t, int k);
    std::vector<std::vector<int>> findKShortestPaths(int s, int t, int k);
    std::vector<Route> getCandidateReRoutings(const Service &service);
    std::vector<Route> getKCandidateReRoutings(const Service &service, int k);

    // 添加 clone 函数，返回当前图的深拷贝
    std::unique_ptr<Graph> clone() const
    {
        auto newGraph = std::make_unique<Graph>(N, M, W);
        newGraph->edges = edges;
        newGraph->edgeUsedFreqs = edgeUsedFreqs;
        newGraph->adj = adj;
        newGraph->curReservedFrequenciesMask = curReservedFrequenciesMask;
        newGraph->initReservedFrequenciesMask = initReservedFrequenciesMask;
        newGraph->deletedEdges = deletedEdges;
        memcpy(newGraph->belongingService, belongingService, sizeof(belongingService));
        return newGraph;
    }

    void updateFromGlobalGraph(const Graph &globalGraph);
};

#endif // GRAPH_H
