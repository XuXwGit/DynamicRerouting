#include "Graph.h"

Graph::Graph(int n, int m, int w) : N(n), M(m), W(w)
{
    curReservedFrequenciesMask.assign(M + 1, 0LL);
    initReservedFrequenciesMask.assign(M + 1, 0LL);
}

Graph::Graph(int n, int m, int w, vector<Edge> initEdges) : N(n), M(m), W(w), edges(initEdges)
{
    edgeUsedFreqs.assign(M + 1, 0);
    curReservedFrequenciesMask.assign(M + 1, 0LL);
    initReservedFrequenciesMask.assign(M + 1, 0LL);
    for (const auto &edge : initEdges)
    {
        adj[edge.u][edge.v] = edge.id;
        adj[edge.v][edge.u] = edge.id;
    }
}

void Graph::addEdge(const Edge &edge)
{
    edges.push_back(edge);
    adj[edge.u][edge.v] = edge.id;
    adj[edge.v][edge.u] = edge.id;
}

void Graph::reset()
{
    edges.clear();
    edgeUsedFreqs.assign(M + 1, 0);
    curReservedFrequenciesMask.assign(M + 1, 0LL);
    deletedEdges.clear();
    memset(belongingService, 0, sizeof belongingService);
}

void Graph::generateCandidatePaths(const Service &service)
{
    // 生成候选路径
    auto kShortestPaths = findKShortestPaths(service.s, service.t, 10);

    for (const auto &path : kShortestPaths)
    {
        candidateServicePaths[service.id].insert(path);
    }
}

void Graph::generateCandidateRoutes(const Service &service)
{
    if (candidateServicePaths[service.id].empty())
    {
        generateCandidatePaths(service);
    }

    // 基于 path + 波长 组合生成 候选路由
    for(const auto &path: candidateServicePaths[service.id]){
        if (checkPathConnected(path) == false)
        {
            continue;
        }
        
        for (int w = 1; w <= W; ++w)
        {
            Route route = {service.id, w, path};
            if (checkRouteValid(route))
            {
                candidateServiceRoutes[service.id].insert(route);
            }
        }
    }
}


bool Graph::checkEdgeFreqAvailable(int edgeId, int freq) const
{
    // 边已被删除
    if (deletedEdges.count(edgeId))
    {
        return false;
    }

    // 当前边当前波长已被使用
    if (curReservedFrequenciesMask[edgeId] & (1LL << freq))
    {
        return false;
    }

    return true;
}


bool Graph::checkEdgeFreqServiceAvailable(int edgeId, int freq, int serviceId) const
{
    // 边已被删除
    if (deletedEdges.count(edgeId))
    {
        return false;
    }

    // 当前边当前波长已被使用
    if (curReservedFrequenciesMask[edgeId] & (1LL << freq))
    {
        return false;
    }

    // 对于服务 d ∈ D 的初始路径 p∗ d 和波长 wd∗，任何其他服务 d′ ̸= d 的当前路径不能使用初始路径 p∗ d 的资源
    // 1. 初始时被使用
    // 2. 初始归属服务不是当前服务
    if ((initReservedFrequenciesMask[edgeId] >> freq & 1ll) && belongingService[edgeId][freq] != serviceId)
    {
        return false;
    }

    return true;
}

bool Graph::checkPathConnected(const vector<int> &path) const{
    for(int edgeID : path){
        if(deletedEdges.count(edgeID)){
            return false;
        }
    }

    return true;
}

bool Graph::checkRouteValid(const Route &route) const
{
    for (int edgeId : route.p)
    {
        if (!checkEdgeFreqServiceAvailable(edgeId, route.w, route.service_id))
        {
            return false;
        }
    }
    return true;
}

std::pair<std::vector<int>, std::vector<int>> Graph::dijkstra(const Service& service, int freq) const
{
    int s = service.s;
    int t = service.t;

    vector<int> dist(this->N + 1, numeric_limits<int>::max());
    unordered_map<int, int> preNode;
    unordered_map<int, int> preEdge;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

    dist[s] = 0;
    pq.push({0, s});

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();

        if (u == t)
            break;

        for (const auto &[neighbor, edgeID] : adj.at(u))
        {
            if (! checkEdgeFreqServiceAvailable(edgeID, freq, service.id))
            {
                continue;
            }

            if (dist[u] + 1 < dist[neighbor])
            {
                dist[neighbor] = dist[u] + 1;
                preNode[neighbor] = u;
                preEdge[neighbor] = edgeID;
                pq.push({dist[neighbor], neighbor});
            }
        }
    }

    vector<int> nodePath;
    vector<int> edgePath;
    for (int at = t; at != 0; at = preNode[at])
    {
        nodePath.push_back(at);
        if (at != s && preEdge[at] != 0)
        {
            edgePath.push_back(preEdge[at]);
        }
    }
    reverse(nodePath.begin(), nodePath.end());
    reverse(edgePath.begin(), edgePath.end());
    return {nodePath, edgePath};
}

std::pair<std::vector<int>, std::vector<int>> Graph::dijkstra(int s, int t, const unordered_set<int>& invalidEdges) const
{
    vector<int> dist(this->N + 1, numeric_limits<int>::max());
    unordered_map<int, int> preNode;
    unordered_map<int, int> preEdge;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

    dist[s] = 0;
    pq.push({0, s});

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();

        if (u == t)
            break;

        for (const auto &[neighbor, edgeID] : adj.at(u))
        {
            if (invalidEdges.count(edgeID) || this->deletedEdges.count(edgeID))
            {
                continue;
            }

            if (dist[u] + 1 < dist[neighbor])
            {
                dist[neighbor] = dist[u] + 1;
                preNode[neighbor] = u;
                preEdge[neighbor] = edgeID;
                pq.push({dist[neighbor], neighbor});
            }
        }
    }

    vector<int> nodePath;
    vector<int> edgePath;
    for (int at = t; at != 0; at = preNode[at])
    {
        nodePath.push_back(at);
        if (at != s && preEdge[at] != 0)
        {
            edgePath.push_back(preEdge[at]);
        }
    }
    reverse(nodePath.begin(), nodePath.end());
    reverse(edgePath.begin(), edgePath.end());
    return {nodePath, edgePath};
}


/**
 * @Description: BFS 求k最短路径（not need in the submit version）
 *         Step 1. 用优先队列存储路径，每次取出最短路径
 *         Step 2. 从优先队列中取出的路径，如果到达终点，将其加入结果集
 *         Step 3. 否则：遍历当前节点的邻接边，构造新路径，加入优先队列
 *         Step 4. 重复 2-3，直到取出 k 条最短路径
 * @param {int} s 起点
 * @param {int} t 终点
 * @param {int} k 最短路径数量
 * @return {std::vector<std::vector<int>>} k条最短路径
 */
std::vector<std::vector<int>> Graph::bfsKShortestPaths(int s, int t, int k)
{
    std::vector<std::vector<int>> kShortestPaths;                                   // 最终结果
    using PathInfo = std::pair<int, std::pair<std::vector<int>, std::vector<int>>>; // {路径代价, {节点路径, 边集合}}
    auto cmp = [](const PathInfo &a, const PathInfo &b)
    { return a.first > b.first; };                                               // 小顶堆
    std::priority_queue<PathInfo, std::vector<PathInfo>, decltype(cmp)> pq(cmp); // 优先队列

    pq.push({0, {{s}, {}}}); // 初始路径，从起点开始

    while (!pq.empty() && kShortestPaths.size() < k)
    {
        int cost;
        std::vector<int> nodes, path;
        std::tie(cost, std::tie(nodes, path)) = pq.top();
        pq.pop();
        int currentNode = nodes.back();

        // 如果到达目标节点 t，将路径加入结果集
        if (currentNode == t)
        {
            kShortestPaths.push_back(path);
            continue;
        }

        // 用一个集合存储路径中出现过的边，避免环路
        std::unordered_set<int> visitedEdges(path.begin(), path.end());

        // 遍历当前节点的邻接边
        for (const auto &[neighbor, edgeId] : adj[currentNode])
        {
            if (visitedEdges.find(edgeId) == visitedEdges.end())
            {                                      // 检查是否有环路
                std::vector<int> newNodes = nodes; // 延迟构造新路径
                std::vector<int> newPath = path;   // 延迟构造新路径
                newNodes.push_back(neighbor);
                newPath.push_back(edgeId);
                pq.push({cost + 1, {newNodes, newPath}}); // 假设权重为 1
            }
        }
    }

    return kShortestPaths;
}

/**
 * @Description: Yen's KSP 算法
 *      Step 1: 用 Dijkstra 算法求出第一条最短路径
 *      Step 2: 选择前面最短路径每个节点作为偏离节点，生成偏离路径，加入候选路径集合
 *      Step 3: 从候选路由中选择最短路径，加入结果集
 *      Step 4: 重复 2-3，直到取出 k 条最短路径
 * @param {int} s 起点
 * @param {int} t 终点
 * @param {int} k 最短路径数量
 */
vector<vector<int>> Graph::YenKShortestPaths(int s, int t, int k)
{
    vector<vector<int>> A;                                   // 已找到的最短路径集
    using PathInfo = pair<int, std::pair<vector<int>, vector<int>>>;                 // {路径代价, 路径}
    priority_queue<PathInfo, vector<PathInfo>, greater<>> B; // 候选路径优先队列

    // 自定义哈希函数
    auto vectorHash = [](const vector<int> &v)
    {
        size_t hash = 0;
        for (int num : v)
        {
            hash ^= hash * 100007 + num; // 简单哈希函数
        }
        return hash;
    };

    unordered_set<vector<int>, decltype(vectorHash)> pathSet(0, vectorHash); // 存储已找到的路径

    // 第一步：使用 Dijkstra 算法找到第一条最短路径
    auto [nodePath, edgePath] = dijkstra(s, t, {});
    if (edgePath.empty())
    {
        return A; // 如果找不到路径，直接返回空结果
    }
    A.push_back(edgePath);
    pathSet.insert(edgePath);

    // 第二步：生成候选路径
    while (A.size() < k)
    {
        unordered_set<int> invalidEdges;
        for (size_t i = 0; i < edgePath.size(); ++i)
        {
            int spurNode = nodePath[i];

            // 修改图，只移除 spurNode 到根路径其他节点的边
            invalidEdges.insert(edgePath[i]);

            // 寻找偏离路径
            auto [spurNodePath, spurEdgePath] = dijkstra(spurNode, t, invalidEdges);
            if (spurEdgePath.empty())
                continue; // 无法找到偏离路径

            // 合并根路径和偏离路径
            vector<int> rootNodePath(nodePath.begin(), nodePath.begin() + i);
            rootNodePath.insert(rootNodePath.end(), spurNodePath.begin(), spurNodePath.end());
            vector<int> rootEdgePath(edgePath.begin(), edgePath.begin() + i);
            rootEdgePath.insert(rootEdgePath.end(), spurEdgePath.begin(), spurEdgePath.end());

            // 检查路径是否已存在
            if (pathSet.find(rootEdgePath) == pathSet.end())
            {
                B.push({static_cast<int>(rootEdgePath.size()), {rootNodePath, rootEdgePath}});
                pathSet.insert(rootEdgePath);
            }
        }

        // 如果找不到更多候选路径，退出循环
        if (B.empty())
            break;

        // 从候选路径集中取出代价最小的一条路径
        if (!B.empty())
        {
            auto [nodePath, edgePath] = B.top().second;
            A.push_back(edgePath);
            B.pop();
        }
    }

    return A;
}

std::vector<std::vector<int>> Graph::findKShortestPaths(int s, int t, int k)
{
    return YenKShortestPaths(s, t, k);
}

std::vector<Route> Graph::getCandidateReRoutings(const Service &service)
{
    std::vector<Route> serviceCandidateRoutes;
    auto &serviceRoutes = candidateServiceRoutes[service.id];

    if (serviceRoutes.empty() || serviceRoutes.size() < 10)
    {
        generateCandidateRoutes(service);
    }

    for (auto it = serviceRoutes.begin(); it != serviceRoutes.end();)
    {
        if (checkRouteValid(*it))
        {
            serviceCandidateRoutes.push_back(*it);
            ++it;
        }
        else
        {
            it = serviceRoutes.erase(it); // 清理无效路由
        }
    }

    return serviceCandidateRoutes;
}

std::vector<Route> Graph::getKCandidateReRoutings(const Service &service, int k)
{
    std::vector<Route> serviceCandidateRoutes;
    auto &serviceRoutes = candidateServiceRoutes[service.id];

    if(serviceRoutes.empty() || serviceRoutes.size() < k){
        generateCandidateRoutes(service);
    }

    // 提前筛除无效路由
    for (auto it = serviceRoutes.begin(); it != serviceRoutes.end() && serviceCandidateRoutes.size() < k;)
    {
        if (checkRouteValid(*it))
        {
            serviceCandidateRoutes.push_back(*it);
            ++it;
        }
        else
        {
            it = serviceRoutes.erase(it); // 清理无效路由
        }
    }

    // 返回前 k 条有效路由
    return serviceCandidateRoutes;
}

void Graph::updateFromGlobalGraph(const Graph &globalGraph)
{
    // 更新删除的边
    for (const int edgeId : globalGraph.deletedEdges)
    {
        this->deletedEdges.insert(edgeId);
    }

    // 更新当前波长占用状态
    this->curReservedFrequenciesMask = globalGraph.curReservedFrequenciesMask;
}
