#include "RouteManager.h"

RouteManager::RouteManager(int n, int m, int w, int k, vector<ReroutingAlgorithmType> enabledAlgorithms)
    : graph(n, m, w), currentRoutes(k), enabledAlgorithms(enabledAlgorithms)
{
}

RouteManager::~RouteManager()
{
    for (auto algorithm : algorithms)
    {
        delete algorithm;
    }
}

void RouteManager::init(vector<Edge> initEdges, vector<Service> initServices)
{
    graph.reset(); // 重置图的状态
    services = initServices;

    // 添加所有边
    for (const auto &edge : initEdges)
    {
        graph.addEdge(edge);
    }

    // 初始化每个服务的初始路径和波长占用
    for (const auto &service : services)
    {
        currentRoutes[service.id - 1] = {service.id, service.w_star, service.p_star};
        for (int edgeId : service.p_star)
        {
            graph.initReservedFrequenciesMask[edgeId] |= (1LL << service.w_star);
            graph.curReservedFrequenciesMask[edgeId] |= (1LL << service.w_star);
            graph.belongingService[edgeId][service.w_star] = service.id;
            graph.edgeUsedFreqs[edgeId]++;
        }
    }

    // 初始化算法
    for (const auto &algoType : enabledAlgorithms)
    {
        switch (algoType)
        {
        case DEFAULT:
        {
            auto algo = new DefaultRouting(graph);
            algo->algoStr = "Default";
            algorithms.emplace_back(algo);
            break;
        }
        case DIJKSTRA:
        {
            auto algo = new DijkstraRouting(graph);
            algo->algoStr = "Dijkstra";
            algorithms.emplace_back(algo);
            break;
        }
        case A_STAR:
        {
            auto algo = new AStarRouting(graph);
            algo->algoStr = "A*";
            algorithms.emplace_back(algo);
            break;
        }
        case ARC_IP:
        {
            auto algo = new ArcIPModelRouting(graph);
            algo->algoStr = "Arc-IP";
            algorithms.emplace_back(algo);
            break;
        }
        case PATH_IP:
        {
            auto algo = new PathIPModelRouting(graph);
            algo->algoStr = "Path-IP";
            algorithms.emplace_back(algo);
            break;
        }
        case LAGRANGE_DECOMPOSE:
        {
            auto algo = new LagrangePathIPModelRouting(graph);
            algo->algoStr = "Lagrange-Decompose";
            algorithms.emplace_back(algo);
            break;
        }
        default:
        {
            std::cerr << "Unknown algorithm type" << std::endl;
            break;
        }
        }
    }
}

void RouteManager::clearInterruptedRoutes(int deletedEdgeId)
{
    for (Route &route : currentRoutes)
    {
        if (find(route.p.begin(), route.p.end(), deletedEdgeId) != route.p.end())
        {
            // 清理受影响的路径
            for (int edgeId : route.p)
            {
                graph.curReservedFrequenciesMask[edgeId] ^= (1LL << route.w);
                graph.edgeUsedFreqs[edgeId]--;
            }
            route.p.clear();
            route.w = 0;
        }
    }
}

std::pair<int, vector<Route>> RouteManager::handleRequest(int deletedEdgeId)
{
    if (deletedEdgeId == 0)
    {
        // 恢复初始状态
        init(graph.edges, services);
        return {0, currentRoutes};
    }

    graph.deletedEdges.insert(deletedEdgeId);
    clearInterruptedRoutes(deletedEdgeId);

    // 创建一个临时路由集合，后面如果检查通过再更新currentRoutes
    vector<Route> tempRoutes = this->currentRoutes;
    // 收集中断服务/路由（路由为空）
    unordered_map<int, Service> interruptedServices;
    unordered_map<int, Route *> interruptedRoutes; // 改为存储Route的指针
    for (Route &route : tempRoutes)
    {
        if (route.p.empty())
        {
            interruptedServices[route.service_id] = services[route.service_id - 1]; // 确保service_id从1开始
            interruptedRoutes[route.service_id] = &route;                           // 存储Route对象的指针
        }
    }

    int minInterrupted = static_cast<int>(services.size());
    vector<Route> bestRoutes;

    // 遍历启用的算法
    for (auto algorithm : algorithms)
    {
        algorithm->updateGraph(graph);

        // 在局部图上运行算法
        vector<Route> tempRoutes = currentRoutes;
        if(algorithm->algoStr == "Arc-IP"){
            // 用当前最优解初始化：热启动
            tempRoutes = algorithm->getNewRoutes(interruptedServices, bestRoutes);
        }else{
            tempRoutes = algorithm->getNewRoutes(interruptedServices, tempRoutes);
        }

        // 统计中断服务数量
        int interruptedCount = 0;
        for (const auto &route : tempRoutes)
        {
            if (route.p.empty())
            {
                ++interruptedCount;
            }
        }

        std::cout << algorithm->algoStr << ": " << " Interrupt " << deletedEdgeId << " No path: " << interruptedCount << "/" << services.size() << std::endl;

        // 选择中断最少的方案
        if (interruptedCount < minInterrupted)
        {
            minInterrupted = interruptedCount;
            bestRoutes = tempRoutes;
        }
    }

    // No need in submit version
    if (bestRoutes.empty())
    {
        std::cerr << "No routes found after rerouting" << std::endl;
        return {interruptedServices.size(), this->currentRoutes}; // 返回原始的currentRoutes
    }
    else if (!checkRoutesValid(bestRoutes))
    {
        std::cerr << "Invalid routes after rerouting" << std::endl;
        return {interruptedServices.size(), this->currentRoutes}; // 返回原始的currentRoutes
    }
    else
    {
        // 更新全局图状态
        for (const auto &route : bestRoutes)
        {
            if (!route.p.empty())
            {
                for (int edgeId : route.p)
                {
                    graph.curReservedFrequenciesMask[edgeId] |= (1LL << route.w);
                    graph.edgeUsedFreqs[edgeId]++;
                }
            }
        }
        this->currentRoutes = bestRoutes; // 更新currentRoutes
    }

    return {minInterrupted, this->currentRoutes};
}

bool RouteManager::checkRoutesValid(const vector<Route> &newRoutes) const
{
    // 1. 路径和波长不变
    for (const auto &route : newRoutes)
    {
        const Route &currentRoute = this->currentRoutes[route.service_id - 1];
        if (!currentRoute.p.empty() && (route.p != currentRoute.p || route.w != currentRoute.w))
        {
            std::cerr << "Route " << route.service_id << " is modified" << std::endl;
            return false; // 路径或波长被修改
        }
    }

    // 2. 不同服务的路径公共边使用不同波长
    vector<vector<int>> edgeFreqUsage(graph.M + 1, vector<int>(graph.W + 1, 0));
    for (const auto &route : newRoutes)
    {
        for (int edgeId : route.p)
        {
            if (++edgeFreqUsage[edgeId][route.w] > 1)
            {
                std::cerr << "Edge " << edgeId << " is used by multiple services with the same frequency " << route.w << std::endl;
                return false; // 公共边使用了相同的波长
            }
        }
    }

    // 3. 初始路径资源不被其他服务使用
    for (const auto &route : newRoutes)
    {
        for (int edgeId : route.p)
        {
            if ((graph.initReservedFrequenciesMask[edgeId] >> route.w & 1ll) && graph.belongingService[edgeId][route.w] != route.service_id)
            {
                std::cerr << "Initial path edge " << graph.belongingService[edgeId][route.w] << " (" << edgeId << ")" << " is used by other services " << route.service_id << std::endl;
                return false; // 初始路径资源被其他服务使用
            }
        }
    }

    return true;
}

bool RouteManager::checkReRoutingConstrs(const unordered_map<int, Service> &interruptedServices, const vector<Route> &currentRoutes) const
{
    // Step1: check flow constrs
    for (auto [d, service] : interruptedServices)
    {
        int s = service.s, t = service.t;
        const Route &route = currentRoutes[service.id - 1];

        for (int v = 1; v <= graph.N; ++v)
        {
            int flow = 0;
            for (int w = 1; w <= graph.W; ++w)
            {
                for (const auto &[neighbor, edgeId] : graph.adj.at(v))
                {
                    if (route.w == w && !route.p.empty() && find(route.p.begin(), route.p.end(), edgeId) != route.p.end())
                    {
                        flow += 1;
                    }
                }
            }

            if (v == s || v == t)
            {
                if (!route.p.empty())
                {
                    if (flow != 1)
                    {
                        std::cerr << "Flow constrs violated at source or destination node" << std::endl;
                        return false;
                    }
                }
            }
            else
            {
                if (!route.p.empty())
                {
                    if (flow != 2 && flow != 0)
                    {
                        std::cerr << "Flow constrs violated at intermediate node" << std::endl;
                        return false;
                    }
                }
            }
        }
    }

    // Step2: Check Edge Freq Constr
    for (int edgeId = 1; edgeId <= graph.M; ++edgeId)
    {
        for (int w = 1; w <= graph.W; ++w)
        {
            int usage = 0;
            for (auto [d, service] : interruptedServices)
            {
                if (!graph.checkEdgeFreqServiceAvailable(edgeId, w, service.id))
                {
                    continue;
                }
                const Route &route = currentRoutes[d - 1];
                if (route.w == w && route.p.size() != 0 && find(route.p.begin(), route.p.end(), edgeId) != route.p.end())
                {
                    usage += 1;
                }
            }

            if (graph.checkEdgeFreqAvailable(edgeId, w))
            {
                if (usage > 1)
                {
                    std::cerr << "Edge " << edgeId << " is used by multiple services with the same frequency " << w << std::endl;
                    return false;
                }
            }
            else
            {
                if (usage > 0)
                {
                    std::cerr << "Edge " << edgeId << " is used by multiple services with the same frequency " << w << std::endl;
                    return false;
                }
            }
        }
    }

    return true;
}