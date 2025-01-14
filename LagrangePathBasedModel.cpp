#include "LagrangePathBasedModel.h"

// 初始化拉格朗日乘子
void LagrangePathBasedModel::initializeLagrangeMultipliers()
{
    lambda.clear();
    for (auto [id, service] : interruptedServices)
    {
        lambda[id] = 1.0; // 初始化拉格朗日乘子
    }

    edgeFreqDualPrice.clear();
    for (int edgeId = 1; edgeId <= graph.M; ++edgeId)
    {
        for (int freq = 1; freq <= graph.W; ++freq)
        {
            if (graph.checkEdgeFreqAvailable(edgeId, freq))
            {
                edgeFreqDualPrice[edgeId][freq] = 0; // 初始化为 0
            }
            else
            {
                edgeFreqDualPrice[edgeId][freq] = 1000000; // 初始化为 大数
            }
        }
    }
}

void LagrangePathBasedModel::generateCandidateRoutes()
{
    // 为每个服务生成可能的路径
    for (const auto &[id, service] : interruptedServices)
    {
        try
        {
            std::vector<Route> serviceRoutes = graph.getKCandidateReRoutings(service, 1000);
            candidateRoutes[id] = serviceRoutes;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in get K Candidate ReRoutings of " << service.id << std::endl;
            std::cerr << e.what() << '\n';
        }
    }
}

Route LagrangePathBasedModel::solveSubproblem(const Service &service)
{
    Route bestRoutes;
    double bestCost = std::numeric_limits<double>::max();

    for (size_t i = 0; i < candidateRoutes[service.id].size(); ++i)
    {
        const auto &route = candidateRoutes[service.id][i];
        double cost = 0.0;

        // 计算路径成本，包括拉格朗日项
        for (int edgeId : route.p)
        {
            cost += edgeFreqDualPrice[edgeId][route.w];
        }

        // 更新最优解
        if (cost < bestCost)
        {
            bestCost = cost;
            bestRoutes = route;
        }
    }

    if (bestCost <= 1)
    {
        solutionRoute[service.id] = bestRoutes;
        return bestRoutes;
    }
    else
    {
        return {};
    }
}

// 更新拉格朗日乘子
void LagrangePathBasedModel::updateLagrangeMultipliers()
{
    usage.clear();

    for (const auto &[id, route] : solutionRoute)
    {
        for (int edgeId : route.p)
        {
            usage[edgeId][route.w] += 1.0;
        }
    }

    for (int edgeId = 1; edgeId <= graph.M; ++edgeId)
    {
        for (int freq = 1; freq <= graph.W; ++freq)
        {
            if (graph.checkEdgeFreqAvailable(edgeId, freq))
            {
                edgeFreqDualPrice[edgeId][freq] = edgeFreqDualPrice[edgeId][freq] + stepSize * (usage[edgeId][freq] - 1.0);
                edgeFreqDualPrice[edgeId][freq] = std::max(0.0, edgeFreqDualPrice[edgeId][freq]);
            }
            else
            {
                edgeFreqDualPrice[edgeId][freq] = 1000000; // 初始化为 大数
            }
        }
    }
}

double LagrangePathBasedModel::calculateObjective()
{
    double objective = solutionRoute.size();

    for (int edgeId = 0; edgeId <= graph.M; ++edgeId)
    {
        for (int freq = 1; freq <= graph.W; ++freq)
        {
            objective += edgeFreqDualPrice[edgeId][freq] * (usage[edgeId][freq] - 1);
        }
    }
    return objective;
}

void LagrangePathBasedModel::addService(const Service &service)
{
    interruptedServices[service.id] = service;
    lambda[service.id] = 1.0; // 初始化拉格朗日乘子
}

bool LagrangePathBasedModel::optimize(std::vector<Route> &routes)
{
    // 初始化候选路径
    generateCandidateRoutes();

    // 初始化边波长惩罚
    initializeLagrangeMultipliers();

    // 主迭代过程
    for (int iter = 0; iter < maxIter; ++iter)
    {
        // 求解子问题
        for (const auto &[id, service] : interruptedServices)
        {
            solveSubproblem(service);
        }

        // 更新拉格朗日乘子
        updateLagrangeMultipliers();

        // 计算目标函数值
        double objective = calculateObjective();
        // std::cout << "Iteration: " << iter << " Objective: " << objective << std::endl;
    }

    return false;
}

void LagrangePathBasedModel::extractSolution(std::vector<Route> &routes)
{
    optimize(routes);

    usage.clear();
    for (const auto &[id, route] : solutionRoute)
    {
        bool valid = true;
        for (int edgeId : route.p)
        {
            if (++usage[edgeId][route.w] > 1)
            {
                valid = false;
            }
        }
        if (valid)
        {
            routes[id - 1] = (route);
        }
        else
        {
            for (int edgeId : route.p)
            {
                usage[edgeId][route.w]--;
            }
        }
    }
}