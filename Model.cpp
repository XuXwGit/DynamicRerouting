#include "Model.h"
#include <iostream>
#include <algorithm>

Model::Model(Graph &g)
    : graph(g)
    //   curFreqMask(g.curReservedFrequenciesMask),
    //   initFreqMask(g.initReservedFrequenciesMask),
    //   deletedEdges(g.deletedEdges)
{
}

// 添加服务
void Model::addService(const Service &service)
{
    interruptedServices[service.id] = service;
}

// 批量加载服务
void Model::addServices(const unordered_map<int, Service> &services)
{
    for (const auto &[id, service] : services)
    {
        this->interruptedServices[id] = service;
    }
}

// 清空当前服务
void Model::clearServices()
{
    this->interruptedServices.clear();
}

// 删除边并更新约束 
void Model::removeEdge(int edgeId)
{
    graph.deletedEdges.insert(edgeId);
    // 其他逻辑（如标记受影响的服务）
}

// 构建决策变量
void Model::buildDecisionVariables()
{
    // TODO: 初始化优化问题的决策变量
    // 例如 x[d][e][w] 表示服务 d 是否使用边 e 和波长 w
}

// 构建约束条件
void Model::buildConstraints()
{
    // TODO: 添加路径流约束、波长冲突约束等
    // 示例：
    // 1. 路径流约束
    // 2. 波长冲突约束
    // 3. 边删除约束
}

// 构建目标函数
void Model::buildObjective()
{
    // TODO: 构建最大化成功路由服务数量的目标函数
}

// 求解优化问题
bool Model::solve(vector<Route> &routes)
{
    try
    {
        clear();

        buildDecisionVariables();
        buildConstraints();
        buildObjective();
        
        extractSolution(routes);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return true; // 暂时返回 true
}
