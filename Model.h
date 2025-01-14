#ifndef MODEL_H
#define MODEL_H

#include "Graph.h"
#include <vector>
#include <set>
#include <unordered_map>

class Model
{
protected:
    Graph &graph;                   // 引用图对象
    unordered_map<int, Service> interruptedServices;       // 故障服务集合

    // 从graph中获取的成员
    // vector<long long> curFreqMask;  // 当前波长占用状态
    // vector<long long> initFreqMask; // 初始波长占用状态
    // set<int> deletedEdges;          // 已删除的边集合
    // 内部辅助方法
    virtual void buildDecisionVariables(); // 构建决策变量
    virtual void buildConstraints();            // 构建约束条件
    virtual void buildObjective();               // 构建目标函数
    virtual void clear() = 0;                        // 清空模型
    virtual void extractSolution(vector<Route> &routes) = 0; // 从解中提取重路由方案放回 routes

public:
    Model(Graph &graph);

    void addService(const Service &service); // 添加服务
    void addServices(const unordered_map<int, Service> &services);
    void clearServices();
    void removeEdge(int edgeId);               // 删除边并更新约束
    bool solve(vector<Route> &routes); // 求解并返回路由结果
};

#endif // MODEL_H
