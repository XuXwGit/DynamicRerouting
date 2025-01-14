#ifndef LAGRANGE_PATH_BASED_MODEL_H
#define LAGRANGE_PATH_BASED_MODEL_H

#include "Model.h"
#include "Graph.h"

#include <unordered_map>
#include <vector>
#include <set>
#include <queue>
#include <cmath>
#include <iostream>

class LagrangePathBasedModel : public Model
{
private:
    // 路径和对应的拉格朗日乘子
    std::unordered_map<int, std::vector<Route>> candidateRoutes; // 服务对应的路径集合 routes[d][r]

    std::unordered_map<int, Route> solutionRoute;                               // z[d][r]：服务 d 的路径 r 是否被选择
    std::unordered_map<int, double> lambda;                                     // 拉格朗日乘子 lambda[d]
    std::unordered_map<int, std::unordered_map<int, double>> edgeFreqDualPrice; // 每条边的波长惩罚 penalty[e][w]
    std::unordered_map<int, std::unordered_map<int, double>> usage;

    // 优化参数
    double stepSize = 0.1; // 学习率
    int maxIter = 100;     // 最大迭代次数
    double epsilon = 1e-3; // 收敛条件

    // 辅助方法
    void initializeLagrangeMultipliers();
    void generateCandidateRoutes();                // 为每个服务生成候选路径
    Route solveSubproblem(const Service &service); // 求解单个服务的路径分配子问题
    void updateLagrangeMultipliers();              // 更新拉格朗日乘子
    double calculateObjective();                   // 计算目标函数值

protected:
    bool optimize(std::vector<Route> &routes); // 求解并返回路由结果
    void extractSolution(std::vector<Route> &routes);
    void clear() override
    {
        solutionRoute.clear();
    }

public:
    LagrangePathBasedModel(Graph &graph) : Model(graph) {}
    void addService(const Service &service); // 添加服务
};


#endif // LAGRANGE_PATH_BASED_MODEL_H
