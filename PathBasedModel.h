#ifndef PATH_BASED_MODEL_H
#define PATH_BASED_MODEL_H

#include "Model.h"
#include "Graph.h"

#include "gurobi_c++.h"

#include <unordered_map>
#include <vector>

class PathBasedModel : public Model
{
private:
    GRBEnv *env;
    GRBModel *model;

    std::unordered_map<int, std::vector<Route>> candidateRoutes; // 服务对应的路径集合 routes[d][r]

    std::unordered_map<int, std::vector<GRBVar>> z; // 路径&边 选择变量 z[d][r]

    void getCandidateRoutes(); // 生成所有可能的路径
    void optimize();           // 求解优化问题

protected:
    void buildDecisionVariables() override;               // 构建决策变量
    void buildConstraints() override;                     // 构建约束条件
    void buildObjective() override;                       // 构建目标函数
    void extractSolution(vector<Route> &routes) override; // 从解中提取重路由方案放回 routes

    void clear() override
    {
        model = new GRBModel(*env);
        z.clear();
    }

public:
    PathBasedModel(Graph &graph);
};

#endif // PATH_BASED_MODEL_H