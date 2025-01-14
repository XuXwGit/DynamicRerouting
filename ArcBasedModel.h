#ifndef ARC_BASED_MODEL_H
#define ARC_BASED_MODEL_H

#include "Model.h"
#include "gurobi_c++.h"
#include <vector>

/**
 * There seems to be some problem: the model can't find the optimal solution????
 */

class ArcBasedModel : public Model
{
private:
    GRBEnv *env;
    GRBModel *model;

    // 决策变量
    unordered_map<int, vector<vector<GRBVar>>> x; // x[d][e][w]：服务 d 是否使用边 e 和波长 w
    unordered_map<int, vector<GRBVar>> delta;     // 辅助变量：delta[d] 表示服务 d 是否成功路由
    unordered_map<int, vector<GRBVar>> y;                             // 辅助变量：y[n] 表示节点 n 是否是中间节点

    void optimize(std::vector<Route> &routes); // 求解优化问题
    void addFlowConstr();
    void addEdgeFreqOnlyOneConstr();
    void addEdgeFreqAvailableConstr();
    void addOnlyOneFreqConstr();

protected:
    void buildDecisionVariables() override;      // 构建决策变量
    void buildConstraints() override;            // 构建约束条件
    void buildObjective() override;              // 构建目标函数
    void extractSolution(vector<Route> &routes) override; // 从解中提取重路由方案放回 routes
    void setInitialSolution(const vector<Route> &initialRoutes);

    void clear() override{
        model = new GRBModel(*env);
        x.clear();
        delta.clear();
        y.clear();
    }
public : ArcBasedModel(Graph &graph);
    ~ArcBasedModel(); // 析构函数
};

#endif // ARC_BASED_MODEL_H