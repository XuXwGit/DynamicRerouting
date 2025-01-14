#include "ArcBasedModel.h"
#include <iostream>
#include <chrono>
#include <unordered_set>

using namespace std;

ArcBasedModel::ArcBasedModel(Graph &g)
    : Model(g), env(nullptr), model(nullptr)
{
    try
    {
        env = new GRBEnv(true); // 初始化 Gurobi 环境
        env->set("LogFile", "gurobi.log");
        env->start();
        model = new GRBModel(*env); // 初始化 Gurobi 模型
    }
    catch (GRBException &e)
    {
        cerr << "Gurobi initialization failed: " << e.getMessage() << endl;
    }
}

ArcBasedModel::~ArcBasedModel()
{
    delete model;
    delete env;
}

// 构建决策变量
void ArcBasedModel::buildDecisionVariables()
{
    try
    {
        // 初始化 x[d][e][w] 和 delta[d]
        for (auto [d, service] : interruptedServices)
        {
            delta[d] = vector<GRBVar>(graph.W + 1);
            x[d] = vector<vector<GRBVar>>(graph.M + 1, vector<GRBVar>(graph.W + 1));
            y[d] = vector<GRBVar>(graph.N + 1);
            for (int w = 1; w <= graph.W; ++w)
            {
                delta[d][w] = model->addVar(0.0, 1.0, 1.0, GRB_BINARY, "delta_" + to_string(d) + "_" + to_string(w));
            }

            for (int e = 1; e <= graph.M; ++e)
            {
                for (int w = 1; w <= graph.W; ++w)
                {
                    if (graph.checkEdgeFreqServiceAvailable(e, w, d))
                    {
                        x[d][e][w] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                                                   "x_" + to_string(d) + "_" + to_string(e) + "_" + to_string(w));
                    }
                    else
                    {
                        x[d][e][w] = model->addVar(0.0, 0.0, 0.0, GRB_BINARY,
                                                   "x_" + to_string(d) + "_" + to_string(e) + "_" + to_string(w));
                    }
                }
            }

            for (int i = 1; i <= graph.N; i++)
            {
                y[d][i] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY, "y_" + to_string(d) + "_" + to_string(i));
            }
        }
    }
    catch (GRBException &e)
    {
        cerr << "Error in decision variable creation: " << e.getMessage() << endl;
    }
}

void ArcBasedModel::addFlowConstr()
{
    for (auto [d, service] : interruptedServices)
    {
        int s = service.s, t = service.t;

        for (int v = 1; v <= graph.N; ++v)
        {
            for (int w = 1; w <= graph.W; ++w)
            {
                GRBLinExpr flow = 0;

                for (const auto &[neighbor, edgeId] : graph.adj.at(v))
                {
                    flow += x[d][edgeId][w];
                }

                if (v == s || v == t)
                {
                    model->addConstr(flow <= delta[d][w], "flow_s_" + to_string(d));
                    model->addConstr(flow >= delta[d][w], "flow_s_" + to_string(d));
                }
                else
                {
                    model->addConstr(flow <= 2 * y[d][v], "flow_mid_" + to_string(d) + "_" + to_string(v));
                    model->addConstr(flow >= 2 * y[d][v], "flow_mid_" + to_string(d) + "_" + to_string(v));
                }
            }
        }
    }
}
void ArcBasedModel::addEdgeFreqOnlyOneConstr()
{
    for (int edgeId = 1; edgeId <= graph.M; ++edgeId)
    {
        for (int w = 1; w <= graph.W; ++w)
        {
            GRBLinExpr usage = 0;
            for (auto [d, service] : interruptedServices)
            {
                model->addConstr(x[d][edgeId][w] <= delta[d][w], "edge_freq_" + to_string(edgeId) + "_" + to_string(w) + "_" + to_string(d));
                usage += x[d][edgeId][w];
            }
            model->addConstr(usage <= 1, "conflict_" + to_string(edgeId) + "_" + to_string(w));
        }
    }
}

void ArcBasedModel::addOnlyOneFreqConstr()
{
    for (auto [d, service] : interruptedServices)
    {
        GRBLinExpr freqUsage = 0;

        for (int w = 1; w <= graph.W; ++w)
        {
            freqUsage += delta[d][w];
        }
        model->addConstr(freqUsage <= 1, "only_one_freq_" + to_string(d));
    }
}

// 构建约束条件+
void ArcBasedModel::buildConstraints()
{
    try
    {
        // 1. 路径流约束
        addFlowConstr();

        // 2. 边&波长冲突约束
        addEdgeFreqOnlyOneConstr();

        // 4. 单一波长约束
        addOnlyOneFreqConstr();
    }
    catch (GRBException &e)
    {
        cerr << "Error in constraint creation: " << e.getMessage() << endl;
    }
}

// 构建目标函数
void ArcBasedModel::buildObjective()
{
    try
    {
        GRBLinExpr objective = 0;
        for (auto [d, service] : interruptedServices)
        {
            for (int w = 1; w <= graph.W; ++w)
            {
                objective += delta[d][w]; // 最大化成功路由的服务数量
            }
        }
        model->setObjective(objective, GRB_MAXIMIZE);
    }
    catch (GRBException &e)
    {
        std::cerr << "Error in objective creation: " << e.getMessage() << std::endl;
    }
}

void ArcBasedModel::optimize(std::vector<Route> &routes)
{
    try
    {
        model->set(GRB_IntParam_OutputFlag, false); // 关闭输出
        model->set(GRB_DoubleParam_TimeLimit, 1200); // 设置求解时间限制为 120 秒
        model->set(GRB_IntParam_MIPFocus, 2);   // 设置 MIPFocus 为 2，优先证明最优性

        setInitialSolution(routes);
        model->optimize();
    }
    catch (GRBException &e)
    {
        cerr << "Error during optimization: " << e.getMessage() << endl;
    }
}

// 提取解
void ArcBasedModel::extractSolution(std::vector<Route> &routes)
{
    optimize(routes);
    if (model->get(GRB_IntAttr_Status) != GRB_OPTIMAL)
    {
        return;
    }
    try
    {
        for (auto [d, service] : interruptedServices)
        {
            for (int w = 1; w <= graph.W; ++w)
            {
                if (delta[d][w].get(GRB_DoubleAttr_X) > 0.5)
                {
                    Route route;
                    route.service_id = interruptedServices[d].id;
                    int currentNode = interruptedServices[d].s; // 假设每个服务有一个起始节点
                    std::unordered_set<int> visited;            // 记录已经访问过的节点

                    visited.clear();             // 每次尝试新的波长时清空访问记录
                    visited.insert(currentNode); // 标记起始节点为已访问
                    while (true)
                    {
                        bool foundNextNode = false;
                        for (auto [neighborNode, edgeId] : graph.adj.at(currentNode))
                        {
                            if (neighborNode != interruptedServices[d].t && y[d][neighborNode].get(GRB_DoubleAttr_X) < 0.5)
                            {
                                continue; // 防止通过未激活的节点
                            }
                            if (x[d][edgeId][w].get(GRB_DoubleAttr_X) > 0.5)
                            {
                                if (visited.count(neighborNode) > 0)
                                {
                                    continue; // 防止回到已经访问过的节点
                                }
                                route.p.push_back(edgeId);
                                route.w = w;
                                currentNode = neighborNode;
                                visited.insert(currentNode); // 标记当前节点为已访问
                                foundNextNode = true;
                                break;
                            }
                        }

                        if (!foundNextNode || currentNode == interruptedServices[d].t)
                        {
                            break; // 如果没有找到下一个节点或到达目标节点，则退出循环
                        }
                    }
                    if (route.p.size() != 0)
                    {
                        routes.push_back(route);
                        break; // 找到路径后退出波长循环
                    }
                }
            }
        }
    }
    catch (GRBException &e)
    {
        cerr << "Error in solution extraction: " << e.getMessage() << endl;
    }
}

void ArcBasedModel::setInitialSolution(const vector<Route> &initialRoutes)
{
    try
    {
        for (const auto &route : initialRoutes)
        {
            if(!interruptedServices.count(route.service_id) || route.p.empty()){
                continue;
            }
            int d = route.service_id;
            int w = route.w;
            for (int edgeId : route.p)
            {
                x[d][edgeId][w].set(GRB_DoubleAttr_Start, 1.0);
            }
            delta[d][w].set(GRB_DoubleAttr_Start, 1.0);
        }
    }
    catch (GRBException &e)
    {
        cerr << "Error setting initial solution: " << e.getMessage() << endl;
    }
}