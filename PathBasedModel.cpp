#include "PathBasedModel.h"
#include <iostream>

PathBasedModel::PathBasedModel(Graph &g)
    : Model(g), env(nullptr), model(nullptr)
{
    try
    {
        env = new GRBEnv(true); 
        env->set("LogFile", "gurobi.log");
        env->start();
        model = new GRBModel(*env); 
        model->set(GRB_IntParam_OutputFlag, 0); 
    }
    catch (GRBException &e)
    {
        cerr << "Gurobi initialization failed: " << e.getMessage() << endl;
    }
}

void PathBasedModel::getCandidateRoutes()
{
    // 为每个服务生成所有可能的路径
    for (const auto &[id, service] : interruptedServices)
    {
        try
        {
            std::vector<Route> serviceRoutes = graph.getKCandidateReRoutings(service, 1000);
            candidateRoutes[id] = serviceRoutes;
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in get K Candidate ReRoutings of " << service.id << std::endl;
            std::cerr << e.what() << '\n';
        }
    }
}

void PathBasedModel::buildDecisionVariables()
{
    getCandidateRoutes();
    try
    {
        // 路径选择变量
        for (const auto &[d, service] : interruptedServices)
        {
            z[d] = std::vector<GRBVar>(candidateRoutes[d].size());
            for (int p = 0; p < candidateRoutes[d].size(); ++p)
            {
                z[d][p] = model->addVar(0, 1, 1, GRB_BINARY, "z_" + std::to_string(d) + "_" + std::to_string(p));
            }
        }
    }
    catch (GRBException &e)
    {
        std::cerr << "Error adding variables: " << e.getMessage() << std::endl;
    }
}

void PathBasedModel::buildConstraints()
{
    try
    {
        // 约束 1: 每个服务至多只能选择一个路径
        for (const auto &[d, services] : interruptedServices)
        {
            GRBLinExpr pathSelection = 0;
            for (int p = 0; p < candidateRoutes[d].size(); ++p)
            {
                pathSelection += z[d][p];
            }
            model->addConstr(pathSelection <= 1, "path_selection_" + std::to_string(d));
        }

        // 约束 2: 同边同波长冲突
        std::vector<std::vector<int>> counts(graph.M + 1, std::vector<int>(graph.W + 1, 0));
        std::vector<std::vector<GRBLinExpr>> conflicts(graph.M + 1, std::vector<GRBLinExpr>(graph.W + 1, 0));
        for (const auto &[d, services] : interruptedServices)
        {
            for (int p = 0; p < candidateRoutes.at(d).size(); ++p)
            {
                int w = candidateRoutes.at(d)[p].w;
                for (int e : candidateRoutes.at(d)[p].p)
                { 
                    counts[e][w] ++;
                    conflicts[e][w] += z[d][p];
                }
            }
        }
        for (int w = 1; w <= graph.W; ++w)
        {
            for (int e = 1; e <= graph.M; ++e)
            {
                if(counts[e][w] > 1)
                {
                    if (!graph.checkEdgeFreqAvailable(e, w))
                    {
                        model->addConstr(conflicts[e][w] <= 0, "conflict_edge_" + std::to_string(e) + "_freq_" + std::to_string(w));
                    }else{
                        model->addConstr(conflicts[e][w] <= 1, "conflict_edge_" + std::to_string(e) + "_freq_" + std::to_string(w));
                    }
                }
            }
        }

        // 约束 3: 路径上的边+波长 可用性
        // for (const auto &[d, services] : interruptedServices)
        // {
        //     for (int p = 0; p < candidateRoutes[d].size(); ++p)
        //     {
        //         for (int e : candidateRoutes[d][p].p)
        //         {
        //             for (int w = 1; w <= graph.W; ++w)
        //             {
        //                 if(!graph.checkEdgeFreqServiceAvailable(e, w, d))
        //                 {
        //                     model->addConstr(z[d][p] <= 0, "path_freq_consistency_" + std::to_string(d) + "_" + std::to_string(p) + "_" + std::to_string(w));
        //                 }else{
        //                  }
        //             }
        //         }
        //     }
        // }
    }
    catch (GRBException &e)
    {
        std::cerr << "Error adding constraints: " << e.getMessage() << std::endl;
    }
}

void PathBasedModel::buildObjective()
{
    try
    {
        GRBLinExpr objective = 0;
        for (const auto &[d, services] : interruptedServices)
        {
            for (int p = 0; p < candidateRoutes[d].size(); ++p)
            {
                objective += z[d][p];
            }
        }
        model->setObjective(objective, GRB_MAXIMIZE);
    }
    catch (GRBException &e)
    {
        std::cerr << "Error setting objective: " << e.getMessage() << std::endl;
    }
}

void PathBasedModel::optimize()
{
    try
    {
        model->set(GRB_IntParam_OutputFlag, false);
        model->set(GRB_DoubleParam_TimeLimit, 120); 

        model->set(GRB_IntParam_MIPFocus, 2); 

        model->optimize();
    }
    catch (GRBException &e)
    {
        cerr << "Error during optimization: " << e.getMessage() << endl;
    }
}

void PathBasedModel::extractSolution(std::vector<Route> &routes)
{
    optimize();
    if (model->get(GRB_IntAttr_Status) != GRB_OPTIMAL)
    {
        return;
    }
    try
    {
        for (const auto &[d, service] : interruptedServices)
        {
            for (int p = 0; p < candidateRoutes[d].size(); ++p)
            {
                if (z[d][p].get(GRB_DoubleAttr_X) > 0.5)
                {
                    if (this->graph.checkRouteValid(candidateRoutes[d][p]))
                    {
                        routes[d - 1] = candidateRoutes[d][p];
                        break; 
                    }
                    else
                    {
                        std::cerr << "Route " << d << " is invalid." << std::endl;
                    }
                    break;
                }
            }
        }
    }
    catch (GRBException &e)
    {
        cerr << "Error in solution extraction: " << e.getMessage() << endl;
    }
}
