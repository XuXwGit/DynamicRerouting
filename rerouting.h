#ifndef REROUTING_H
#define REROUTING_H

#include <vector>
#include <set>
#include <utility>

using namespace std;

// 公共数据结构
struct Edge
{
    int id; // 边的编号
    int u;  // 起始节点
    int v;  // 目标节点
};

struct Service
{
    int id;        // 服务编号
    int s;         // 源节点
    int t;         // 目标节点
    int w_star;         // 初始使用的波长
    vector<int> p_star; // 初始路径（边编号）
};

struct Route
{
    int service_id; // 服务编号
    int w;          // 使用的波长
    vector<int> p;  // 路径（边编号）

    bool operator< (const Route &other) const
    {
        if (service_id != other.service_id){
            return service_id < other.service_id;
        }
        if(p.size() != other.p.size()){
            return p.size() < other.p.size();
        }
        return w < other.w;
    }
};

// 路由算法枚举
enum ReroutingAlgorithmType
{
    DEFAULT,  // 默认算法
    DIJKSTRA, // Dijkstra 算法
    A_STAR,   // A* 算法
    ARC_IP,   // 基于弧的整数规划
    PATH_IP,  // 基于路由的整数规划
    LAGRANGE_DECOMPOSE, // 基于拉格朗日乘子的路径算法
};

// 必须提供的外部 API

void init(int N, int M, int W, int K,
          std::vector<Edge> E,
          std::vector<Service> D);
vector<Route> request(int r);

#endif // REROUTING_H