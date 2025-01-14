#include "TestCase.h"

void TestCase::generate()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_node(1, N);
    std::uniform_int_distribution<> dis_wave(1, W);

    // 生成随机图
    for(int i = 1; i <= M; ++i)
    {
        int u = dis_node(gen);
        int v = dis_node(gen);
        if (u > v)
        {
            swap(u, v);
        }else if(u == v)
        {
            v = (v % N) + 1;
        }
        graph.addEdge({i, u, v});
    }

    // 生成随机服务
    for (int i = 0; i < K; ++i)
    {
        int s = dis_node(gen);
        int t = dis_node(gen);
        while (s == t)
        {
            t = dis_node(gen);
        }

        int w = 0;
        // 生成满足服务要求的路径
        std::vector<int> p;

        services.push_back({i + 1, s, t, w, p});
    }

    // 生成随机请求序列
    std::uniform_int_distribution<> dis_request(1, M);
    std::uniform_int_distribution<> dis_subsequence(1, 10); // 每个子序列中的故障请求数最多为10

    int total_requests = 0;
    while (total_requests < R)
    {
        int subsequence_size = min(dis_subsequence(gen), R - total_requests);
        for (int i = 0; i < subsequence_size; ++i)
        {
            requests.push_back(dis_request(gen));
        }
        total_requests += subsequence_size;
        if (total_requests < R)
        {
            requests.push_back(0); // 添加恢复初始状态的请求
            total_requests += 1;
        }
    }
}

void TestCase::save_to_file(const std::string &filename) const
{
    std::ofstream outfile(filename);
    assert(outfile.is_open());

    // 写入节点数、边数、波长数和服务数
    outfile << N << " " << M << " " << W << " " << K << std::endl;

    // 写入节点信息
    for (int i = 1; i <= N; ++i)
    {
        int x = i, y = i; // 为了简化，这里使用节点编号作为坐标
        outfile << i << " " << x << " " << y << std::endl;
    }

    // 写入边信息
    for (const auto &edge : graph.edges)
    {
        outfile << edge.id << " " << edge.u << " " << edge.v << std::endl;
    }

    // 写入服务信息
    for (const auto &service : services)
    {
        outfile << service.id << " " << service.s << " " << service.t << " " << service.w_star << " " << service.p_star.size();
        for (int edge_id : service.p_star)
        {
            outfile << " " << edge_id;
        }
        outfile << std::endl;
    }

    // 写入请求数量
    outfile << R << std::endl;

    // 写入请求序列
    for (int request : requests)
    {
        outfile << request << " ";
    }
    outfile << std::endl;

    outfile.close();
}