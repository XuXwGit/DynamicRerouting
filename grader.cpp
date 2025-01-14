#include <vector>
#include <iostream>
#include <fstream> // 用于文件输入输出
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iomanip> // 添加此行

#include "rerouting.h"
#include "evalucate.h"

using namespace std;

int main()
{
    std::cout << std::fixed << std::setprecision(0);
    ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    const string data_path = "open/"; // 使用 fs 命名空间

    std::vector<string> filenames = {"01", "02", "03", "04", "05", "06", "07", "08", "09"};

    std::vector<std::vector<int>> all_request_scores;
    std::vector<double> test_scores;
    for (const string& filename : filenames) // 使用 fs 命名空间
    {
        // 获取文件或文件夹的路径
        // 打开文件输入流 
        ifstream inputFile(data_path + filename);

        // 检查文件是否成功打开
        if (!inputFile.is_open())
        {
            std::cerr << "Can't open file" << std::endl;
            return 1;
        }

        int n, m, w, k;
        inputFile >> n >> m >> w >> k;

        vector<Edge> edges(m);
        int id, u, v;
        int x, y;

        // 读取节点信息
        for (int i = 0; i < n; ++i)
        {
            inputFile >> id >> x >> y;
        }

        // 读取边信息
        for (int i = 0; i < m; ++i)
        {
            inputFile >> id >> u >> v;
            edges[id - 1] = {id, u, v};
        }

        vector<Service> services(k);
        int pLen;

        // 读取服务信息
        for (int i = 0; i < k; ++i)
        {
            inputFile >> id;
            --id;
            services[id].id = id + 1;
            inputFile >> services[id].s >> services[id].t >> services[id].w_star >> pLen;
            services[id].p_star.resize(pLen);
            for (int j = 0; j < pLen; ++j)
            {
                inputFile >> services[id].p_star[j];
            }
        }

        // 初始化数据
        // 节点数 N、边数 M、波长数 W 和服务数 K
        init(n, m, w, k, edges, services);

        // 一个测试用例的请求序列中最多包含 100 个请求
        // 表示测试中的请求数量
        int q;
        inputFile >> q;

        int r;
        int b_q = 0;
        int X = 0;
        int score_q = 1;
        std::vector<int> request_scores;
        while (q--)
        {
            // 请求 q ∈ {0, 1, . . . , M}
            inputFile >> r;
            // 返回长度为 K 的 std::vector<Route>, 每个服务的当前路由
            vector<Route> answer = request(r);

            if (r == 0)
            {
                b_q = 0;
            }
            else
            {
                b_q++;
            }

            // 输出答案
            X = 0;
            for (const auto &route : answer)
            {
                if (route.p.empty())
                {
                    X++;
                }

                // std::cout << route.service_id << ' ' << route.w << ' ' << route.p.size() << ' ';
                for (int edge_id : route.p)
                {
                    // std::cout << edge_id << ' ';
                }
                // std::cout << '\n';
            }
            // std::cout.flush();
            std::cout << "Request " << r << " No path: " << X << "/" << k << " Penalty Score: " << score_q << std::endl;
            score_q = calculate_request_score(b_q, X, k);
            request_scores.push_back(score_q);
        }

        double score = calculate_test_score(request_scores);
        std::cout << "Test " << filename << " Score: " << score << std::endl;
        all_request_scores.push_back(request_scores);
        test_scores.push_back(score);

        inputFile.close(); // 关闭文件流
    }

    std::cout << "Total Score: " << evaluate_submission(test_scores);

    return 0;
} 