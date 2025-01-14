#ifndef _EVALUCATE_H_
#define _EVALUCATE_H_

#include <vector>
#include <iostream>
#include <fstream> // 用于文件输入输出
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iomanip> // 添加此行

using namespace std;

const int M = 3 * 10000000;

// 计算每个故障请求的得分
int calculate_request_score(int b_q, int X, int K)
{
    return static_cast<int>(std::floor(pow(4.0, 10 - b_q) * 100.0 * (X / static_cast<double>(K))));
}

// 计算每个测试用例的总得分
int calculate_test_score(const std::vector<int> &request_scores)
{
    int total_penalty = std::accumulate(request_scores.begin(), request_scores.end(), 0);
    return std::max(1, M - total_penalty);
}

// 评估所有测试用例的得分
double evaluate_submission(const std::vector<std::vector<int>> &all_request_scores)
{
    std::vector<int> test_scores;
    for (const auto &request_scores : all_request_scores)
    {
        test_scores.push_back(calculate_test_score(request_scores));
    }
    return static_cast<double>(std::accumulate(test_scores.begin(), test_scores.end(), 0)) / test_scores.size();
}

// 评估所有测试用例的得分
double evaluate_submission(const std::vector<double> &test_scores)
{
    return static_cast<double>(std::accumulate(test_scores.begin(), test_scores.end(), 0)) / test_scores.size();
}

#endif _EVALUCATE_H_