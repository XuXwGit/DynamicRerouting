#ifndef TEST_CASE_GENERATOR_H
#define TEST_CASE_GENERATOR_H

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <string>

#include "Graph.h"
#include "Rerouting.h"

class TestCase
{
public:
    int N; // 节点数量
    int M; // 边数量
    int W; // 波长数量
    int K; // 服务数量
    int R; // 请求数量
    Graph graph;
    std::vector<Service> services;
    std::vector<int> requests;

    TestCase(int N, int M, int W, int K, int R)
        : N(N), M(M), W(W), K(K), R(R), graph(N, M, W) {}

    void generate();
    void save_to_file(const std::string &filename) const;
};

#endif // TEST_CASE_GENERATOR_H