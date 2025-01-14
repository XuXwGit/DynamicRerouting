#ifndef ROUTING_ALGORITHMS_H
#define ROUTING_ALGORITHMS_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <stdexcept>

#include "Graph.h"
#include "Model.h"
#include "ArcBasedModel.h"
#include "PathBasedModel.h"
#include "LagrangePathBasedModel.h"

/**
 * Base Class：RoutingAlgorithm
 * @details：The base class for all routing algorithms, containing a local graph and common interfaces
 */
class RoutingAlgorithm
{
protected:
    // The local graph for the routing algorithm
    std::unique_ptr<Graph> localGraph; 

public:
    std::string algoStr;

    RoutingAlgorithm(const Graph &globalGraph) : localGraph(globalGraph.clone()) {}

    virtual ~RoutingAlgorithm() = default;

    // Update the local graph from the global graph
    virtual void updateGraph(const Graph &globalGraph)
    {
        localGraph->updateFromGlobalGraph(globalGraph);
    }

    // Pure virtual function: find a path for a service at a given frequency
    virtual std::vector<int> findPath(const Service &service, int freq) = 0;

    // Pure virtual function: get new routes for interrupted services
    virtual std::vector<Route> getNewRoutes(
        const std::unordered_map<int, Service> &interruptedServices,
        std::vector<Route> &currentRoutes) = 0;
};

/**
 * @brief Single Source Routing Algorithm Base Class
 * @details The base class for single source routing algorithms, containing a local graph and common interfaces
 */
class SingleBaseRouting : public RoutingAlgorithm
{
public:
    SingleBaseRouting(Graph &graph) : RoutingAlgorithm(graph) {}

    // Note: children must achieve findPath
    virtual std::vector<int> findPath(const Service &service, int freq) override = 0;

    // Note: children must achieve getNewRoutes
    std::vector<Route> getNewRoutes(
        const std::unordered_map<int, Service> &interruptedServices,
        std::vector<Route> &currentRoutes) override;
};

/**
 * Default Routing Algorithm
 */
class DefaultRouting : public SingleBaseRouting
{
public:
    DefaultRouting(Graph &graph) : SingleBaseRouting(graph) {}

    std::vector<int> findPath(const Service &service, int freq) override;
};

/**
 * Dijkstra Shortest Routing Algorithm
 */
class DijkstraRouting : public SingleBaseRouting
{
private:
    void rollBackRoutes(std::vector<Route> &routes);

public:
    DijkstraRouting(Graph &graph) : SingleBaseRouting(graph) {}

    std::vector<int> findPath(const Service &service, int freq) override;

    std::vector<Route> getNewRoutes(
        const std::unordered_map<int, Service> &interruptedServices,
        std::vector<Route> &currentRoutes) override;
};

/**
 * A* Routing Algorithm Implementation (similar to Dijkstra)
 */
class AStarRouting : public SingleBaseRouting
{
public:
    AStarRouting(Graph &graph) : SingleBaseRouting(graph) {}

    std::vector<int> findPath(const Service &service, int freq) override;
};

/**
 * Integer Programming Based Rerouting Algorithms：ILPBasedRouting
 */
class ILPBasedRouting : public RoutingAlgorithm
{
protected:
    std::unique_ptr<Model> model; 

public:
    ILPBasedRouting(Graph &graph, std::unique_ptr<Model> model)
        : RoutingAlgorithm(graph), model(std::move(model)) {}

    void updateGraph(const Graph &globalGraph) override;

    std::vector<int> findPath(const Service &service, int freq) override;

    std::vector<Route> getNewRoutes(
        const std::unordered_map<int, Service> &interruptedServices,
        std::vector<Route> &currentRoutes) override;
};

/**
 * Arc-Based Integer Programming Model
 */
class ArcIPModelRouting : public ILPBasedRouting
{
public:
    ArcIPModelRouting(Graph &graph)
        : ILPBasedRouting(graph, std::make_unique<ArcBasedModel>(graph)) {}
};

/**
 * Path-Based Integer Programming Model
 */
class PathIPModelRouting : public ILPBasedRouting
{
public:
    PathIPModelRouting(Graph &graph)
        : ILPBasedRouting(graph, std::make_unique<PathBasedModel>(graph)) {}
};

/**
 * Path-Based Integer Programming Model
 */
class LagrangePathIPModelRouting : public ILPBasedRouting
{
public:
    LagrangePathIPModelRouting(Graph &graph)
        : ILPBasedRouting(graph, std::make_unique<LagrangePathBasedModel>(graph)) {}
};

#endif // ROUTING_ALGORITHMS_H