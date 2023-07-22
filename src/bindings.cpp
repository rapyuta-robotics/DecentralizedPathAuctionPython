#include <decentralized_path_auction/graph.hpp>
#include <decentralized_path_auction/path_search.hpp>
#include <decentralized_path_auction/path_sync.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

#include <sstream>
#include <iomanip>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace decentralized_path_auction;

PYBIND11_MAKE_OPAQUE(Nodes);
PYBIND11_MAKE_OPAQUE(Path);
PYBIND11_MAKE_OPAQUE(PathSync::Paths);

std::ostream& operator<<(std::ostream& os, const Point& p) {
    return os << '(' << p[0] << ", " << p[1] << ", " << p[2] << ')';
}

std::ostream& operator<<(std::ostream& os, const PathSync::WaitStatus& w) {
    return os << "{ error: " << w.error << ", blocked_progress: " << w.blocked_progress
              << ", blocked_by: " << w.blocked_by << ", remaining_duration: " << w.remaining_duration << " }";
}

struct PathProgress {
    const Path& path;
    size_t progress_min;
    size_t progress_max;
    size_t blocked_progress;
};

std::ostream& operator<<(std::ostream& os, const PathProgress& p) {
    os << '{' << std::endl;
    for (int i = 0; i < p.path.size(); ++i) {
        if (i == p.blocked_progress) {
            os << "# ";
        } else if (i >= p.progress_min && i <= p.progress_max) {
            os << "> ";
        } else {
            os << "  ";
        }
        os << i << ": " << p.path[i].node->position << " S" << p.path[i].node->state << "    p " << std::setw(11)
           << p.path[i].price << "    d " << p.path[i].duration << std::endl;
    }
    return os << '}';
}

std::ostream& operator<<(std::ostream& os, const Path& p) {
    return os << PathProgress{p, 0, 0, p.size()};
}

std::ostream& operator<<(std::ostream& os, const PathSync& p) {
    for (const auto& [agent_id, info] : p.getPaths()) {
        auto wait_status = p.checkWaitStatus(agent_id);
        os << agent_id << "    pid " << info.path_id << "    len " << info.path.size();

        if (wait_status.error == PathSync::SOURCE_NODE_OUTBID) {
            os << "    source outbid by " << wait_status.blocked_by;
        } else if (!wait_status.blocked_by.empty()) {
            os << "    blocked by " << wait_status.blocked_by;
            size_t visits_until_block = wait_status.blocked_progress - info.progress_max;
            if (visits_until_block > 1) {
                os << " in " << visits_until_block - 1;
            }
        }

        os << std::endl
           << wait_status << "    "
           << PathProgress{info.path, info.progress_min, info.progress_max, wait_status.blocked_progress} << std::endl;
    }
    return os;
}

template <class T>
std::string to_string(const T& t) {
    std::ostringstream os;
    os << t;
    return os.str();
}

PYBIND11_MODULE(bindings, dpa) {
    // Constants
    dpa.attr("FLT_MAX") = FLT_MAX;

    // container bindings
    py::bind_vector<Nodes>(dpa, "Nodes");
    py::bind_vector<PathSearch::Destinations>(dpa, "Destinations");
    py::bind_vector<Path>(dpa, "Path").def("__str__", &to_string<Path>);
    py::bind_map<PathSync::Paths>(dpa, "Paths");

    // Node
    py::class_<Node, NodePtr> node(dpa, "Node");
    py::enum_<Node::State> state(node, "State");
    state.value("DEFAULT", Node::DEFAULT);
    state.value("NO_FALLBACK", Node::NO_FALLBACK);
    state.value("NO_STOPPING", Node::NO_STOPPING);
    state.value("DISABLED", Node::DISABLED);
    state.value("DELETED", Node::DELETED);
    state.export_values();
    node.def(py::init<Point, Node::State, Nodes>(), "position"_a, "state"_a = Node::DEFAULT, "edges"_a = Nodes());
    node.def_readonly("position", &Node::position);
    node.def_readwrite("state", &Node::state);
    node.def_readwrite("edges", &Node::edges);
    node.def_readwrite("reverse_edges", &Node::reverse_edges);
    // node.def_readwrite("auction", &Node::auction);
    node.def_property(
            "custom_data", [](const Node& node) -> size_t { return (size_t) node.custom_data; },
            [](Node& node, size_t val) { *(size_t*) (&node.custom_data) = val; });
    // node.def("validate", &Node::validate);

    // Visit
    py::class_<Visit> visit(dpa, "Visit");
    visit.def(py::init<NodePtr, float, float, float, float, float>(), "node"_a, "price"_a = 0, "duration"_a = 0,
            "base_price"_a = 0, "cost_estimate"_a = 0, "time_estimate"_a = 0);
    visit.def_readwrite("node", &Visit::node);
    visit.def_readwrite("price", &Visit::price);
    visit.def_readwrite("duration", &Visit::duration);
    visit.def_readwrite("base_price", &Visit::base_price);
    visit.def_readwrite("cost_estimate", &Visit::cost_estimate);
    visit.def_readwrite("time_estimate", &Visit::time_estimate);

    // NodeRTree
    py::class_<NodeRTree> node_rtree(dpa, "NodeRTree");
    node_rtree.def(py::init<>());
    node_rtree.def("insertNode", &NodeRTree::insertNode, "node"_a, "cost"_a = 0);
    node_rtree.def("removeNode", &NodeRTree::removeNode, "node"_a);
    node_rtree.def("clearNodes", &NodeRTree::clearNodes);

    node_rtree.def("findNode", &NodeRTree::findNode, "position"_a);
    node_rtree.def("findAnyNode", &NodeRTree::findAnyNode, "criteria"_a = Node::DISABLED);
    node_rtree.def("findNearestNode",
            static_cast<NodePtr (NodeRTree::*)(Point, Node::State) const>(&NodeRTree::findNearestNode), "position"_a,
            "criteria"_a = Node::DISABLED);
    node_rtree.def("containsNode", &NodeRTree::containsNode, "node"_a);

    // Graph
    py::class_<Graph, NodeRTree> graph(dpa, "Graph");
    graph.def(py::init<>());
    graph.def("updateReverseEdges", &Graph::updateReverseEdges);
    graph.def("clearNodes", &Graph::clearNodes);
    graph.def("removeNode", static_cast<bool (Graph::*)(NodePtr)>(&Graph::removeNode), "node"_a);
    graph.def("removeNode", static_cast<bool (Graph::*)(Point)>(&Graph::removeNode), "node"_a);
    graph.def("insertNode", &Graph::insertNode<>, "position"_a);

    // PathSearch
    py::class_<PathSearch> path_search(dpa, "PathSearch");

    py::enum_<PathSearch::Error>(path_search, "Error")
            .value("SUCCESS", PathSearch::SUCCESS)
            .value("FALLBACK_DIVERTED", PathSearch::FALLBACK_DIVERTED)
            .value("COST_LIMIT_EXCEEDED", PathSearch::COST_LIMIT_EXCEEDED)
            .value("ITERATIONS_REACHED", PathSearch::ITERATIONS_REACHED)
            .value("PATH_NOT_FOUND", PathSearch::PATH_NOT_FOUND)
            .value("PATH_EXTENDED", PathSearch::PATH_EXTENDED)
            .value("PATH_CONTRACTED", PathSearch::PATH_CONTRACTED)
            .value("DESTINATION_COST_PENALTY_NEGATIVE", PathSearch::DESTINATION_COST_PENALTY_NEGATIVE)
            .value("DESTINATION_DURATION_NEGATIVE", PathSearch::DESTINATION_DURATION_NEGATIVE)
            .value("DESTINATION_NODE_DISABLED", PathSearch::DESTINATION_NODE_DISABLED)
            .value("DESTINATION_NODE_INVALID", PathSearch::DESTINATION_NODE_INVALID)
            .value("DESTINATION_NODE_DUPLICATED", PathSearch::DESTINATION_NODE_DUPLICATED)
            .value("SOURCE_NODE_NOT_PROVIDED", PathSearch::SOURCE_NODE_NOT_PROVIDED)
            .value("SOURCE_NODE_INVALID", PathSearch::SOURCE_NODE_INVALID)
            .value("SOURCE_NODE_PRICE_INFINITE", PathSearch::SOURCE_NODE_PRICE_INFINITE)
            .value("CONFIG_AGENT_ID_EMPTY", PathSearch::CONFIG_AGENT_ID_EMPTY)
            .value("CONFIG_COST_LIMIT_NON_POSITIVE", PathSearch::CONFIG_COST_LIMIT_NON_POSITIVE)
            .value("CONFIG_PRICE_INCREMENT_NON_POSITIVE", PathSearch::CONFIG_PRICE_INCREMENT_NON_POSITIVE)
            .value("CONFIG_TIME_EXCHANGE_RATE_NON_POSITIVE", PathSearch::CONFIG_TIME_EXCHANGE_RATE_NON_POSITIVE)
            .value("CONFIG_TRAVEL_TIME_MISSING", PathSearch::CONFIG_TRAVEL_TIME_MISSING)
            .export_values();

    py::class_<PathSearch::Config>(path_search, "Config")
            .def_readwrite("agent_id", &PathSearch::Config::agent_id)
            .def_readwrite("cost_limit", &PathSearch::Config::cost_limit)
            .def_readwrite("price_increment", &PathSearch::Config::price_increment)
            .def_readwrite("time_exchange_rate", &PathSearch::Config::time_exchange_rate)
            .def_readwrite("travel_time", &PathSearch::Config::travel_time)
            .def(py::init<std::string, float, float, float, PathSearch::TravelTime>(), "agent_id"_a,
                    "cost_limit"_a = FLT_MAX, "price_increment"_a = 1, "time_exchange_rate"_a = 1,
                    "travel_time"_a = PathSearch::TravelTime(PathSearch::travelDistance))
            .def("validate", &PathSearch::Config::validate);

    py::class_<PathSearch::Destination>(dpa, "Destination")
            .def_readwrite("node", &PathSearch::Destination::node)
            .def_readwrite("queuing", &PathSearch::Destination::queuing)
            .def_readwrite("duration", &PathSearch::Destination::duration)
            .def_readwrite("cost_penalty", &PathSearch::Destination::cost_penalty)
            .def(py::init<NodePtr, bool, float, float>(), "node"_a, "queuing"_a = true, "duration"_a = FLT_MAX,
                    "cost_penalty"_a = 0);

    path_search.def(py::init<PathSearch::Config>(), "config"_a);
    path_search.def("getConfig", static_cast<PathSearch::Config& (PathSearch::*) ()>(&PathSearch::getConfig),
            py::return_value_policy::reference);

    path_search.def("getDestinations", &PathSearch::getDestinations);
    path_search.def("setDestinations", &PathSearch::setDestinations, "destinations"_a);
    path_search.def("initializePathCostEstimates", &PathSearch::initializePathCostEstimates, "source"_a = nullptr,
            "iterations"_a = 0);

    path_search.def("selectSource", &PathSearch::selectSource, "sources"_a);
    path_search.def("iterate", static_cast<PathSearch::Error (PathSearch::*)(Path&, size_t)>(&PathSearch::iterate),
            "path"_a, "iterations"_a = 0);
    path_search.def("iterate",
            static_cast<PathSearch::Error (PathSearch::*)(Path&, size_t, float)>(&PathSearch::iterate), "path"_a,
            "iterations"_a, "fallback_cost"_a);
    path_search.def("resetCostEstimates", &PathSearch::resetCostEstimates);

    // PathSync
    py::class_<PathSync> path_sync(dpa, "PathSync");

    py::enum_<PathSync::Error>(path_sync, "Error")
            .value("SUCCESS", PathSync::SUCCESS)
            .value("REMAINING_DURATION_INFINITE", PathSync::REMAINING_DURATION_INFINITE)
            .value("SOURCE_NODE_OUTBID", PathSync::SOURCE_NODE_OUTBID)
            .value("VISIT_PRICE_CONFLICT", PathSync::VISIT_PRICE_CONFLICT)
            .value("VISIT_NODE_INVALID", PathSync::VISIT_NODE_INVALID)
            .value("VISIT_NODE_DISABLED", PathSync::VISIT_NODE_DISABLED)
            .value("VISIT_DURATION_NEGATIVE", PathSync::VISIT_DURATION_NEGATIVE)
            .value("VISIT_PRICE_LESS_THAN_START_PRICE", PathSync::VISIT_PRICE_LESS_THAN_START_PRICE)
            .value("VISIT_BID_ALREADY_REMOVED", PathSync::VISIT_BID_ALREADY_REMOVED)
            .value("PATH_EMPTY", PathSync::PATH_EMPTY)
            .value("PATH_NODE_REPEATED", PathSync::PATH_NODE_REPEATED)
            .value("PATH_BID_DUPLICATED", PathSync::PATH_BID_DUPLICATED)
            .value("PATH_ID_STALE", PathSync::PATH_ID_STALE)
            .value("PATH_ID_MISMATCH", PathSync::PATH_ID_MISMATCH)
            .value("AGENT_ID_EMPTY", PathSync::AGENT_ID_EMPTY)
            .value("AGENT_ID_NOT_FOUND", PathSync::AGENT_ID_NOT_FOUND)
            .value("PROGRESS_DECREASE_DENIED", PathSync::PROGRESS_DECREASE_DENIED)
            .value("PROGRESS_EXCEED_PATH_SIZE", PathSync::PROGRESS_EXCEED_PATH_SIZE)
            .value("PROGRESS_MIN_EXCEED_MAX", PathSync::PROGRESS_MIN_EXCEED_MAX)
            .value("PROGRESS_NODE_LOOPBACK", PathSync::PROGRESS_NODE_LOOPBACK)
            .export_values();

    py::class_<PathSync::PathInfo>(path_sync, "PathInfo")
            .def_readwrite("path", &PathSync::PathInfo::path)
            .def_readwrite("path_id", &PathSync::PathInfo::path_id)
            .def_readwrite("progress_min", &PathSync::PathInfo::progress_min)
            .def_readwrite("progress_max", &PathSync::PathInfo::progress_max)
            .def_readwrite("stationary_block", &PathSync::PathInfo::stationary_block)
            .def(py::init<Path, size_t, size_t, size_t, bool>(), "path"_a, "path_id"_a = 0, "progress_min"_a = 0,
                    "progress_max"_a = 0, "stationary_block"_a = true);

    py::class_<PathSync::WaitStatus>(path_sync, "WaitStatus")
            .def_readwrite("error", &PathSync::WaitStatus::error)
            .def_readwrite("blocked_progress", &PathSync::WaitStatus::blocked_progress)
            .def_readwrite("blocked_by", &PathSync::WaitStatus::blocked_by)
            .def_readwrite("remaining_duration", &PathSync::WaitStatus::remaining_duration)
            .def("__str__", &to_string<PathSync::WaitStatus>);
    path_sync.def(py::init<>());
    path_sync.def(
            "updatePath", &PathSync::updatePath, "agent_id"_a, "path_id"_a, "path"_a, "stationary_block"_a = true);
    path_sync.def("updateProgress", &PathSync::updateProgress, "agent_id"_a, "path_id"_a, "progress_min"_a,
            "progress_max"_a, "price"_a = FLT_MAX, "duration"_a = 0);
    path_sync.def("removePath", &PathSync::removePath, "agent_id"_a);
    path_sync.def("clearPaths", &PathSync::clearPaths);
    path_sync.def("getPaths", &PathSync::getPaths, py::return_value_policy::reference);
    path_sync.def("checkWaitStatus", &PathSync::checkWaitStatus, "agent_id"_a, "patience"_a = FLT_MAX,
            "force_stationary_block"_a = false);
    path_sync.def("__str__", &to_string<PathSync>);
}
