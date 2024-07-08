#ifndef MODEL_ASTAR_H
#define MODEL_ASTAR_H

#include <algorithm>  // for std::find
#include <string>
#include <utility>  // for std::pair
#include <vector>

#include "path_planning_cpp/model_base.h"

namespace PPCPP {

const float MAX_COST = 1000;

class Neighbor {
   public:
    int nn;
    float head;
    int x, y;
    float cost;
    Coord coord;
    Neighbor() {}
    Neighbor(int t_x, int t_y, int t_nn, int t_head, float t_cost)
        : x(t_x), y(t_y), nn(t_nn), head(t_head), cost(t_cost) {}
    ~Neighbor() {}
};

class Closed {
   public:
    int n_closed = 0;
    std::vector<int> nns;
    Closed() {}
    ~Closed() {}
    void add_closed(int t_nn) {
        n_closed += 1;
        nns.push_back(t_nn);
    }
    bool is_closed(int t_nn) { return std::find(nns.begin(), nns.end(), t_nn) != nns.end(); }
};

class TopNode {
   public:
    int nn = 0;
    int ind = -1;
    int p_nn = 0;
    // float head = 0;
    bool visited = false;
    float g_cost = MAX_COST;
    float f_cost = MAX_COST;
    // float h_cost = MAX_COST;
    // float head_cost = MAX_COST;
};

class Open {
   public:
    int n_open = 0;
    std::vector<int> nns;
    std::vector<float> costs;
    std::vector<TopNode> list;
    Open() {}
    ~Open() {}
    void add_open(TopNode t_top_node) {
        list.push_back(t_top_node);
        list.back().ind = n_open;
        nns.push_back(t_top_node.nn);
        costs.push_back(t_top_node.f_cost);
        n_open += 1;
    }
    bool is_open(const int& t_nn, int& ind) {
        std::vector<int>::iterator it = std::find(nns.begin(), nns.end(), t_nn);
        if (it != nns.end()) {
            ind = it - nns.begin();
            return true;
        }
        return false;
    }
};

class ModelAstar : public BaseModel {
   public:
    // config
    CPTConfig config;
    int n_adj = 4;
    std::string dist_type = "manhattan";
    std::string expand_method = "random";
    float edge_len;

    //
    std::vector<std::vector<Neighbor>> neighbors_all;

    //
    ModelAstar(const InputModel& input_model, const CPTConfig& config);
    ~ModelAstar() {}
};

};  // namespace PPCPP

#endif  // MODEL_ASTAR_H