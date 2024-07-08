#ifndef MH_ASTAR_H
#define MH_ASTAR_H

#include <chrono>

#include "path_planning_cpp/model_astar.h"

namespace PPCPP {

typedef std::chrono::high_resolution_clock::time_point TimeVar;
typedef std::chrono::high_resolution_clock::duration Duration;
#define timeNow() std::chrono::high_resolution_clock::now()

class Sol {
   public:
    ModelAstar* model;
    float proc_time;
    std::vector<int> nns;
    std::vector<int> xs;
    std::vector<int> ys;
    std::vector<int> heads;
    Sol(ModelAstar* t_model, const std::vector<int> t_nns) : model(t_model), nns(t_nns) {
        int nn;
        for (int i = 0; i < nns.size(); i++) {
            nn = nns[i];
            xs.push_back(model->nodes.coords.at(nn).x);
            ys.push_back(model->nodes.coords.at(nn).y);
            // heads.push_back();
        }
    }
    // Sol() {}
    ~Sol() {}
};

class AStar {
   public:
    //
    ModelAstar model;
    Sol* sol;

    // astar config
    bool from_start = true;
    float head_cost_coeff = 1.0;

    // statistics
    int n_closed;
    int n_opened;
    int n_reopened;
    int n_expanded;
    int n_final_open;
    bool success = false;

    //
    TimeVar start_time, end_time;
    Duration duration_cr;
    std::chrono::duration<double> duration;

    //
    Open open;
    Closed closed;
    StartGoal goal;
    StartGoal start;
    TopNode* top_node;

    AStar(ModelAstar& model);
    ~AStar();
    void initialize();
    void create_top_node();
    void astar();
    bool expand(std::vector<TopNode>* successors);
    void update_open(std::vector<TopNode>* successors);
    bool select_top_node();
    void optimal_path(std::vector<int>& path_nodes);
    void finalize();
};

};  // namespace PPCPP

#endif