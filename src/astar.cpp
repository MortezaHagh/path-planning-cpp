#include "path_planning_cpp/astar.h"

namespace PPCPP {

AStar::AStar(ModelAstar& t_model) : model(t_model) { initialize(); }
AStar::~AStar() {
    delete sol;
    sol = nullptr;
}

void AStar::initialize() {
    // start - goal
    if (from_start) {
        start = model.start;
        goal = model.goal;
    } else {
        start = model.goal;
        goal = model.start;
    }
    std::cout << "CPT, initialize, Start Node: " << start.nn << std::endl;
    std::cout << "CPT, initialize, Goal Node: " << goal.nn << std::endl;

    // closed - open - topnode
    create_top_node();                // top_node
    open.add_open(*top_node);         // open
    closed.add_closed(top_node->nn);  // closed

    // starting process time
    start_time = timeNow();
}

void AStar::astar() {
    std::vector<TopNode>* successors = new std::vector<TopNode>;
    while (top_node->nn != goal.nn) {
        // finf successors
        successors->clear();
        if (!expand(successors)) {
            std::cout << "CPT, astar: expand failed!" << std::endl;
            break;
        }

        // update open
        update_open(successors);

        // if no path found, exit
        if (open.n_open == 0) {
            std::cout << "CPT, astar: Open is empty, can not expand. Astar failed!" << std::endl;
            break;
        }

        // select top node
        select_top_node();

        // add the top node to closed
        closed.add_closed(top_node->nn);

        // print the top node
        if (true) {
            std::cout << "CPT, astar, Top Node: " << "nn " << top_node->nn << " gc: " << top_node->g_cost
                      << " fc: " << top_node->f_cost << std::endl;
        }
    }

    finalize();
}

void AStar::create_top_node() {
    top_node = new TopNode;
    top_node->ind = 0;
    top_node->visited = true;
    top_node->head = start.head;
    top_node->nn = start.nn;
    top_node->p_nn = start.nn;
    top_node->g_cost = 0;
    // top_node->h_cost = utils::distance(start.coord, goal.coord, model.dist_type);
}
bool AStar::expand(std::vector<TopNode>* successors) {
    for (auto& succ : model.neighbors_all.at(top_node->nn)) {
        if (closed.is_closed(succ.nn)) {
            continue;
        }
        n_expanded += 1;
        TopNode* successor = new TopNode;
        successor->head = succ.head;
        successor->nn = succ.nn;
        successor->p_nn = top_node->nn;
        float head_cost = std::fabs(successor->head - top_node->head);
        head_cost = (head_cost)*head_cost_coeff / (M_PI / 2);
        successor->g_cost = top_node->g_cost + succ.cost + head_cost;
        float h_cost = utils::distance(goal.coord, succ.coord, model.dist_type);
        successor->f_cost = successor->g_cost + h_cost;
        successors->push_back(*successor);
        // successor->g_cost = top_node->g_cost + succ.cost;
        // successor->h_cost = utils::distance(goal.coord, succ.coord, model.dist_type);
        // successor->f_cost = successor->g_cost + successor->h_cost;
    }

    if (successors->size() == 0) {
        std::cout << "CPT, astar: No successors found!" << std::endl;
        return false;
    }

    return true;
}
void AStar::update_open(std::vector<TopNode>* successors) {
    for (auto& succ : *successors) {
        int ind = -1;
        if (open.is_open(succ.nn, ind)) {
            if (succ.f_cost < open.costs.at(ind)) {
                n_opened += 1;
                n_reopened += 1;
                open.list.at(ind) = succ;
                open.list.at(ind).ind = ind;
            }
        } else {
            open.add_open(succ);
        }
    }
}

bool AStar::select_top_node() {
    float min_cost = std::numeric_limits<float>::max();
    int min_ind = -1;
    for (auto& op : open.list) {
        if (!op.visited) {
            if (op.f_cost < min_cost) {
                min_cost = op.f_cost;
                min_ind = op.ind;
            }
        }
    }

    if (min_ind == -1) {
        std::cout << "CPT, astar: can not find new top node!" << std::endl;
        return false;
    }

    top_node->visited = true;
    top_node = &open.list.at(min_ind);
    closed.add_closed(top_node->nn);
    return true;
}

void AStar::optimal_path(std::vector<int>& path_nodes) {
    path_nodes.push_back(goal.nn);
    int p_nn = top_node->p_nn;
    int p_ind;
    if (open.is_open(p_nn, p_ind)) {
        while (p_nn != start.nn) {
            path_nodes.push_back(p_nn);
            p_nn = open.list.at(p_ind).p_nn;
            if (!open.is_open(p_nn, p_ind)) {
                std::cout << "CPT, astar, optimal_path: unexpected error!" << std::endl;
                return;
            }
        }
    } else {
        std::cout << "CPT, astar, optimal_path: unexpected error!" << std::endl;
    }
    path_nodes.push_back(start.nn);
    std::reverse(path_nodes.begin(), path_nodes.end());
}

void AStar::finalize() {
    // end process time
    end_time = timeNow();
    duration = end_time - start_time;
    duration_cr = Duration(end_time - start_time);
    std::cout << "CPT, astar, finalize: duration: " << duration.count() << " s" << std::endl;
    std::cout << "CPT, astar, finalize: duration_cr: "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(duration_cr).count() << " ms" << std::endl;

    //
    if (top_node->nn == goal.nn) {
        success = true;
        std::cout << "CPT, astar, finalize: path found successfully!" << std::endl;
    } else {
        success = false;
        std::cout << "CPT, astar, finalize: No path found!" << std::endl;
        return;
    }

    // optimal path
    std::vector<int> path_nodes;
    optimal_path(path_nodes);

    // path nodes
    sol = new Sol(&model, path_nodes);
}

};  // namespace PPCPP
