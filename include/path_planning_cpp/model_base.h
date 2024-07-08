#ifndef MODEL_BASE_H
#define MODEL_BASE_H

#include <jsoncpp/json/json.h>
#include <math.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "path_planning_cpp/model_input.h"

namespace PPCPP {

class Map {
   public:
    int lim;
    int n_nodes;
    int n_x, n_y;
    int x_min, y_min;
    int x_max, y_max;

    Map(const InputModel& t_model);
    ~Map() {}
};

class Robot {
   public:
    float head;
    int start_nn, goal_nn;
    Coord start_c, goal_c;
    Robot(const InputModel& t_model, Map& t_map);
    ~Robot() {}
};

class Obstacles {
   public:
    int n_obss;
    float radius;
    std::vector<int> nns;
    std::vector<int> xs;
    std::vector<int> ys;
    std::vector<Coord> coords;
    Obstacles(const InputModel& t_model, Map& t_map);
    ~Obstacles() {}
    void set_radius(float t_r) { radius = t_r; }
};

class Nodes {
   public:
    int n_nodes;
    std::vector<int> nns;
    std::vector<Coord> coords;
    Nodes(Map& t_map);
    ~Nodes() {}
};

class StartGoal {
   public:
    int nn;
    int x, y;
    float head;
    Coord coord;
    StartGoal() {}
    StartGoal(Coord t_coord, int t_head);
    ~StartGoal() {}
};

class BaseModel {
   public:
    Map map;
    Robot robot;
    Obstacles obss;
    Nodes nodes;
    StartGoal start;
    StartGoal goal;
    BaseModel(const InputModel& t_model);
    ~BaseModel() {}
};

class CPTConfig {
   public:
    int n_adj = 4;  // 4, 8
    std::string map_name;
    std::string dist_type = "euclidean";   // manhatten,  euclidean
    std::string expand_method = "random";  // random, heading
    CPTConfig() {
        if (parse_json()) {
            std::cout << "CPT, model_base: CPTConfig parsed successfully" << std::endl;
        } else {
            std::cout << "CPT, model_base: CPTConfig parse failed" << std::endl;
        }
    }
    ~CPTConfig() {}
    bool parse_json();
};

class utils {
   public:
    static float distance(Coord c1, Coord c2, std::string type = "manhattan") {
        if (type == "manhattan") {
            return std::abs(c1.x - c2.x) + std::abs(c1.y - c2.y);
        }
        return std::sqrt(std::pow(c1.x - c2.x, 2) + std::pow(c1.y - c2.y, 2));
    }
    static float distance(int x1, int y1, int x2, int y2, std::string type = "manhattan") {
        if (type == "manhattan") {
            return std::abs(x1 - x2) + std::abs(y1 - y2);
        }
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }
};

};  // namespace PPCPP

#endif
