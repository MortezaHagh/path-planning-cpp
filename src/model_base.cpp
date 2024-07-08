#include "path_planning_cpp/model_base.h"

namespace PPCPP {

int c2n(Coord c, Map t_map) { return (c.y - t_map.y_min) * t_map.n_x + (c.x - t_map.x_min); }

Map::Map(const InputModel& t_model) {
    lim = t_model.lim;
    x_min = t_model.x_min;
    x_max = t_model.x_max;
    y_min = t_model.y_min;
    y_max = t_model.y_max;
    n_x = x_max - x_min + 1;
    n_y = y_max - y_min + 1;
    n_nodes = n_x * n_y;
}

Robot::Robot(const InputModel& t_model, Map& t_map) {
    head = t_model.head;
    goal_c = t_model.goal_c;
    start_c = t_model.start_c;
    goal_c.nn = c2n(goal_c, t_map);
    start_c.nn = c2n(start_c, t_map);
    goal_nn = goal_c.nn;
    start_nn = start_c.nn;
}

Obstacles::Obstacles(const InputModel& t_model, Map& t_map) {
    set_radius(0.5);
    for (std::vector<Coord>::const_iterator it = t_model.obstacles_c.begin(); it != t_model.obstacles_c.end(); ++it) {
        nns.push_back(c2n(*it, t_map));
        coords.push_back(*it);
        coords.back().nn = nns.back();
        xs.push_back(it->x);
        ys.push_back(it->y);
    }
    n_obss = coords.size();
}

Nodes::Nodes(Map& t_map) {
    int n = 0;
    for (int y = t_map.y_min; y <= t_map.y_max; y++) {
        for (int x = t_map.x_min; x <= t_map.x_max; x++) {
            Coord c(x, y, n);
            coords.push_back(c);
            n++;
        }
    };
    n_nodes = coords.size();
}

StartGoal::StartGoal(Coord t_coord, int t_head) {
    x = t_coord.x;
    y = t_coord.y;
    head = t_head;
    nn = t_coord.nn;
    coord = t_coord;
}

BaseModel::BaseModel(const InputModel& t_model)
    : map(t_model),
      robot(t_model, map),
      obss(t_model, map),
      nodes(map),
      goal(robot.goal_c, robot.head),
      start(robot.start_c, robot.head) {}

bool CPTConfig::parse_json() {
    // json reader
    std::string config_file_path = "/home/morteza/catkin_ws/src/mh/path_planning_cpp/config/config.json";
    Json::Value j_config;
    std::ifstream ifs;
    ifs.open(config_file_path);
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, ifs, &j_config, &errs)) {
        std::cout << "CPT, model_base: parse json failed. " << errs << std::endl;
        return false;
    }

    // parse config
    n_adj = j_config["n_adj"].asInt();
    map_name = j_config["map_name"].asString();
    dist_type = j_config["dist_type"].asString();
    expand_method = j_config["expand_method"].asString();

    //
    std::cout << "CPT, model_base: CPTConfig: n_adj = " << n_adj << std::endl;
    std::cout << "CPT, model_base: CPTConfig: map_name = " << map_name << std::endl;

    return true;
}

};  // namespace PPCPP