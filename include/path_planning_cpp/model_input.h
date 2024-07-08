#ifndef MODEL_INPUT_H
#define MODEL_INPUT_H

#include <vector>

namespace PPCPP {

class Coord {
   public:
    int nn;
    int x, y;
    Coord(){};
    Coord(int t_x, int t_y) : x(t_x), y(t_y) {}
    Coord(int t_x, int t_y, int t_nn) : x(t_x), y(t_y), nn(t_nn) {}

    ~Coord() {}
};

class InputModel {
   public:
    // robot
    float head;
    Coord start_c, goal_c;

    // area
    int lim;           ///< map limit
    int x_min, x_max;  ///< map x limits
    int y_min, y_max;  ///< map y limits

    // obstacles
    int n_obstacles = 0;             ///< number of obstacles
    std::vector<Coord> obstacles_c;  ///< obstacles coordinates

    InputModel();
    ~InputModel() {}

    void set_x_lim(int t_xmin, int t_xmax) {
        x_min = t_xmin;
        x_max = t_xmax;
    }

    void set_y_lim(int t_ymin, int t_ymax) {
        y_min = t_ymin;
        y_max = t_ymax;
    }

    void set_head(int t_head) { head = t_head; }
    void set_start(Coord t_c) { start_c = t_c; }
    void set_start(int t_x, int t_y) {
        Coord c(t_x, t_y);
        start_c = c;
    }
    void set_goal(Coord t_c) { goal_c = t_c; }
    void set_goal(int t_x, int t_y) {
        Coord c(t_x, t_y);
        goal_c = c;
    }

    void add_obstacle(int t_x, int t_y) {
        Coord c(t_x, t_y);
        obstacles_c.push_back(c);
        n_obstacles = obstacles_c.size();
    }
    void add_obstacle(Coord t_c) {
        obstacles_c.push_back(t_c);
        n_obstacles = obstacles_c.size();
    }
};

};  // namespace PPCPP
#endif
