#include "path_planning_cpp/model_input.h"

namespace PPCPP {

InputModel::InputModel() {
    // define area
    set_x_lim(0, 10);
    set_y_lim(0, 10);

    // define robot
    set_head(0);
    set_start(0, 0);
    set_goal(8, 8);

    // define obstacles
    add_obstacle(2, 2);
    add_obstacle(3, 3);
    add_obstacle(7, 4);
    add_obstacle(8, 4);
}

};  // namespace PPCPP