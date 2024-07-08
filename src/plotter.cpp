#include "path_planning_cpp/plotter.h"

#include <ros/ros.h>

namespace plt = matplotlibcpp;

namespace PPCPP {

void Plotter::plot_map() {
    std::cout << "Plotting map ..." << std::endl;
    // To initialize the plot
    plt::figure();
    plt::plot();
    plt::xlim(model.map.x_min - 1, model.map.x_max + 1);
    plt::ylim(model.map.y_min - 1, model.map.y_max + 1);

    // walls
    float marg1 = 0.5;
    float marg2 = 0.2;
    std::vector<float> xx1{model.map.x_min - marg1, model.map.x_max + marg1, model.map.x_max + marg1,
                           model.map.x_min - marg1};
    std::vector<float> yy1{model.map.y_min - marg1, model.map.y_min - marg1, model.map.y_max + marg1,
                           model.map.y_max + marg1};
    std::vector<float> xx2{model.map.x_min - marg2, model.map.x_max + marg2, model.map.x_max + marg2,
                           model.map.x_min - marg2};
    std::vector<float> yy2{model.map.y_min - marg2, model.map.y_min - marg2, model.map.y_max + marg2,
                           model.map.y_max + marg2};
    plt::fill(xx1, yy1, {{"color", "k"}});
    plt::fill(xx2, yy2, {{"color", "w"}});

    // obstacles
    plt::plot(model.obss.xs, model.obss.ys, "ko");

    // robot
    std::vector<int> sx{model.robot.start_c.x};
    std::vector<int> sy{model.robot.start_c.y};
    std::vector<int> gx{model.robot.goal_c.x};
    std::vector<int> gy{model.robot.goal_c.y};
    plt::plot(sx, sy, "bs");
    plt::plot(gx, gy, "gs");

    // // sol
    // plt::plot(sol.xs, sol.ys, "-og");

    // //
    // plt::show();
}

void Plotter::plot_sol(Sol sol) {
    plt::plot(sol.xs, sol.ys, "-og");
    plt::show();
}

};  // namespace PPCPP