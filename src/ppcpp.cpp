#include "path_planning_cpp/cp_tracker.h"
#include "ros/ros.h"

namespace PPCPP {}  // namespace PPCPP

int main(int argc, char** argv) {
    ros::init(argc, argv, "pp_astar_node");
    ros::NodeHandle nh;

    PPCPP::InputModel model_inputs;
    PPCPP::BaseModel base_model(model_inputs);
    PPCPP::CPTConfig cpt_config;
    PPCPP::ModelAstar astar_model(model_inputs, cpt_config);
    PPCPP::AStar astar(astar_model);
    astar.astar();
    // PPCPP::PPCPP cpt(base_model, cpt_config);
    PPCPP::Plotter plotter(base_model);
    plotter.plot_map();
    plotter.plot_sol(*astar.sol);

    return 0;
}
