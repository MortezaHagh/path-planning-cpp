#ifndef PLOTTER_H
#define PLOTTER_H

#include <path_planning_cpp/astar.h>

#include <iostream>
#include <vector>

#include "matplotlibcpp.h"
#include "path_planning_cpp/model_base.h"

namespace plt = matplotlibcpp;

namespace PPCPP {

class Plotter {
   public:
    BaseModel model;
    Plotter(const BaseModel& t_model) : model(t_model) {}
    ~Plotter() {}

    void plot_map();
    void plot_sol(Sol sol);
    void plot_show() { plt::show(); }
};

};  // namespace PPCPP

#endif