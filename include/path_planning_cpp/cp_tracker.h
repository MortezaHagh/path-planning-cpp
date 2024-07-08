#ifndef PPCPP_H
#define PPCPP_H

#include "path_planning_cpp/astar.h"
#include "path_planning_cpp/model_astar.h"
#include "path_planning_cpp/model_base.h"
#include "path_planning_cpp/model_input.h"
#include "path_planning_cpp/plotter.h"

namespace PPCPP {

class CPath {
   public:
    CPath() {}
    ~CPath() {}
};

class PPCPP {
   public:
    CPTConfig config;
    BaseModel model;
    PPCPP(const BaseModel& t_model, const CPTConfig& t_config) : model(t_model), config(t_config){};
    ~PPCPP() {}
};
};  // namespace PPCPP

#endif