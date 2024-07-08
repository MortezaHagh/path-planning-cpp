#include "path_planning_cpp/model_astar.h"

#include <math.h>

namespace PPCPP {
ModelAstar::ModelAstar(const InputModel& t_input_model, const CPTConfig& t_config)
    : BaseModel(t_input_model), config(t_config) {
    // update config
    n_adj = config.n_adj;
    dist_type = config.dist_type;
    expand_method = config.expand_method;
    std::vector<std::pair<int, int>> ixy;
    if (n_adj == 4) {
        edge_len = 1;
        ixy = {{{1, 0}, {0, 1}, {0, -1}, {-1, 0}}};
    } else {
        edge_len = std::sqrt(2);
        ixy = {{{1, 0}, {0, 1}, {0, -1}, {-1, 0}, {1, 1}, {-1, -1}, {-1, 1}, {1, -1}}};
    }

    // neighbors
    float cost;
    int ix, iy;
    int x, y, nn, head;
    std::vector<int> obss_nn = obss.nns;
    for (std::vector<Coord>::iterator it_n = nodes.coords.begin(); it_n != nodes.coords.end(); ++it_n) {
        std::vector<Neighbor> neighbors;

        // check if obstacle
        if (std::find(obss_nn.begin(), obss_nn.end(), it_n->nn) != obss_nn.end()) {
            neighbors_all.push_back(neighbors);
            continue;
        }

        //
        for (auto it_ixy : ixy) {
            ix = it_ixy.first;
            iy = it_ixy.second;
            x = it_n->x + ix;
            y = it_n->y + iy;
            head = std::atan2(iy, ix);

            // check if the Node is within array bound
            if (x >= map.x_min && x <= map.x_max && y >= map.y_min && y <= map.y_max) {
                // new node num
                nn = it_n->nn + ix + iy * (map.n_x);

                // check if it is obstacle
                if (std::find(obss_nn.begin(), obss_nn.end(), nn) == obss_nn.end()) {
                    // edge cost
                    if (ix != 0 && iy != 0) {
                        cost = edge_len;
                    } else {
                        cost = 1;
                    }

                    // update new neighbor
                    Neighbor new_neighbor;
                    new_neighbor.x = x;
                    new_neighbor.y = y;
                    new_neighbor.nn = nn;
                    new_neighbor.head = head;
                    new_neighbor.cost = cost;
                    new_neighbor.coord = Coord(x, y, nn);
                    neighbors.push_back(new_neighbor);
                }
            }
        }
        neighbors_all.push_back(neighbors);
    }
}
};  // namespace PPCPP