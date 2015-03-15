
#ifndef _DM_MP_RRT_H_
#define _DM_MP_RRT_H_

#include <vector>
#include <random>
#include <iostream>
#include "RRTData.h"

class RRT {
private:
    NodeTree rrt = NodeTree();
    std::vector<double> j_min, j_max;
    std::vector<int> ident;
    ulong c_dim = 0;

    std::default_random_engine _rand;
    std::ostream &sout;

    std::vector<double> sample();
    double dRand(double fMin, double fMax) {
        std::uniform_real_distribution<double> unif(fMin, fMax);
        return unif(_rand);
    }

public:
    RRT(std::vector<double> _j_min, std::vector<double> _j_max,
            std::vector<int> ident, std::ostream &sout);

    std::vector<RRTNodePtr> do_search(std::vector<double> start_config, std::vector<double> _goal_cfg,
        double step_size, double goal_freq);
};

#endif //_DM_MP_RRT_H_
