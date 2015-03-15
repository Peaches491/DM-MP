
#ifndef _DM_MP_RRT_H_
#define _DM_MP_RRT_H_

#include <vector>
#include <random>
#include <iostream>
#include <time.h>
#include <stdio.h>

#include "RRTData.h"
#include "RRTConfig.h"

class RRT {
private:
    std::ostream& sout;
    std::default_random_engine _rand = std::default_random_engine(time(NULL));

    std::vector<double> sample();
    double dRand(double fMin, double fMax) {
        std::uniform_real_distribution<double> unif(fMin, fMax);
        return unif(_rand);
    }

public:
    RRTConfig* cfg;
    NodeTree root;

    RRT(RRTConfig* cfg);
    std::vector<RRTNode*> do_search(std::vector<double> start_config, std::vector<double> _goal_cfg,
        double step_size, double goal_freq);


    bool is_ident(int i) {
        return cfg->ident.at(i) == 1;
    };
};

#endif //_DM_MP_RRT_H_
