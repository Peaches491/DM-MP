
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
    std::default_random_engine _rand;
    std::uniform_real_distribution<double> goal_rand = std::uniform_real_distribution<double>(0.0, 1.0);
    std::vector<double> start_config;
    std::vector<double> goal_config;

    std::pair<bool, std::vector<double> > sample(double goal_freq, bool non_collide = true);
    double dRand(double fMin, double fMax) {
        std::uniform_real_distribution<double> unif(fMin, fMax);
        return unif(_rand);
    }
    void smooth(std::vector<RRTNode>& path, double step_size);

public:
    RRTConfig* cfg;
    NodeTree root;

    RRT(RRTConfig* cfg);
    std::pair<std::vector<RRTNode>, std::vector<RRTNode> > do_search(std::vector<double> start_config, std::vector<double> _goal_cfg,
        double step_size, double goal_freq);
    bool collides(std::vector<double> _joints);
    std::pair<bool, RRTNode> connect(int nn_id, std::vector<double> smp, double step_size, bool connect=true);
    std::pair<bool, std::vector<RRTNode> > connect_points(std::vector<double> start, std::vector<double> end, double step_size, RRTConfig* cfg, bool check);

    bool is_ident(int i) {
        return cfg->ident.at(i) == 1;
    };
};

#endif //_DM_MP_RRT_H_
