//
// Created by peaches on 3/14/15.
//

#ifndef _DM_MP_RRTCONFIG_H_
#define _DM_MP_RRTCONFIG_H_

#include <iostream>
#include <vector>
#include <sys/types.h>
#include <openrave/openrave.h>

class RRTConfig {
public:
    std::vector<double> j_min, j_max;
    std::vector<int> ident;
    ulong c_dim = 0;
    int max_iter = 1000;
    std::ostream &sout;
    OpenRAVE::EnvironmentBasePtr env;
    OpenRAVE::RobotBasePtr robot;

    RRTConfig(std::vector<double> j_min, std::vector<double> j_max,
              std::vector<int> ident, ulong dim, int max_iter, std::ostream& sout,
              OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot)
            : j_min(j_min)
            , j_max(j_max)
            , ident(ident)
            , c_dim(dim)
            , max_iter(max_iter)
            , sout(sout)
            , env(env)
            , robot(robot)
    {};
};

#endif //_DM_MP_RRTCONFIG_H_
