//
// Created by peaches on 3/14/15.
//

#ifndef _DM_MP_RRTCONFIG_H_
#define _DM_MP_RRTCONFIG_H_

#include <iostream>
#include <vector>
#include <sys/types.h>

class RRTConfig {
public:
    std::vector<double> j_min, j_max;
    std::vector<int> ident;
    ulong c_dim = 0;
    std::ostream &sout;

    RRTConfig(std::vector<double> j_min, std::vector<double> j_max,
              std::vector<int> ident, ulong dim, std::ostream& sout)
            : j_min(j_min)
            , j_max(j_max)
            , ident(ident)
            , c_dim(dim)
            , sout(sout){};
};

#endif //_DM_MP_RRTCONFIG_H_
