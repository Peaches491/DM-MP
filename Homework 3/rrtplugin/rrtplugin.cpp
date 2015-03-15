
#include <openrave/plugin.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/assign/list_of.hpp>

#include "../RRT.h"

using namespace OpenRAVE;
using namespace std;

class RRTModule : public ModuleBase {
private:
    EnvironmentBasePtr env;
    std::vector<RobotBasePtr> robots;
    RobotBasePtr robot;
public:
    RRTModule(EnvironmentBasePtr penv, std::istream &ss) : ModuleBase(penv) {

        std::cout << "Constructing plugin... ";

        RegisterCommand("RunRRT",
                boost::bind(&RRTModule::RunRRT, this, _1, _2),
                "This is an example command");
        env = penv;
        robots = std::vector<RobotBasePtr>();
        env->GetRobots(robots);
        robot = robots.at(0);

        std::cout << "DONE" << std::endl;
    }

    virtual ~RRTModule() {
    }

    bool RunRRT(std::ostream &sout, std::istream &sinput) {
        int dimension = -1;
        std::vector<double> start_config, goal_config;
        std::vector<double> j_min, j_max;
        std::vector<int> ident;
        double step_size = 0.0, goal_freq = 0.0;

        sout << "Converting input... ";
        string str;
        getline(sinput, str);
        std::stringstream ss(str);
        std::istream_iterator<std::string> begin(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> arg_vec(begin, end);
        sout << "DONE" << endl;

        // Loop through the tokenized string, looking for tokens which begin with a single dash.
        // Populate prameters accordingly
        for(uint i = 0; i < arg_vec.size(); i++) {
//            sout << "Interpreting: " << arg_vec.at(i) << endl;
            if(arg_vec.at(i)[0] == '-'){
                switch(arg_vec.at(i)[1]) {
                    case 'n':
                        dimension = atoi(arg_vec.at(++i).c_str());
                    case 's':
                        for(int j_idx = 0; j_idx < dimension; j_idx++){
                            i++;
                            start_config.push_back(atof(arg_vec.at(i).c_str()));
                        }
                        break;
                    case 'g':
                        for(int j_idx = 0; j_idx < dimension; j_idx++){
                            i++;
                            goal_config.push_back(atof(arg_vec.at(i).c_str()));
                        }
                        break;
                    case 'i':
                        for(int j_idx = 0; j_idx < dimension; j_idx++){
                            i++;
                            ident.push_back(atoi(arg_vec.at(i).c_str()));
                        }
                        break;
                    case 'd':
                        step_size = atof(arg_vec.at(++i).c_str());
                        break;
                    case 'f':
                        goal_freq = atof(arg_vec.at(++i).c_str());
                        break;
                    default:
                        sout << "Skipping token " << arg_vec.at(i) << endl;
                }
            }
        }


        sout << "Dimension: " << dimension << endl;

        sout << "Starting pose: ";
        for(auto j : start_config){
            sout << j << ", ";
        }
        sout << endl;

        sout << "Goal Pose: ";
        for(auto j : goal_config){
            sout << j << ", ";
        }
        sout << endl;

        sout << "Identification: ";
        for(auto j : ident){
            sout << j << ", ";
        }
        sout << endl;

        sout << "Step size: " << step_size << endl;

        sout << "Goal sampling frequency: " << goal_freq << endl;

        std::vector<int> joint_idxs;
        joint_idxs.push_back(15);
        joint_idxs.push_back(16);
        joint_idxs.push_back(17);
        joint_idxs.push_back(18);
        joint_idxs.push_back(19);
        joint_idxs.push_back(20);
        joint_idxs.push_back(21);
        robot->GetDOFLimits(j_min, j_max, joint_idxs);

        assert(dimension > 0);
        assert(j_min.size() == (uint)dimension);
        assert(j_max.size() == (uint)dimension);
        assert(start_config.size() == (uint)dimension);

        RRT* rrt = new RRT(j_min, j_max, ident, sout);

        rrt->do_search(start_config, goal_config, step_size, goal_freq);

        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string &interfacename, std::istream &sinput, EnvironmentBasePtr penv) {
    if (type == PT_Module && interfacename == "rrtmodule") {
        return InterfaceBasePtr(new RRTModule(penv, sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO &info) {
    info.interfacenames[PT_Module].push_back("RRTModule");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin() {
}