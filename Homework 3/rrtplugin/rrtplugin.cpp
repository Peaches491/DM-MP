#include <algorithm>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/assign/list_of.hpp>
#include <iostream>


using namespace OpenRAVE;

class RRTModule : public ModuleBase {
private:
    EnvironmentBasePtr env;
    std::vector<RobotBasePtr> robots;
    RobotBasePtr robot;
    std::vector<double> goal_config;
    std::vector<double> j_min, j_max;
public:
    RRTModule(EnvironmentBasePtr penv, std::istream &ss) : ModuleBase(penv) {

        std::cout << "Constructing plugin... ";

        RegisterCommand("RunRRT",
                boost::bind(&RRTModule::RunRRT, this, _1, _2),
                "This is an example command");
        this->env = penv;
        this->robots = std::vector<RobotBasePtr>();
        this->env->GetRobots(robots);
        this->robot = robots.at(0);
        this->goal_config = std::vector<double>();

        std::vector<int> joint_idxs;
        joint_idxs.push_back(15);
        joint_idxs.push_back(16);
        joint_idxs.push_back(17);
        joint_idxs.push_back(18);
        joint_idxs.push_back(19);
        joint_idxs.push_back(20);
        joint_idxs.push_back(21);
        robot->GetDOFLimits(j_min, j_max, joint_idxs);

        std::cout << "DONE" << std::endl;
    }

    virtual ~RRTModule() {
    }

    bool RunRRT(std::ostream &sout, std::istream &sinput) {

        sout << "Converting input... ";
        sout.flush();

        std::istreambuf_iterator<char> eos;
        std::string input(std::istreambuf_iterator<char>(sinput), eos);
        std::stringstream ss(input);

        sout << "DONE" << std::endl;


        double i;
        while (ss >> i) {
            goal_config.push_back(i);
            if (ss.peek() == ',')
                ss.ignore();
        }

        sout << "In Collision: " << this->env->CheckCollision(robot) << std::endl;
        for (uint i = 0; i < goal_config.size(); i++) {
            sout << "[" << j_min.at(i) <<
                    " " << goal_config.at(i) <<
                    " " << j_max.at(i) <<
                    "]" << std::endl;
        }

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