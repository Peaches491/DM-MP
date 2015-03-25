
#include <openrave/plugin.h>
#include <openrave/planningutils.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <random>
#include <string>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/assign/list_of.hpp>

#include "../RRT.h"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace OpenRAVE;
using namespace std;

void my_handler(int s){
    printf("Caught signal %d\n",s);
    exit(1);
}

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
        int dimension = -1, max_iter = 10000;
        std::vector<double> start_config, goal_config;
        std::vector<double> j_min, j_max;
        std::vector<int> ident;
        double step_size = 0.0, goal_freq = 0.0;

        cout << "Converting input... ";
        string str;
        getline(sinput, str);
        std::stringstream ss(str);
        std::istream_iterator<std::string> begin(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> arg_vec(begin, end);
        cout << "DONE" << endl;

        // Loop through the tokenized string, looking for tokens which begin with a single dash.
        // Populate prameters accordingly
        for(uint i = 0; i < arg_vec.size(); i++) {
//            cout << "Interpreting: " << arg_vec.at(i) << endl;
            if(arg_vec.at(i)[0] == '-'){
                switch(arg_vec.at(i)[1]) {
                    case 'n':
                        dimension = atoi(arg_vec.at(++i).c_str());
                        break;
                    case 's':
                        for(int j_idx = 0; j_idx < dimension; j_idx++){
                            i++;
                            cout << "Pushing Start: " << arg_vec.at(i).c_str() << endl;
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
                    case 'm':
                        max_iter = atoi(arg_vec.at(++i).c_str());
                        break;
                    default:
                        cout << "Skipping token " << arg_vec.at(i) << endl;
                }
            }
        }


        cout << "Dimension: " << dimension << endl;

        cout << "Starting pose: ";
        for(auto j : start_config){
            cout << j << ", ";
        }
        cout << endl;

        cout << "Goal Pose: ";
        for(auto j : goal_config){
            cout << j << ", ";
        }
        cout << endl;

        cout << "Identification: ";
        for(auto j : ident){
            cout << j << ", ";
        }
        cout << endl;

        cout << "Step size: " << step_size << endl;

        cout << "Goal sampling frequency: " << goal_freq << endl;

        std::vector<int> joint_idxs;
        cout << "Joint Indices: ";
        for(auto idx : robot->GetActiveDOFIndices()) {
            cout << idx << ", ";
            joint_idxs.push_back(idx);
        }
        cout << endl;
        robot->GetDOFLimits(j_min, j_max, joint_idxs);

//        cout << endl;
//        for(uint i = 0; i < dimension; i++){
//            cout << "Start: " << start_config.at(i) << endl;
//            cout << "Joint Idx: " << joint_idxs.at(i) << endl;
//            cout << "Joint Name: " << robot->GetJointFromDOFIndex(joint_idxs.at(i))->GetName() << endl;
//            cout << "Min: " << j_min.at(i) << endl;
//            cout << "Max: " << j_max.at(i) << endl;
//            cout << endl;
//        }

        assert(dimension > 0);
        assert(j_min.size() == (uint)dimension);
        assert(j_max.size() == (uint)dimension);
        assert(start_config.size() == (uint)dimension);

        for(int i = 0; i < dimension; i++) {
            if(ident.at(i)) {
                j_min.at(i) = -2*M_PI;
                j_max.at(i) =  2*M_PI;
            }
        }

        struct sigaction sigIntHandler;
        sigIntHandler.sa_handler = my_handler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);

//        for(int i = 0; i < robot->GetManipulators().size(); i++) {
//            cout << i << ": " << robot->GetManipulators().at(i)->GetName() << endl;
//        }

        robot->SetActiveManipulator(robot->GetManipulators().at(robot->GetManipulators().size()-1));

        RRT* rrt = new RRT(new RRTConfig(j_min, j_max, ident, dimension, max_iter, cout, env, robot));
        std::pair<std::vector<RRTNode>, std::vector<RRTNode> > path = rrt->do_search(start_config, goal_config, step_size, goal_freq);

        vector<OpenRAVE::GraphHandlePtr> handles;
        vector<float> points;
        vector<float> short_points;
        for (auto node : path.first) {
            robot->SetActiveDOFValues(node.get_config());
            OpenRAVE::RobotBase::ManipulatorPtr manip = robot->GetActiveManipulator();
            OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
            points.push_back((float)trans.x);
            points.push_back((float)trans.y);
            points.push_back((float)trans.z);
        }
        for (auto node : path.second) {
            robot->SetActiveDOFValues(node.get_config());
            OpenRAVE::RobotBase::ManipulatorPtr manip = robot->GetActiveManipulator();
            OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
            short_points.push_back((float)trans.x);
            short_points.push_back((float)trans.y);
            short_points.push_back((float)trans.z);
        }
        handles.push_back(env->plot3(&points[0], (int)(points.size()/3), sizeof(float)*3, 3.0, OpenRAVE::RaveVector<double>({1.0, 0.3, 0.3, 1})));
        handles.push_back(env->plot3(&short_points[0], (int)(short_points.size()/3), sizeof(float)*3, 3.0, OpenRAVE::RaveVector<double>({0.3, 0.3, 1.0, 1})));

//        env->GetMutex().unlock();
//        env->GetMutex().lock();
//        robot->SetActiveDOFValues(start_config);
//        TrajectoryBasePtr traj = RaveCreateTrajectory(env);
//        traj->Init(robot->GetActiveConfigurationSpecification());
//        std::vector<double> active_values;
//        robot->GetActiveDOFValues(active_values);
//        traj->Insert(0, active_values);
//
//        uint i = 0;
//        for(i = 0; i < path.second.size(); i++){
//            traj->Insert(i+1, path.second.at(i).get_config());
//        }
//        traj->Insert(i + 1, goal_config);
//
//        planningutils::RetimeActiveDOFTrajectory(traj, robot, false, 3, 1, "ParabolicTrajectoryRetimer");
//        robot->GetController()->SetPath(traj);
//        env->GetMutex().unlock();
//
//        while (!robot->GetController()->IsDone()) {
//            usleep(1000);
//        }

//        if(path.first.size() > 2){
//            sout << "success";
//        } else {
//            sout << "failure";
//        }

        cout << "Search time: " << rrt->search_time/1000000.0L << endl;
        cout << "Smooth time: " << rrt->smooth_time/1000000.0L << endl;
        cout << "Node sampled: " << rrt->root.get_nodes()->size() << endl;
        cout << "Unsmooth length: " << path.first.size() << endl;
        cout << "Smooth length: " << path.second.size() << endl;
        cout << endl;


        sout << rrt->search_time/1000000.0L << ", ";
        sout << rrt->smooth_time/1000000.0L << ", ";
        sout << rrt->root.get_nodes()->size() << ", ";
        sout << path.first.size() << ", ";
        sout << path.second.size();
        sout << endl;

        env->GetMutex().unlock();

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