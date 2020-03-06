#include <teb_local_planner/teb_local_planner_ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <yaml-cpp/yaml.h>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
TebConfig config;
ViaPointContainer via_points;


int main(int argc, char* argv[])
{
  obst_vector.push_back( boost::make_shared<PointObstacle>(-3,1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(6,2) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );

  // Setup robot shape model
  RobotFootprintModelPtr robot_model = boost::make_shared<LineRobotFootprint>(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(3.0, 0.0));

  // Setup planner (homotopy class planning or just the local teb planner)
  if (true)
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));

  planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0));

  return 0;
}