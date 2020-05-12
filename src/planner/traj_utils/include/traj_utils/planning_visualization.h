#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline/non_uniform_bspline.h>
#include <iostream>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

using std::vector;
namespace rebound_planner {
class PlanningVisualization {
private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500
  };

  enum TOPOLOGICAL_PATH_PLANNING_ID {
    GRAPH_NODE = 1,
    GRAPH_EDGE = 100,
    RAW_PATH = 200,
    FILTERED_PATH = 300,
    SELECT_PATH = 400
  };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  ros::Publisher traj_pub_;      // 0
  ros::Publisher topo_pub_;      // 1
  ros::Publisher predict_pub_;   // 2
  ros::Publisher visib_pub_;     // 3, visibility constraints
  ros::Publisher frontier_pub_;  // 4, frontier searching
  ros::Publisher yaw_pub_;       // 5, yaw trajectory
  vector<ros::Publisher> pubs_;  //

  int last_topo_path1_num_;
  int last_topo_path2_num_;
  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;

public:
  PlanningVisualization(/* args */) {}
  ~PlanningVisualization() {}
  PlanningVisualization(ros::NodeHandle& nh);

  // draw basic shapes
  void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);

  // draw a piece-wise straight line path
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id = 0);

  // draw a polynomial trajectory
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color,
                          int id = 0);

  // draw a bspline trajectory
  void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);

  // draw a set of bspline trajectories generated in different phases
  void drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size);
  void drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size);

  void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double line_width);
  void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width);

  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);

  Eigen::Vector4d getColor(double h, double alpha = 1.0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  // SECTION developing
  void drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt);
  void drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt);


  /************************** zxzx display ****************************/
  ros::Publisher init_list_pub;
  ros::Publisher optimal_list_pub;
  ros::Publisher a_star_list_pub;
  ros::Publisher guide_vector_pub;
  ros::Publisher intermediate_state_pub;

  void displayMarkerList(ros::Publisher& pub, const vector<Eigen::Vector3d>& list, double scale,
                                                Eigen::Vector4d color, int id);
  void generatePathDisplayArray(visualization_msgs::MarkerArray &array, 
    const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color, int id);
  void generateArrowDisplayArray(visualization_msgs::MarkerArray &array, 
    const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color, int id);
  void displayInitList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
  void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
  void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
  void displayArrowList(ros::Publisher& pub, const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color, int id);
  // void displayIntermediateState(ros::Publisher& intermediate_pub, rebound_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
  // void displayNewArrow(ros::Publisher& guide_vector_pub, rebound_planner::BsplineOptimizer::Ptr optimizer);

};
}  // namespace rebound_planner
#endif