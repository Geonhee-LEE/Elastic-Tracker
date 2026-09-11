#pragma once
#include <cstdio>

struct TrajOptCfg {
  int K=8; double vmax=3.0, amax=6.0;
  double rhoT=100, rhoP=10000, rhoV=1000, rhoA=1000;
  double rhoTracking=1000, rhosVisibility=10000, theta_clearance=0.8;
  double clearance_d=0.2, tolerance_d=0.3;
  double tracking_dur=3.0, tracking_dist=2.5, tracking_dt=0.2;
};

#include <traj_opt/minco.hpp>

namespace traj_opt {

class TrajOpt {
 public:
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // weight for time regularization term
  double rhoT_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double rhoP_, rhoV_, rhoA_;
  double rhoTracking_, rhosVisibility_;
  double clearance_d_, tolerance_d_, theta_clearance_;
  // corridor
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  // Minimum Jerk Optimizer
  minco::MinJerkOpt jerkOpt_;
  // weight for each vertex
  Eigen::VectorXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double* x_;
  double sum_T_;

  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<Eigen::Vector3d> tracking_visible_ps_;
  std::vector<double> tracking_thetas_;
  double tracking_dur_;
  double tracking_dist_;
  double tracking_dt_;

  // polyH utils
  bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                 std::vector<Eigen::MatrixXd>& vPs) const;

 public:
  TrajOpt(const TrajOptCfg& cfg);
  ~TrajOpt() {}

  void setBoundConds(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState);
  int optimize(const double& delta = 1e-4);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& target_predcit,
                     const std::vector<Eigen::Vector3d>& visible_ps,
                     const std::vector<double>& thetas,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& target_predcit,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);

  void addTimeIntPenalty(double& cost);
  void addTimeCost(double& cost);
  bool grad_cost_p_corridor(const Eigen::Vector3d& p,
                            const Eigen::MatrixXd& hPoly,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_p_tracking(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& target_p,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_p_landing(const Eigen::Vector3d& p,
                           const Eigen::Vector3d& target_p,
                           Eigen::Vector3d& gradp,
                           double& costp);
  bool grad_cost_visibility(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& center,
                            const Eigen::Vector3d& vis_p,
                            const double& theta,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_a(const Eigen::Vector3d& a,
                   Eigen::Vector3d& grada,
                   double& costa);
};

}  // namespace traj_opt