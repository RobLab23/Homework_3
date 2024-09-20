#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"
#include <cmath>

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

class KDLPlanner
{

public:

    KDLPlanner(double _maxVel, double _maxAcc);
    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    //HomeWork 2.a------------------------------------------------------------------------------------------------------
    //Linear trajectory with trapezoidal velocity profile
    KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    
    //Linear trajectory with cubic polinomial profile
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    
    //Circular trajectory with trapezoidal velocity profile
    KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius);

    //Circular trajectory with cubic polinomial profile
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius);
    //-------------------------------------------------------------------------------------------------------------------

    // trajectory_point compute_trajectory(double time);
    trajectory_point compute_trajectory(double time);


private:



    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_, accDuration_, trajRadius_; //HomeWork 2.a : radius
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;

    //HomeWork 1.a
    void trapezoidal_vel(double time, double accDuration_, double &s, double &ds, double &dds);
    //HomeWork 1.b
    void cubic_polinomial(double time, double &s, double &ds, double &dds);

};

#endif
