#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}


//HomeWork 2.a------------------------------------------------------------------------------------------------------------
//Linear trajectory with trapezoidal velocity profile
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
  trajDuration_ = _trajDuration;
  accDuration_  = _accDuration;
  trajInit_     = _trajInit;
  trajEnd_      = _trajEnd;
  trajRadius_   = -1;
}

//Linear trajectory with cubic polinomial profile
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
  trajDuration_ = _trajDuration;
  accDuration_  = -1;
  trajInit_     = _trajInit;
  trajEnd_      = _trajEnd;
  trajRadius_   = -1;
}

//Circular trajectory with trapezoidal velocity profile
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
  trajDuration_ = _trajDuration;
  accDuration_  = _accDuration;
  trajInit_     = _trajInit;
  trajEnd_      = _trajInit;
  trajRadius_   = _trajRadius;
}
    
//Circular trajectory with cubic polinomial profile
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
  trajDuration_ = _trajDuration;
  accDuration_  = -1;
  trajInit_     = _trajInit;
  trajEnd_      = _trajInit;
  trajRadius_   = _trajRadius;
}
//---------------------------------------------------------------------------------------------------------------------------

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}


//HomeWork 2.b-c-----------------------------------------------------------------------------------------------
trajectory_point KDLPlanner::compute_trajectory(double time)
{

  double s, ds, dds;
  //Profile selection
  if(accDuration_ < 0)
    cubic_polinomial(time, s, ds, dds);
  else
    trapezoidal_vel(time, accDuration_, s, ds, dds);

  trajectory_point traj;
  //HomeWork 2.c
  //Curve selection
  if(trajRadius_ < 0)
  {
    traj.pos = trajInit_ + s*(trajEnd_ - trajInit_);
    traj.vel = ds*(trajEnd_ - trajInit_);
    traj.acc = dds*(trajEnd_ - trajInit_);
  }
  //HomeWork 2.b
  else
  {

    traj.pos(0) = trajInit_(0);
    traj.pos(1) = trajInit_(1) - trajRadius_*std::cos(2*M_PI*s);
    traj.pos(2) = trajInit_(2) - trajRadius_*std::sin(2*M_PI*s);
  
    traj.vel(0) = 0;
    traj.vel(1) = trajRadius_*2*M_PI*std::sin(2*M_PI*s)*ds;
    traj.vel(2) = -trajRadius_*2*M_PI*std::cos(2*M_PI*s)*ds;
  
    traj.acc(0) = 0;
    traj.acc(1) = trajRadius_*std::pow(2*M_PI,2)*std::cos(2*M_PI*s)*std::pow(ds,2) + trajRadius_*2*M_PI*std::sin(2*M_PI*s)*dds;
    traj.acc(2) = trajRadius_*std::pow(2*M_PI,2)*std::sin(2*M_PI*s)*std::pow(ds,2) - trajRadius_*2*M_PI*std::cos(2*M_PI*s)*dds;
  }

  return traj;
}
//-------------------------------------------------------------------------------------------------------------------------------------

//Homework 1.a-------------------------------------------------------------------------------------------------------------------------
void KDLPlanner::trapezoidal_vel(double time, double accDuration_, double &s, double &ds, double &dds)
{
  /* 
    trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time

     s,ds,dds = curvilinear abscissa ∈ [0,1] and its derivatives
  */

  double ddsc = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_);//FR pag.166 formula 4.5 with qi=0, qf=1, accDuration_<=trajDuration_/2

  if(time <= accDuration_)
  {
    s = 0.5*ddsc*std::pow(time,2);
    ds = ddsc*time;
    dds= ddsc;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s = ddsc*accDuration_*(time-accDuration_/2);
    ds = ddsc*accDuration_;
    dds = 0.0;
  }
  else
  {
    s = 1.0 - 0.5*ddsc*std::pow(trajDuration_-time,2);
    ds = ddsc*(trajDuration_-time);
    dds = -ddsc;
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------------------

//HomeWork 1.b-------------------------------------------------------------------------------------------------------------------------------------
void KDLPlanner::cubic_polinomial(double time, double &s, double &ds, double &dds)
{
  /* 
    cubic polinomial profile curvilinear abscissa:
    s(t) = a3*t^3 + a2*t^2 + a1*t + a0
    ds(t) = 3*a3*t^2 + 2*a2*t + a1 
    dds(t) = 6*a3*t + 2*a2

     time = current time
     trajDuration_  = final time
     s,ds,dds = curvilinear abscissa ∈ [0,1] and its derivatives
  */

  static Eigen::Matrix4d matCoeffs;
  static Eigen::Vector4d boundaries;
  static Eigen::Vector4d coeffs;

  /*
                        A*x == b

    s(0)  = a0 +  0    +  0      + 0         == 0
    ds(0) =  0 + a1    +  0      + 0         == 0
    s(tf) = a0 + a1*tf + a2*tf^2 + a3*tf^3   == 1
    ds(tf)=  0 + a1    + 2*a2*tf + 3*a3*tf^2 == 0
  */
  matCoeffs << 1,0,0,0,
               0,1,0,0,
               1,trajDuration_,pow(trajDuration_,2),pow(trajDuration_,3),
               0,1,2*trajDuration_,3*pow(trajDuration_,2),
  boundaries << 0,0,1,0;
  coeffs = matCoeffs.colPivHouseholderQr().solve(boundaries);
  
  s = coeffs(3)*pow(time,3) + coeffs(2)*pow(time,2) + coeffs(1)*time + coeffs(0);
  ds = 3*coeffs(3)*pow(time,2) + 2*coeffs(2)*time + coeffs(1);
  dds = 6*coeffs(3)*time + 2*coeffs(2);
}
//------------------------------------------------------------------------------------------------------------------------------------------