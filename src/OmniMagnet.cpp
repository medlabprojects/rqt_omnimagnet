// OMNIMAG CLASS DOMINICK ROPELLA VANDERBILT UNIVERSITY 2019
// Contact: dominick.ropella@vanderbilt.edu

// BEFORE RUNNING - MAKE SURE PROGRAM CAN FIND EIGEN
// FOR JUST LINUX (NO ROS) PUT EIGEN AND UNSUPPORTED EIGEN FOLDERS INTO
// usr/local/include or usr/include

#include <rqt_omnimagnet/OmniMagnet.h>
#include <eigen3/Eigen/Dense>
#include <array>

OmniMagnet::OmniMagnet()
  : M_(Eigen::DiagonalMatrix<double,3>{26.2347, 26.2366, 26.2356}) // [(A*m^2)/A]
  , amp_scaling_(5.0) // [A/V] using AMC PS50A ampifiers with {-10V,10V} intput corresponding to {-50A,50A} output
{
}

OmniMagnet::OmniMagnet(Eigen::Vector3d dipole_moment_per_amp, double amp_scaling)
  : M_(Eigen::DiagonalMatrix<double,3>{dipole_moment_per_amp})
  , amp_scaling_(amp_scaling)
{
}

Eigen::Vector3d OmniMagnet::calcCoilCurrents(Eigen::Vector3d desired_field, Eigen::Vector3d p)
{
  // currents = (2*pi/mu_0) * ||p||^3 * M_inv * (3*p_hat*p_hat_t - 2I) * B

  Eigen::Vector3d coil_currents = (2*M_PI/mu_0_) * pow((p.norm()),3) * M_.inverse()
      * (3 * p.normalized() * p.normalized().transpose() - 2*I_) * desired_field;

  return coil_currents;
}
