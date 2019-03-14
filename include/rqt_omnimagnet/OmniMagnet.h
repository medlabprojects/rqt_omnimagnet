// OMNIMAG CLASS DOMINICK ROPELLA VANDERBILT UNIVERSITY 2019
// Contact: dominick.ropella@vanderbilt.edu

#ifndef OMNIMAGNET_H
#define OMNIMAGNET_H

#define _USE_MATH_DEFINES // necessary to use constants defined in cmath
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <array>


class OmniMagnet {

public:

  // default constructor uses properties of Vanderbilt's square-core Omnimagnet
  OmniMagnet(void);

  // constructs an OmniMagnet using the specified properties
  OmniMagnet(Eigen::Vector3d dipole_moment_per_amp, double dac_volts_per_amp);


  // Function that maps desired field [T] at a point [m] to omnimag coil currents
  Eigen::Vector3d calcCoilCurrents(Eigen::Vector3d desired_field, Eigen::Vector3d p);

  // Maps the desired coil currents to input control voltages for the amps
  Eigen::Vector3d calcDacVoltages(Eigen::Vector3d desired_coil_currents) {return desired_coil_currents * coil_current_scaling_;}

  Eigen::Vector3d dipoleMomentPerAmp(void) {return M_.diagonal();}

  void setCoilCurrentScaling(double amps_per_volt) {coil_current_scaling_ = amps_per_volt;}
  double coilCurrentScaling(void) {return coil_current_scaling_;}

private:

  // M is a linear transformation that maps the three applied coil currents
  // to the dipole moment m. [(A*m^2)/A]
  const Eigen::DiagonalMatrix<double, 3> M_;

  // Scaling from control voltage to output current [A/V]
  double coil_current_scaling_;

  // Last computed currents for each coil in the omnimagnet [A]
  Eigen::Vector3d coil_currents_ = {0, 0, 0};

  // Position of the tool in terms of the omnimagnet frame [m]
//  Eigen::Vector3d tool_position_ = {0, 0, 0};

  // Applied magnetic field on the tool in terms of omnimagnet frame [T]
//  Eigen::Vector3d b_field_applied_ = {0, 0, 0};


  // Magnetic Constant for Vacuum Permeabiltiy
  // [m*kg*s^-2*A^-2] [also known as 4pie-7 [T*m*A^-1]
  const double mu_0_ = 4*M_PI*1.0e-7;
//  const double mu_0_ = 1.25663706e-6;

  // 3x3 Identity Matrix
  const Eigen::Matrix3d I_ = Eigen::Matrix3d::Identity();
};


#endif // OMNIMAGNET_H
