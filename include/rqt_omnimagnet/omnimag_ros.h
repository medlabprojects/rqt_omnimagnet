#ifndef OMNIMAG_H
#define OMNIMAG_H

#include <QObject>
#include <string>
#include <QString>
#include <stdint.h>
#include <QVector>
#include <utility>
#include <memory>
#include <rqt_omnimagnet/OmniMagnet.h>
#include <geometry_msgs/Vector3.h>
#include <medlab_motor_control_board/McbStatus.h>
#include <medlab_motor_control_board/EnableMotor.h>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>

class OmnimagRos
    : public QObject
{
  Q_OBJECT

public:
  OmnimagRos();
  ~OmnimagRos();
  std::string nodeName(void) {return nodeName_;}
  void init(std::string nodeName);
  bool isConnected(void);
  void setCoilCurrentScaling(double dac_volts_per_amp) {omnimagnet_->setCoilCurrentScaling(dac_volts_per_amp);}
  bool setCoilCurrents(Eigen::Vector3d coil_currents);
  bool setFieldAtPoint(Eigen::Vector3d desired_field, Eigen::Vector3d p);
  bool setDacs(Eigen::Vector3d voltages);
  bool setDac(int amp, double voltage);
  Eigen::Vector3d dacVoltages(void) {return dac_voltages_;}
  int     getNumAmps(void) {return (connected_ ? current_status_.number_modules : 0);} // returns number of detected amp modules
  QString getIp(void);   // returns IP address via uint8_t[4]
  QString getMac(void); // returns MAC address via uint8_t[6]
  bool    isRosControlEnabled(void) {return current_control_state_;} // NOTE: may not be accurate if state has changed since last status query
  bool    isAmpEnabled(uint8_t amp);
  double coilCurrentScaling(void) {return omnimagnet_->coilCurrentScaling();}

public slots:
  void enableRosControl(bool cmd);
  bool enableAmp(uint8_t amp, bool cmd);
  void enableAllAmps(bool cmd);
  void requestStatus(void); // for manual use if desired; automatically called by callbackStatusTimer
  bool resetDacs(void); // re-initializes all DACs (useful if you notice a amp not changing despite being enabled

private:
  std::unique_ptr<OmniMagnet> omnimagnet_;
  std::string     nodeName_;
  ros::NodeHandle nh_;
  ros::Publisher  pubEnableRos_;
  ros::Publisher  pubEnableAllAmps_;
  ros::Publisher  pubEnableAmp_;
  ros::Publisher  pubGetStatus_;
  ros::Publisher  pubSetDacs_;
  ros::Publisher  pubResetDacs_;
  ros::Subscriber subStatus_;
  ros::Subscriber subLimitSwitchEvent_;

  bool       connected_; // true after first status message received
  ros::Timer status_timer_;
  double     status_timer_interval_;
  void       callbackStatusTimer(const ros::TimerEvent &e);
  uint       status_requests_since_reply_; // number of requestStatus() calls since last response
  const uint status_requests_since_reply_limit_; // limit before connected_ is set false and connectionLost() is signaled
  bool       current_control_state_; // 0 => ROS Idle; 1 => ROS Control
  void       callbackSubStatus(const medlab_motor_control_board::McbStatus::ConstPtr& msg);
  Eigen::Vector3d dac_voltages_;
  medlab_motor_control_board::McbStatus current_status_; // most recently received status

signals:
  void connectionEstablished(void);
  void connectionLost(void);
  void controlStateChanged(bool); // 0 => ROS Idle; 1 => ROS Control
  void ampStateChanged(int); // index of amp that changed
  void newStatus(void); // emitted after every new status is received
  void lastAmpStates(QVector<bool>);
};
#endif // OMNIMAG_H
