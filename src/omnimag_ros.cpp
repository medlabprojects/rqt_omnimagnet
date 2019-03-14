#include "rqt_omnimagnet/omnimag_ros.h"
#include <rqt_omnimagnet/OmniMagnet.h>
#include <QObject>
#include <QTimer>
#include <stdint.h>
#include <QString>
#include <QVector>
#include <string>
#include <vector>
#include <memory>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include <geometry_msgs/Vector3.h>
#include "medlab_motor_control_board/McbStatus.h"
#include "medlab_motor_control_board/EnableMotor.h"


OmnimagRos::OmnimagRos()
  : connected_(false)
  , status_timer_interval_(0.05) // [s] request status at 20 Hz
  , status_requests_since_reply_(0)
  , status_requests_since_reply_limit_(50)
{
  // register data types used in signals/slots
  qRegisterMetaType< QVector<bool> >();

  // omnimagnet
  omnimagnet_ = std::make_unique<OmniMagnet>(); // use default properties -> Vanderbilt omnimagnet
}

OmnimagRos::~OmnimagRos()
{
  // stop timer
  status_timer_.stop();

  // alert user
//  emit connectionLost();

  // ensure MCB is disabled
  enableRosControl(false);

  // shutdown node
  nh_.shutdown();
}

void OmnimagRos::init(std::string nodeName)
{
  nodeName_ = nodeName;

  // setup pubEnableRos_
  std::string topicEnableRosControl = "/" + nodeName_+ "/enable_ros_control";
  pubEnableRos_ = nh_.advertise<std_msgs::Bool>(topicEnableRosControl.c_str(),1);

  // setup pubEnableAllMotors_
  std::string topicEnableAllAmps = "/" + nodeName_+ "/enable_all_amps";
  pubEnableAllAmps_ = nh_.advertise<std_msgs::Bool>(topicEnableAllAmps.c_str(),1);

  // setup pubEnableMotor_
  std::string topicEnableAmp = "/" + nodeName_+ "/enable_amp";
  pubEnableAmp_ = nh_.advertise<medlab_motor_control_board::EnableMotor>(topicEnableAmp.c_str(),1);

  // setup pubGetStatus_
  std::string topicGetStatus = "/" + nodeName_+ "/get_status";
  pubGetStatus_ = nh_.advertise<std_msgs::Empty>(topicGetStatus.c_str(),1);

  // setup pubResetDacs_
  std::string topicResetDacs = "/" + nodeName_+ "/reset_dacs";
  pubResetDacs_ = nh_.advertise<std_msgs::Empty>(topicResetDacs.c_str(),1);

  // setup pubSetDacs_
  std::string topicSetDacs = "/" + nodeName_+ "/dac_voltages";
  pubSetDacs_ = nh_.advertise<geometry_msgs::Vector3>(topicSetDacs.c_str(),1);

  // setup subStatus_
  std::string topicStatus = "/" + nodeName + "/status";
  subStatus_ = nh_.subscribe(topicStatus.c_str(), 1, &OmnimagRos::callbackSubStatus, this);

  // start status timer
  status_timer_ = nh_.createTimer(ros::Duration(status_timer_interval_), &OmnimagRos::callbackStatusTimer, this);
}

bool OmnimagRos::setCoilCurrents(Eigen::Vector3d coil_currents)
{
  bool success = false;

  if(connected_ && current_control_state_){
    success = setDacs(omnimagnet_->calcDacVoltages(coil_currents));
  }

  return success;
}

bool OmnimagRos::setFieldAtPoint(Eigen::Vector3d desired_field, Eigen::Vector3d p)
{
  bool success = false;

  // must in ROS Control mode and enabled
  if(connected_ && current_control_state_){
    success = setCoilCurrents(omnimagnet_->calcCoilCurrents(desired_field, p));
  }

  return success;
}

void OmnimagRos::enableRosControl(bool cmd)
{
  std_msgs::Bool msg;
  msg.data = cmd;
  pubEnableRos_.publish(msg);
}

void OmnimagRos::enableAllAmps(bool cmd)
{
  std_msgs::Bool msg;
  msg.data = cmd;
  pubEnableAllAmps_.publish(msg);
}

bool OmnimagRos::enableAmp(uint8_t amp, bool cmd)
{
  bool success = false;

  if(connected_ && (amp<getNumAmps())){
    medlab_motor_control_board::EnableMotor msg;
    msg.motor = amp;
    msg.enable = cmd;
    pubEnableAmp_.publish(msg);

    success = true;
  }

  return success;
}

void OmnimagRos::requestStatus()
{
    // send request
    pubGetStatus_.publish(std_msgs::Empty());

    // increment counter
    status_requests_since_reply_++;

    if(status_requests_since_reply_ == status_requests_since_reply_limit_){
      connected_ = false;
      emit connectionLost();
    }
}

bool OmnimagRos::setDac(int amp, double voltage)
{
  bool success = false;

  if(!connected_){
    return success;
  }

  if((amp<getNumAmps()) && (amp>-1)){
    dac_voltages_(amp) = voltage; // ensure we keep others the same
    success = setDacs(dac_voltages_); // publish
  }
  return success;
}

bool OmnimagRos::setDacs(Eigen::Vector3d voltages)
{
  bool success = false;

  if(connected_){
    // assemble message
    geometry_msgs::Vector3 msg;
    msg.x = voltages(0);
    msg.y = voltages(1);
    msg.z = voltages(2);

    pubSetDacs_.publish(msg);
    dac_voltages_ = voltages;
    success = true;
  }

  return success;
}

bool OmnimagRos::resetDacs()
{
  bool success = false;

  if(connected_){
    std_msgs::Empty msg;
    pubResetDacs_.publish(msg);

    success = true;
  }

  return success;
}

void OmnimagRos::callbackStatusTimer(const ros::TimerEvent &e)
{
  requestStatus();
}

QString OmnimagRos::getIp(void)
{
  QString ip = "Not Connected";

  if(connected_){
    // convert IP address to a string
    ip =  QString::number(current_status_.ip[0]) + "."
        + QString::number(current_status_.ip[1]) + "."
        + QString::number(current_status_.ip[2]) + "."
        + QString::number(current_status_.ip[3]);
  }

  return ip;
}

QString OmnimagRos::getMac(void)
{
  QString mac = "Not Connected";

  if(connected_){
    // convert IP address to a hex string
    mac = QString::number(current_status_.mac[0],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(current_status_.mac[1],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(current_status_.mac[2],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(current_status_.mac[3],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(current_status_.mac[4],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(current_status_.mac[5],16).toUpper().rightJustified(2,'0');
  }

  return mac;
}

bool OmnimagRos::isAmpEnabled(uint8_t amp)
{
  if(isRosControlEnabled() && (amp<6)){
    return current_status_.motor_enabled[amp];
  }
  else{
    return false; // FIX: should probably return -1
  }
}

void OmnimagRos::callbackSubStatus(const medlab_motor_control_board::McbStatus::ConstPtr &msg)
{
  // reset counter
  status_requests_since_reply_ = 0;

  // emit signal if we were previously disconnected
  if(!connected_){
    connected_ = true;
    emit connectionEstablished();
  }

  // save previous status and control state
  medlab_motor_control_board::McbStatus previousStatus = current_status_;
  bool previousControlState = current_control_state_;

  // update status and control state
  current_status_ = *msg;
  std::string currentStateString = current_status_.current_state;
  current_control_state_ = !currentStateString.compare("ROS Control"); // compare() returns 0 if strings are equal

  // emit signal if control state has changed
  if(current_control_state_ != previousControlState){
    emit controlStateChanged(current_control_state_);
  }

  // emit signal(s) if any amp states have changed
  for(uint ii=0; ii<getNumAmps(); ii++){
    if(current_status_.motor_enabled[ii] != previousStatus.motor_enabled[ii]){
      emit ampStateChanged(ii);
    }
  }

  emit newStatus();
}
