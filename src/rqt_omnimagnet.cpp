#include <rqt_omnimagnet/rqt_omnimagnet.h>
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace rqt_omnimagnet {

OmnimagTest::OmnimagTest()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("OmnimagTest");
}

void OmnimagTest::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  // event filter to detect key presses
  widget_->installEventFilter(this);
  widget_->setFocus(); // window must have focus to see key presses

  // advertise ROS topics
  pubLeft_   = getNodeHandle().advertise<std_msgs::Bool>("/footpedal/left",1);
}

void OmnimagTest::shutdownPlugin()
{
  pubLeft_.shutdown();
}

void OmnimagTest::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void OmnimagTest::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void OmnimagTest::publishLeft(bool currentState)
{
  std_msgs::Bool msg;
  msg.data = currentState;
  pubLeft_.publish(msg);
}


/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_EXPORT_CLASS(rqt_omnimagnet::OmnimagTest, rqt_gui_cpp::Plugin)
