#ifndef rqt_omnimagnet__omnimagtest_H
#define rqt_omnimagnet__omnimagtest_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_omnimag_gui.h>
#include <QWidget>
#include "ros/ros.h"
#include "std_msgs/Bool.h"



namespace rqt_omnimagnet {

class OmnimagTest
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  OmnimagTest();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();


private:
  Ui::omnimag_gui ui_;
  QWidget* widget_;

  ros::Publisher pubLeft_;

  void publishLeft(bool currentState);
};
} // namespace
#endif // my_namespace__my_plugin_H
