#ifndef rqt_omnimagnet__omnimagtest_H
#define rqt_omnimagnet__omnimagtest_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_omnimag_gui.h>
#include <QWidget>

#include <rqt_omnimagnet/omnimag_ros.h>
#include <memory>
#include <array>

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

protected slots:
  void connectNode(void);
  void connectionEstablished(void);
  void connectionLost(void);
  void controlStateChanged(bool controlState);
  void ampStateChanged(int amp);
  void powerOnOmnimag(void);
  void powerOffOmnimag(void);
  void modeChanged(int new_mode);
  void setCurrents(void);

private:
  Ui::omnimag_gui ui_;
  QWidget* widget_;
  std::unique_ptr<OmnimagRos> omnimag_;
  std::string omnimag_node_name_;
  std::array<double,3> omnimag_currents_;
  enum Mode {CURRENTS = 0, FIELD = 1, SINE = 2};
  Mode current_mode_ = Mode::CURRENTS;
  void setupCurrentsMode(void);
  void setupFieldMode(void);
  void setupSineMode(void);
};
} // namespace
#endif // my_namespace__my_plugin_H
