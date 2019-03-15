#include <rqt_omnimagnet/rqt_omnimagnet.h>
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace rqt_omnimagnet {

OmnimagTest::OmnimagTest()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , omnimag_node_name_("omnimag")
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

  // connect gui buttons
  connect(ui_.button_connect_omnimag, SIGNAL(pressed()),
          this, SLOT(connectNode()));
}

void OmnimagTest::shutdownPlugin()
{
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

void OmnimagTest::connectNode()
{
  ui_.label_status->setText("CONNECTING...");

  // initialize omnimag
  omnimag_ = std::make_unique<OmnimagRos>();
  omnimag_->init(omnimag_node_name_);     

  // coil current scaling
  ui_.doubleSpinBox_coil_current_scaling->setValue(omnimag_->coilCurrentScaling());
  connect(ui_.doubleSpinBox_coil_current_scaling, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
      [this](double new_value){ omnimag_->setCoilCurrentScaling(new_value); }
  );

  // connect OmnimagRos signals/slots
  connect(omnimag_.get(), SIGNAL(controlStateChanged(bool)),
          this,           SLOT(controlStateChanged(bool)));

  connect(omnimag_.get(), SIGNAL(connectionEstablished()),
          this,           SLOT(connectionEstablished()));

  connect(omnimag_.get(), SIGNAL(connectionLost()),
          this,           SLOT(connectionLost()));

  connect(omnimag_.get(), SIGNAL(ampStateChanged(int)),
          this,           SLOT(ampStateChanged(int)));
}

void OmnimagTest::connectionEstablished()
{
  // status label
  ui_.label_status->setText("CONNECTED");
  connect(omnimag_.get(), &OmnimagRos::controlStateChanged,
          this, [this](bool state){ ui_.label_status->setText(state ? "Control" : "Idle"); } );


  // change connect button to disconnect
  ui_.button_connect_omnimag->setText("Disconnect");
  ui_.button_connect_omnimag->disconnect();
  connect(ui_.button_connect_omnimag, SIGNAL(pressed()),
          this,                       SLOT(connectionLost()));


  // connect reset DACS button
  connect(ui_.button_reset_dacs, &QAbstractButton::clicked,
          omnimag_.get(), &OmnimagRos::resetDacs);

  // connect tabWidget
  connect(ui_.tabWidget_mode, SIGNAL(currentChanged(int)),
          this,               SLOT(modeChanged(int)));

  // initialize the current mode
  modeChanged(ui_.tabWidget_mode->currentIndex());

  // omnimag power buttons
  connect(ui_.button_power_on,  SIGNAL(pressed()),
          this,                 SLOT(powerOnOmnimag()));
  connect(ui_.button_power_off, SIGNAL(pressed()),
          this,                 SLOT(powerOffOmnimag()));

  // enable checkboxes
  ui_.checkBox_inner->setCheckable(true);
  ui_.checkBox_middle->setCheckable(true);
  ui_.checkBox_outer->setCheckable(true);

  // put omnimag into ROS Control state
  omnimag_->enableRosControl(true);
}

void OmnimagTest::connectionLost()
{
  ui_.label_status->setText("DISCONNECTED");

  ui_.button_reset_dacs->disconnect();
  ui_.tabWidget_mode->disconnect();
  ui_.button_power_on->disconnect();
  ui_.button_power_off->disconnect();

  // disable checkboxes
  ui_.checkBox_inner->setCheckable(true);
  ui_.checkBox_middle->setCheckable(true);
  ui_.checkBox_outer->setCheckable(true);

  // enable connection button
  ui_.button_connect_omnimag->setText("Connect");
  connect(ui_.button_connect_omnimag, SIGNAL(pressed()),
          this, SLOT(connectNode()));
}

void OmnimagTest::controlStateChanged(bool controlState)
{

}

void OmnimagTest::ampStateChanged(int amp)
{

}

void OmnimagTest::powerOnOmnimag()
{
  // ensure ROS Control mode
  omnimag_->enableRosControl(true);
  ros::Duration(0.01).sleep(); // delay to allow message to be received/processed

  // set currents
  setCurrents();

  // enable amps
  omnimag_->enableAllAmps(true);
}

void OmnimagTest::powerOffOmnimag()
{
  // set ROS Idle mode
  omnimag_->enableRosControl(false);
}

void OmnimagTest::modeChanged(int new_mode)
{
  // power off for safety
  powerOffOmnimag();

  // transition from current mode
  switch (current_mode_){
  case Mode::CURRENTS:
    // disconnect
    ui_.checkBox_inner->disconnect();
    ui_.checkBox_middle->disconnect();
    ui_.checkBox_outer->disconnect();
    ui_.slider_current_inner->disconnect();
    ui_.slider_current_middle->disconnect();
    ui_.slider_current_outer->disconnect();
    ui_.button_set_currents->disconnect();
    ui_.button_reset_currents->disconnect();
    break;
  case Mode::FIELD:
    break;
  case Mode::SINE:
    break;
  }

  // setup new mode
  switch(new_mode){
  case Mode::CURRENTS:
    setupCurrentsMode();
    break;
  case Mode::FIELD:
    setupFieldMode();
    break;
  case Mode::SINE:
    setupSineMode();
    break;
  }
}

void OmnimagTest::setCurrents()
{
  // depends on which mode we are in
  switch (ui_.tabWidget_mode->currentIndex()){
  case Mode::CURRENTS:
  {
    Eigen::Vector3d desired_coil_currents;
    desired_coil_currents << ui_.doubleSpinBox_current_inner->value()
                           , ui_.doubleSpinBox_current_middle->value()
                           , ui_.doubleSpinBox_current_outer->value();
    omnimag_->setCoilCurrents(desired_coil_currents);
    break;
  }
  case Mode::FIELD:
    break;
  case Mode::SINE:
    break;
  }

  ros::Duration(0.005).sleep();
}

void OmnimagTest::updateBmagLabel()
{
  Eigen::Vector3d b;
  b << ui_.doubleSpinBox_bx->value(),
          ui_.doubleSpinBox_by->value(),
          ui_.doubleSpinBox_bz->value();
  double bmag = b.norm();
}

void OmnimagTest::setupCurrentsMode()
{
  // checkboxes 
  connect(ui_.checkBox_inner, &QAbstractButton::clicked, this,
          [this](){ omnimag_->enableAmp(0, ui_.checkBox_inner->isChecked()); } );

  connect(ui_.checkBox_middle, &QAbstractButton::clicked, this,
          [this](){ omnimag_->enableAmp(1, ui_.checkBox_middle->isChecked()); } );

  connect(ui_.checkBox_outer, &QAbstractButton::clicked, this,
          [this](){ omnimag_->enableAmp(2, ui_.checkBox_outer->isChecked()); } );

  connect(omnimag_.get(), &OmnimagRos::ampStateChanged, this,
          [this](int amp){
                  if(amp == 0){
                    ui_.checkBox_inner->setChecked(omnimag_->isAmpEnabled(amp));
                  }
                  else if (amp == 1){
                    ui_.checkBox_middle->setChecked(omnimag_->isAmpEnabled(amp));
                  }
                  else if (amp == 2){
                    ui_.checkBox_outer->setChecked(omnimag_->isAmpEnabled(amp));
                  }
                }
  );

  // update spin boxes as sliders are dragged
  connect(ui_.slider_current_inner, &QAbstractSlider::valueChanged, this,
          [this](int new_value){ ui_.doubleSpinBox_current_inner->setValue(50.0*new_value/100.0); } // slider range [-100,100]
  );

  connect(ui_.slider_current_middle, &QAbstractSlider::valueChanged, this,
          [this](int new_value){ ui_.doubleSpinBox_current_middle->setValue(50.0*new_value/100.0); } // slider range [-100,100]
  );

  connect(ui_.slider_current_outer, &QAbstractSlider::valueChanged, this,
          [this](int new_value){ ui_.doubleSpinBox_current_outer->setValue(50.0*new_value/100.0); } // slider range [-100,100]
  );

  // set currents button
  connect(ui_.button_set_currents, SIGNAL(pressed()),
          this,                    SLOT(setCurrents()));

  // reset currents button
  connect(ui_.button_reset_currents, &QAbstractButton::pressed,
          this, [this](){
                  ui_.slider_current_inner->setValue(0);
                  ui_.slider_current_middle->setValue(0);
                  ui_.slider_current_outer->setValue(0);
                  ui_.doubleSpinBox_current_inner->setValue(0.0);
                  ui_.doubleSpinBox_current_middle->setValue(0.0);
                  ui_.doubleSpinBox_current_outer->setValue(0.0);
                  setCurrents();
                }
  );



  current_mode_ = Mode::CURRENTS;
}

void OmnimagTest::setupFieldMode()
{

  // slider to scale ||B||
  connect(ui_.slider_bmag, SIGNAL(valueChanged()),
          ui_.label_bmag_percent, SLOT(setNum()));

  connect(ui_.slider_bmag, &QAbstractSlider::sliderReleased, this,
          [this](){
            ui_.doubleSpinBox_bx->setValue(ui_.doubleSpinBox_bx->value() * ui_.slider_bmag->value()/100.0);
            ui_.doubleSpinBox_by->setValue(ui_.doubleSpinBox_by->value() * ui_.slider_bmag->value()/100.0);
            ui_.doubleSpinBox_bz->setValue(ui_.doubleSpinBox_bz->value() * ui_.slider_bmag->value()/100.0);
          }
  );

  // ||B|| label
//  connect(ui_.)

  // set currents button
  connect(ui_.button_set_currents2, SIGNAL(pressed()),
          this,                    SLOT(setCurrents()));

  // reset currents button
  connect(ui_.button_reset_currents2, &QAbstractButton::pressed,
          this, [this](){
                  ui_.slider_current_inner->setValue(0); // will also update spin boxes
                  ui_.slider_current_middle->setValue(0);
                  ui_.slider_current_outer->setValue(0);
                  setCurrents();
                }
  );

  current_mode_ = Mode::FIELD;
}

void OmnimagTest::setupSineMode()
{
  current_mode_ = Mode::SINE;
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
