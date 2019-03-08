#include <rqt_footpedal/FootPedal.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QEvent>
#include <QKeyEvent>
#include <QFrame>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace rqt_footpedal {

FootPedal::FootPedal()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("FootPedal");
}

void FootPedal::initPlugin(qt_gui_cpp::PluginContext& context)
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

  connect(ui_.button_setFocus, SIGNAL(pressed()), this, SLOT(setFocus()));
  ui_.label_focusLost->setVisible(false);

  // advertise ROS topics
  pubLeft_   = getNodeHandle().advertise<std_msgs::Bool>("/footpedal/left",1);
  pubMiddle_ = getNodeHandle().advertise<std_msgs::Bool>("/footpedal/middle",1);
  pubRight_  = getNodeHandle().advertise<std_msgs::Bool>("/footpedal/right",1);
}

void FootPedal::shutdownPlugin()
{
  pubLeft_.shutdown();
  pubMiddle_.shutdown();
  pubRight_.shutdown();
}

void FootPedal::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void FootPedal::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void FootPedal::setFocus(void)
{
  // set window focus
  widget_->setFocus();

  // update gui
  ui_.button_setFocus->setText("Has Focus");
  ui_.button_setFocus->setFlat(true);
  ui_.label_focusLost->setVisible(false);
}

bool FootPedal::eventFilter(QObject *target, QEvent *event)
{
  if(event->type() == QEvent::KeyPress){
    QKeyEvent* key = static_cast<QKeyEvent*>(event);

    switch(key->key())
    {
    case Qt::Key_Q:
      // left pressed
      ui_.label_leftPedal->setFrameShadow(QFrame::Sunken);
      publishLeft(true);
      break;

    case Qt::Key_A:
      // left released
      ui_.label_leftPedal->setFrameShadow(QFrame::Raised);
      publishLeft(false);
      break;

    case Qt::Key_W:
      // middle pressed
      ui_.label_middlePedal->setFrameShadow(QFrame::Sunken);
      publishMiddle(true);
      break;

    case Qt::Key_S:
      // middle released
      ui_.label_middlePedal->setFrameShadow(QFrame::Raised);
      publishMiddle(false);
      break;

    case Qt::Key_E:
      // right pressed
      ui_.label_rightPedal->setFrameShadow(QFrame::Sunken);
      publishRight(true);
      break;

    case Qt::Key_D:
      // right released
      ui_.label_rightPedal->setFrameShadow(QFrame::Raised);
      publishRight(false);
      break;

    default:
      return rqt_gui_cpp::Plugin::eventFilter(target, event);
      break;
    }
  }
  else if( event->type() == QEvent::FocusOut ){
    ui_.button_setFocus->setText("Set Focus");
    ui_.button_setFocus->setFlat(false);
    ui_.label_focusLost->setVisible(true);

    // reset to ensure states are set to 'unpressed'
    ui_.label_leftPedal->setFrameShadow(QFrame::Raised);
    publishLeft(false);
    ui_.label_middlePedal->setFrameShadow(QFrame::Raised);
    publishMiddle(false);
    ui_.label_rightPedal->setFrameShadow(QFrame::Raised);
    publishRight(false);
  }
  else{
      return rqt_gui_cpp::Plugin::eventFilter(target, event);
  }

  return true;


//    if (key->key()==Qt::Key_F21) {
//      // left pedal pressed
//      ui_.label_left->setText("PRESSED");
//    }
//    else if (key->key() == Qt::Key_F22) {
//      // left predal released
//      ui_.label_left->setText("RELEASED");
//    }
//    else if (key->key() == Qt::Key_F20){
//      ui_.label_middle->setText("PRESSED");
//    }
//    else {
//      return rqt_gui_cpp::Plugin::eventFilter(target, event);
//    }
//    return true;
//  } else {
//    return rqt_gui_cpp::Plugin::eventFilter(target, event);
//  }

  //  return rqt_gui_cpp::Plugin::eventFilter(target, event);
}

void FootPedal::publishLeft(bool currentState)
{
  std_msgs::Bool msg;
  msg.data = currentState;
  pubLeft_.publish(msg);
}

void FootPedal::publishMiddle(bool currentState)
{
  std_msgs::Bool msg;
  msg.data = currentState;
  pubMiddle_.publish(msg);
}

void FootPedal::publishRight(bool currentState)
{
  std_msgs::Bool msg;
  msg.data = currentState;
  pubRight_.publish(msg);
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
PLUGINLIB_EXPORT_CLASS(rqt_footpedal::FootPedal, rqt_gui_cpp::Plugin)
