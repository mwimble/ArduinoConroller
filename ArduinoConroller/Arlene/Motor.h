#ifndef __MOTOR_H
#define __MOTOR_H

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros.h>
# include <boost/shared_ptr.hpp>

#// include <turtlesim/Pose.h>
# include <geometry_msgs/Twist.h>
//# include <turtlesim/SetPen.h>
//# include <turtlesim/TeleportRelative.h>
//# include <turtlesim/TeleportAbsolute.h>
//# include <turtlesim/Color.h>
#endif

//#include <QImage>
//#include <QPainter>
//#include <QPen>
//#include <QPointF>

#define PI 3.14159265

class Motor {
  public:
  typedef struct {
    float x;
    float y;
  } Point;
  
  Motor(const ros::NodeHandle& nh, const Point& pos, float orient);
  
  private:
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
};

#endif
