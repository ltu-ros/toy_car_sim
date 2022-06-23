#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>

/* point_to_ackermann
 *
 * Convert a twist vector into an AckermannDrive message
 *
 * Publish:   AckermannDrive on "/ackermann_cmd"
 * Subscribe: Twist on "twist_to_ackermann/twist"
 *
 */
class Twist2Ackermann
{
public:
  Twist2Ackermann();

private:
  void vecCallback(const geometry_msgs::Twist& twist);

  // ROS objects
  ros::NodeHandle nh_;
  ros::Publisher ackermann_pub_;
  ros::Subscriber point_sub_;

  //Last drive command
  ackermann_msgs::AckermannDrive drive_;
};



// Cnstructor
//   Set up publisher and subscriber
Twist2Ackermann::Twist2Ackermann()
{
  ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 100);
  point_sub_ = nh_.subscribe("twist_cmd", 100, &Twist2Ackermann::vecCallback, this);
}

// Callback function for the points
void Twist2Ackermann::vecCallback(const geometry_msgs::Twist& twist)
{

  ackermann_msgs::AckermannDrive drive;

  // Convert a twist into an AckermannDrive message
  //  twist.x and twist.y represent the power for the
  //  steering_angle and speed respectively
  drive.speed = twist.linear.x;
  drive.steering_angle = 0.33 * twist.angular.z;

  ackermann_pub_.publish(drive);
}

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("twist_to_ackermann is running!!");
  ros::init(argc, argv, "twist_to_ackermann");
  Twist2Ackermann point2ack;

  ros::spin();
}
