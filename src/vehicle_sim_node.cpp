#include "vehicle_sim.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vehicle_simulation");
  ros::NodeHandle n;
  ros::Publisher msg_pub_ = n.advertise<automated_driving_msgs::ObjectStateArray>(
      "/sensor/laser/objects", 1000);

  Vehicle vehicle;
  // vehicle.SetVideoMode();
  vehicle.keyboard_control(n, msg_pub_);
}
