#include "inertial_sense.h"

int main(int argc, char**argv)
{
  // ros::init(argc, argv, "inertial_sense_node");
  // InertialSenseROS thing;
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   thing.update();
  // }
  // return 0;

  rclcpp::init(argc, argv);

  try
  {
    auto thing = std::make_shared<InertialSenseROS>();
    while (rclcpp::ok())
    {
      rclcpp::spin_some(thing);
      // thing->spin_once();
      thing->update();
    }
  }
  catch (std::runtime_error &e)
  {
    std::cerr << "inertial_sense_node exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}