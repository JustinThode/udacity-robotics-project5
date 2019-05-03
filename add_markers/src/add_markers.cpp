#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

constexpr float pickX = 6.0F;
constexpr float pickY = -8.0F;
constexpr float dropX = 6.0F;
constexpr float dropY = -4.0F;

constexpr float accuracy = 0.3F;

ros::Publisher marker_pub;
int stage = 0;
visualization_msgs::Marker marker;

// control marker based on robot's proximity to the targets
void process_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  double xPos = msg->pose.pose.position.x;
  double yPos = msg->pose.pose.position.y;

  ROS_INFO("x %f y %f", xPos, yPos);

  if (stage == 0)
  {
    float dist = sqrt(pow(xPos - pickX, 2) + pow(yPos - pickY, 2));
    if (dist < accuracy)
    {
      ROS_INFO("Reached pick location");
      stage++;
      //simulate pickup
      sleep(5);
      marker.color.a = 0.0;
      marker_pub.publish(marker);
    }
  }
  else if (stage == 1)
  {
    float dist = sqrt(pow(xPos - dropX, 2) + pow(yPos - dropY, 2));
    if (dist < accuracy)
    {
      ROS_INFO("Reached drop location");
      stage++;
      marker.color.a = 1.0;
      marker.pose.position.x = dropX;
      marker.pose.position.y = dropY;
      marker_pub.publish(marker);
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle nh("~");
  ros::Rate r(1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  std::string param;
  nh.getParam("param", param);
  ROS_INFO("Got param: %s", param.c_str());

  //monitor robot position
  ros::Subscriber sub1 = nh.subscribe("/amcl_pose", 1, process_pose_callback);

  ROS_INFO("pickX %f", pickX);
  ROS_INFO("pickY %f", pickY);
  ROS_INFO("dropX %f", dropX);
  ROS_INFO("dropY %f", dropY);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pickX;
    marker.pose.position.y = pickY;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
    }

    ROS_INFO("Publish marker at pickup location");
    marker_pub.publish(marker);

    // special timed case
    if (param == "timed")
    {
      sleep(5);
      ROS_INFO("Disappear");
      marker.color.a = 0.0;
      marker_pub.publish(marker);
      sleep(5);
      ROS_INFO("Publish marker at dropoff location");
      marker.color.a = 1.0;
      marker.pose.position.x = dropX;
      marker.pose.position.y = dropY;
      marker_pub.publish(marker);
    }

    ros::spin();
}

