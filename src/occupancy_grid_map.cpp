// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// %EndTag(INCLUDES)%


/// El eje X es rojo.
/// El eje Y es verde.
/// El eje Z apunta hacia arriba y el marcador es azul.

class Mapa {
private:
  const int WIDTH = 24;          /// A lo largo del eje rojo x
  const int HEIGHT = 31;         /// A lo largo del eje verde
  const float RESOLUTION = 0.3f; /// [m/cell]

  ros::Publisher marker_pub;
  visualization_msgs::Marker velocity_mark;

  ros::Publisher grid_pub;
  nav_msgs::OccupancyGrid map;

  ros::NodeHandle& r_n;

public:

  /** Constructor. */
  Mapa(ros::NodeHandle& r_n) : r_n(r_n)
  {
    marker_pub = r_n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    grid_pub = r_n.advertise<nav_msgs::OccupancyGrid>("occupancy_marker", 1);
    llenaVelocidad();
    llenaMapa();
  }

  /** Recibe la velocidad en coordenadas del robot y publica para rviz. */
  void publicaVelocidad(const geometry_msgs::Twist& robotVel)
  {
    double magnitud = sqrt(pow(robotVel.linear.x, 2) + pow(robotVel.angular.z, 2));
    tf2::Quaternion q;
    double angle = atan2(robotVel.angular.z, robotVel.linear.x);
    q.setRPY(angle, angle, angle);  // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();
    //ROS_INFO_STREAM("x: " << robotVel.linear.x << "; z: " << robotVel.angular.z << "; Angle: "  << angle << "; Quaternion: " << q);

    velocity_mark.scale.x = magnitud;
    velocity_mark.pose.orientation = tf2::toMsg(q);
    marker_pub.publish(velocity_mark);
  }

  void publicate()
  {
    grid_pub.publish(map);
  }

private:
  void llenaVelocidad()
  {
    velocity_mark.header.frame_id = "base_link";
    velocity_mark.header.stamp = ros::Time();
    velocity_mark.ns = "kobu_namespace";
    velocity_mark.id = 0;
    velocity_mark.type = visualization_msgs::Marker::ARROW;
    velocity_mark.action = visualization_msgs::Marker::ADD;
    velocity_mark.pose.position.x = 0;
    velocity_mark.pose.position.y = 0;
    velocity_mark.pose.position.z = 0.2;
    velocity_mark.pose.orientation.x = 0.0;
    velocity_mark.pose.orientation.y = 0.0;
    velocity_mark.pose.orientation.z = 0.0;
    velocity_mark.pose.orientation.w = 1.0;
    velocity_mark.scale.x = 0.25;
    velocity_mark.scale.y = 0.05;
    velocity_mark.scale.z = 0.05;
    velocity_mark.color.a = 1.0; // Don't forget to set the alpha!
    velocity_mark.color.r = 0.0;
    velocity_mark.color.g = 1.0;
    velocity_mark.color.b = 0.5;
  }

  /** Agrega los obstáculos al mapa */
  void llenaMapa()
  {
    // %Tag(MAP_INIT)%
  
    // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
    map.header.frame_id = "/odom";
    map.header.stamp = ros::Time::now();   // No caduca

    map.info.resolution = RESOLUTION;      // [m/cell]
    map.info.width = WIDTH;                // [cells]
    map.info.height = HEIGHT;              // [cells]
    map.info.origin.position.x = -RESOLUTION * WIDTH / 2.0;
    map.info.origin.position.y = -RESOLUTION * HEIGHT / 2.0;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    //int8[] &_data = &map.data
    int size = WIDTH * HEIGHT;
    char* data = new char[size];
    for(int i = 0; i < size; i++) {
      data[i] = 0;
    }

    //data[0] = 50;                               // El origen está en la esquina inferior izquierda.
    fillRectangle(data, 0, 1, 0, WIDTH-1, 100);   // Renglón 0. Las columnas van de 0 a WIDTH-1.  Los renglones corren sobre el eje Y.
    fillRectangle(data, 0, 0, HEIGHT-1, 0, 100);  // Columna 0. Los renglones va de 0 a HEIGHT-1.  Las columnas corren sobre el eje X.
    fillRectangle(data, HEIGHT-1, 1, HEIGHT-1, WIDTH-1, 100);
    fillRectangle(data, 1, WIDTH-1, HEIGHT-1, WIDTH-1, 100);
    fillRectangle(data, 5, 1, 6, 11, 100);          // Mesa izq1
    fillRectangle(data, 11, 1, 13, 11, 100);        // Mesa izq2
    fillRectangle(data, 18, 1, 20, 11, 100);        // Mesa izq3
    fillRectangle(data, 5, 17, 6, 22, 100);          // Mesa der1
    fillRectangle(data, 11, 17, 13, 22, 100);          // Mesa der2
    fillRectangle(data, 18, 17, 20, 22, 100);          // Mesa der3

    map.data = std::vector<int8_t>(data, data + size);
  
    // %EndTag(MAP_INIT)%
  }


  /** Sets the cells between [i1,j1] and [i2,j2] inclusive as occupied with probability value. */
  void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value)
  {
    for(int i = i1; i <= i2; i++)
    {
      for(int j = j1; j <= j2; j++)
      {
        data[i*WIDTH+j] = value;
      }
    }
  }
};



/** Receives the message of the navigation goal from rviz. */
void receiveNavGoal(const geometry_msgs::PoseStamped& poseStamped)
{
  ROS_INFO("\nFrame: %s\nMove to: [%f, %f, %f] - [%f, %f, %f, %f]",
           poseStamped.header.frame_id.c_str(),
           poseStamped.pose.position.x,
           poseStamped.pose.position.y,
           poseStamped.pose.position.z,
           poseStamped.pose.orientation.x,
           poseStamped.pose.orientation.y,
           poseStamped.pose.orientation.z,
           poseStamped.pose.orientation.w);
}



// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_map");
  ros::NodeHandle n;
  ros::Rate r(1);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  Mapa mapa(n);
  ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 5, receiveNavGoal); // Máximo 5 mensajes en la cola.
  ros::Subscriber sub_vel = n.subscribe("/mobile_base/commands/velocity", 2, &Mapa::publicaVelocidad, &mapa); // Máximo 5 mensajes en la cola.
// %EndTag(INIT)%


  while (ros::ok())
  {
    mapa.publicate();

// %Tag(SLEEP_END)%
    ros::spinOnce();
    r.sleep();
  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
