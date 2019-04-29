// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// %EndTag(INCLUDES)%


///
/// Clases que contienen información
///

class VelocidadKobuki {
private:
  double _linear;
  double _angular;

public:
  void linear(double linear)
  {
    _linear = linear;
  }

  void angular(double angular)
  {
    _angular = angular;
  }
};

/**
 * Localización del robot en el piso, con orientación.
 */
class Loc2D
{
private:
  double _x;
  double _y;
  double _angulo;

public:
  Loc2D() : _x(0), _y(0), _angulo(0) {}
  Loc2D(double x, double y, double angulo) : _x(x), _y(y), _angulo(angulo){}
  double x() { return _x; }
  double y() { return _y; }
  double ang() { return _angulo; }
  void x(double x) { this->_x = x; }
  void y(double y) { this->_y = y; }
  void ang(double angulo) { this->_angulo = angulo; }
};


///
/// Funciones auxiliares
///

/**
 * Devuelve ángulo en el rango [-PI, PI]
 * @param angulo
 * @return ángulo normalizado
 */
double anguloEnRango(double angulo)
{
  angulo = remainder(angulo, 2.0 * M_PI);
  if (angulo > M_PI) angulo -= 2.0 * M_PI;
  return angulo;
}

/// El eje X es rojo.
/// El eje Y es verde.
/// El eje Z apunta hacia arriba y el marcador es azul.

class Sonar
{
private:
  double angulo;  // Ángulo con respecto al frente del robot

public:
  /** Obtiene las coordenadas del sensor dado el centro del robot. */
  Loc2D getPosicion(double xr, double yr)
  {

  }
};

class RobotInfo
{
private:
  VelocidadKobuki _velocidad;
  Loc2D _posicion;

public:
  void extraePosicion(const nav_msgs::Odometry& odom)
  {
    _posicion.x(odom.pose.pose.position.x);
    _posicion.y(odom.pose.pose.position.y);

    // TODO
    //tf2::Quaternion q;
    //_posicion.ang(odom.pose.pose.orientation);
  }

  VelocidadKobuki& velocidad() { return _velocidad; }

  Loc2D& posicion() { return _posicion; }
};



typedef struct structCoordsCelda {
  int i;
  int j;
} CoordsCelda;

class Mapa {
private:
  const int WIDTH = 24;          /// A lo largo del eje rojo x
  const int HEIGHT = 31;         /// A lo largo del eje verde
  const float RESOLUTION = 0.3f; /// [m/cell]
  const int OCUPADA = 100;

  ros::Publisher marker_pub;
  visualization_msgs::Marker velocity_mark;

  ros::Publisher grid_pub;
  ros::Publisher grid_pub_marcas;
  nav_msgs::OccupancyGrid mapa;
  nav_msgs::OccupancyGrid mapa_marcas;

  ros::NodeHandle& r_n;

  /// INFO
  RobotInfo _robot_info;
  CoordsCelda _celdaPrevia;
  int _colorPrevio;

public:

  /** Constructor. */
  Mapa(ros::NodeHandle& r_n) : r_n(r_n), _colorPrevio(-1)
  {
    marker_pub = r_n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    grid_pub = r_n.advertise<nav_msgs::OccupancyGrid>("occupancy_marker", 1);
    grid_pub_marcas = r_n.advertise<nav_msgs::OccupancyGrid>("occupancy_marker_marcas", 1);
    llenaVelocidad();
    llenaMapa();
  }

  /** Recibe la velocidad en coordenadas del robot y publica para rviz. */
  void publicaVelocidad(const geometry_msgs::Twist& robotVel)
  {
    _robot_info.velocidad().linear(robotVel.linear.x);
    _robot_info.velocidad().angular(robotVel.angular.z);
    double magnitud = sqrt(pow(robotVel.linear.x, 2) + pow(robotVel.angular.z, 2));
    tf2::Quaternion q;
    double angle = atan2(robotVel.angular.z, robotVel.linear.x);
    q.setRPY(0, 0, angle);  // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();
    //ROS_INFO_STREAM("x: " << robotVel.linear.x << "; z: " << robotVel.angular.z << "; Angle: "  << angle << "; Quaternion: " << q);

    velocity_mark.scale.x = magnitud;
    velocity_mark.pose.orientation = tf2::toMsg(q);
    marker_pub.publish(velocity_mark);
  }

  void leePosicion(const nav_msgs::Odometry& odom)
  {
    _robot_info.extraePosicion(odom);
    CoordsCelda coords = calculaCelda(_robot_info.posicion().x(), _robot_info.posicion().y());
    if (_colorPrevio != -1)
    {
      mapa_marcas.data[_celdaPrevia.i*WIDTH+_celdaPrevia.j] = _colorPrevio;
    }
    _colorPrevio = mapa_marcas.data[coords.i*WIDTH+coords.j];
    _celdaPrevia = coords;
    mapa_marcas.data[coords.i*WIDTH+coords.j] = 20;

    _robot_info.extraePosicion(odom);
  }

  void publicate()
  {
    grid_pub.publish(mapa);
    grid_pub_marcas.publish(mapa_marcas);
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

  /** Pasa de las coordenadas según el odométro a los número de celda. */
  CoordsCelda calculaCelda(double dx, double dy)
  {
    CoordsCelda coords;
    coords.i = (dy - mapa.info.origin.position.y) / RESOLUTION;
    coords.j = (dx - mapa.info.origin.position.x) / RESOLUTION;
    return coords;
  }

  /** Agrega los obstáculos al mapa */
  void llenaMapa()
  {
    // %Tag(MAP_INIT)%

    // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
    mapa.header.frame_id = "/odom";
    mapa.header.stamp = ros::Time::now();   // No caduca

    mapa.info.resolution = RESOLUTION;      // [m/cell]
    mapa.info.width = WIDTH;                // [cells]
    mapa.info.height = HEIGHT;              // [cells]
    mapa.info.origin.position.x = -RESOLUTION * WIDTH / 2.0;
    mapa.info.origin.position.y = -RESOLUTION * HEIGHT / 2.0;
    mapa.info.origin.position.z = 0;
    mapa.info.origin.orientation.x = 0.0;
    mapa.info.origin.orientation.y = 0.0;
    mapa.info.origin.orientation.z = 0.0;
    mapa.info.origin.orientation.w = 1.0;


    /// --- Mapa decorativo para información de depurado

    mapa_marcas.header.frame_id = "/odom";
    mapa_marcas.header.stamp = ros::Time::now();   // No caduca

    mapa_marcas.info.resolution = RESOLUTION;      // [m/cell]
    mapa_marcas.info.width = WIDTH;                // [cells]
    mapa_marcas.info.height = HEIGHT;              // [cells]
    mapa_marcas.info.origin.position.x = -RESOLUTION * WIDTH / 2.0;
    mapa_marcas.info.origin.position.y = -RESOLUTION * HEIGHT / 2.0;
    mapa_marcas.info.origin.position.z = 0.01;
    mapa_marcas.info.origin.orientation.x = 0.0;
    mapa_marcas.info.origin.orientation.y = 0.0;
    mapa_marcas.info.origin.orientation.z = 0.0;
    mapa_marcas.info.origin.orientation.w = 1.0;

    /// ---


    //int8[] &_data = &mapa.data
    int size = WIDTH * HEIGHT;
    char* data = new char[size];
    for(int i = 0; i < size; i++) {
      data[i] = 0;
    }

    /// --- dec
    mapa_marcas.data = std::vector<int8_t>(data, data + size);
    /// ---

    //data[0] = 50;                               // El origen está en la esquina inferior izquierda.
    fillRectangle(data, 0, 1, 0, WIDTH-1, OCUPADA);   // Renglón 0. Las columnas van de 0 a WIDTH-1.  Los renglones corren sobre el eje Y.
    fillRectangle(data, 0, 0, HEIGHT-1, 0, OCUPADA);  // Columna 0. Los renglones va de 0 a HEIGHT-1.  Las columnas corren sobre el eje X.
    fillRectangle(data, HEIGHT-1, 1, HEIGHT-1, WIDTH-1, OCUPADA);
    fillRectangle(data, 1, WIDTH-1, HEIGHT-1, WIDTH-1, OCUPADA);
    fillRectangle(data, 5, 1, 6, 11, OCUPADA);          // Mesa izq1
    fillRectangle(data, 11, 1, 13, 11, OCUPADA);        // Mesa izq2
    fillRectangle(data, 18, 1, 20, 11, OCUPADA);        // Mesa izq3
    fillRectangle(data, 5, 17, 6, 22, OCUPADA);         // Mesa der1
    fillRectangle(data, 11, 17, 13, 22, OCUPADA);       // Mesa der2
    fillRectangle(data, 18, 17, 20, 22, OCUPADA);       // Mesa der3

    mapa.data = std::vector<int8_t>(data, data + size);


    // %EndTag(MAP_INIT)%
  }


  /** Devuelve el índice del arreglo unidimensional donde está el mapa 2D. */
  int mInd(int i, int j)
  {
    return i*WIDTH+j;
  }


  /** Sets the cells between [i1,j1] and [i2,j2] inclusive as occupied with probability value. */
  void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value)
  {
    for(int i = i1; i <= i2; i++)
    {
      for(int j = j1; j <= j2; j++)
      {
        data[mInd(i, j)] = value;
      }
    }
  }


  /**
     * Acciones a realizar cuando se ha detectado una colisión.
     * @param x coordenada x del origen del rayo
     * @param y coordenada y del origen del rayo
     * @param xn
     * @param yn
     * @return
     */
  double colision(double x, double y, double xn, double yn)
  {
    //float[] colision = {(float)xn, (float)yn};
    //colisiones.add(colision);
    return sqrt(pow(xn - x, 2) + pow(yn - y, 2));
  }

  /**
   * Lanza un rayo a partir de las coordenadas (<code>x</code>,<code>y</code>)
   * en dirección <code>angulo</code> y devuelve la distancia al obstáculo más
   * cercano o a un muro en el mapa.
   * @param x coordenada horizontal, de izquierda a derecha.
   * @param y coordenada vertical, de arriba a abajo.
   * @param ángulo dirección en la que se extiende el rayo en radianes.
   * @param limpia borra las línea usadas para calcular las distancias.
   * @return distancia
   */
  double distanciaAColision(double x, double y, double angulo, bool limpia) {
        /*if (limpia) {
            origen[0] = x;
            origen[1] = y;
            ilumina.clear();
            colisiones.clear();
        }*/
        angulo = anguloEnRango(angulo);
        double m = tan(angulo);
        double b = - y - m * x;  // y está invertida
        CoordsCelda ij = calculaCelda(x, y);
        int i = ij.i, j = ij.j;
        double xn, yn;

        if (angulo > 0) {
            if (angulo < M_PI/2) {
                // Primer cuadrante
                int l = i, k = j;
                // arriba y a la derecha
                while(k < WIDTH && l >= 0) {
                    //int indx[2] = {l, k};
                    //ilumina.add(indx);

                    xn = (k + 1) * RESOLUTION;
                    yn = y - m * (xn - x);
                    if (yn < (l + 1) * RESOLUTION && yn > l * RESOLUTION) {
                        k++; // ve a la derecha
                        if (k == WIDTH || mapa.data[mInd(l, k)] == OCUPADA) {
                            //System.out.println("Caso 1");
                            return colision(x, y, xn, yn);
                        }
                    } else if (yn == l * RESOLUTION) {
                        k++; // a la derecha
                        l--; // arriba
                        if (l < 0 || k == WIDTH || mapa.data[mInd(l, k)] == OCUPADA) {
                            //System.out.println("Caso 2");
                            return colision(x, y, xn, yn);
                        }
                    } else {
                        l--; // arriba

                        yn = (l + 1) * RESOLUTION;
                        xn = (-yn - b)/m;
                        if (l < 0 || mapa.data[mInd(l, k)] == OCUPADA) {
                            //System.out.println("Caso 3");
                            return colision(x, y, xn, yn);
                        }
                    }
                }
            } else if (angulo <= M_PI) {
                // Segundo cuadrante
                int l = i, k = j;
                // arriba y a la izquierda
                while(k >= 0 && l >= 0) {
                    //int[] indx = {l, k};
                    //ilumina.add(indx);
                    yn = l * RESOLUTION;
                    xn = (-yn - b)/m;
                    if (xn > k * RESOLUTION && xn < (k + 1) * RESOLUTION) {
                        l--; // ve arriba
                        if (l < 0 || mapa.data[mInd(l, k)] == OCUPADA) {
                            return colision(x, y, xn, yn);
                        }
                    } else if (xn == (k + 1) * RESOLUTION) {
                        k--; // a la izquierda
                        l--; // arriba
                        if (l < 0 || k < 0 || mapa.data[mInd(l, k)] == OCUPADA) {
                            return colision(x, y, xn, yn);
                        }
                    } else {
                        k--; // a la izquierda
                        xn = (k + 1) * RESOLUTION;
                            yn = y - m * (xn - x);
                        if (k < 0 || mapa.data[mInd(l, k)] == OCUPADA) {
                            return colision(x, y, xn, yn);
                        }
                    }
                }
            }
        } else {
            if (angulo > -M_PI/2) {
                // Cuarto cuadrante
                int l = i, k = j;
                // abajo y a la derecha
                while(k < WIDTH && l < HEIGHT) {
                    //int[] indx = {l, k};
                    //ilumina.add(indx);

                    xn = (k + 1) * RESOLUTION;
                    yn = y - m * (xn - x);
                    if (yn < (l + 1) * RESOLUTION && yn > l * RESOLUTION) {
                        k++; // ve a la derecha
                        if (k == WIDTH || mapa.data[mInd(l, k)] == OCUPADA) {
                            //System.out.println("Caso 1");
                            return colision(x, y, xn, yn);
                        }
                    } else if (yn == (l + 1) * RESOLUTION) {
                        k++; // a la derecha
                        l++; // abajo
                        if (l == HEIGHT || k == WIDTH && mapa.data[mInd(l, k)] == OCUPADA) {
                            //System.out.println("Caso 2");
                            return colision(x, y, xn, yn);
                        }
                    } else {
                        l++; // abajo
                        if (l == HEIGHT || mapa.data[mInd(l, k)] == OCUPADA) {
                            yn = l * RESOLUTION;
                            xn = (-yn - b)/m;
                            //System.out.println("Caso 3");
                            return colision(x, y, xn, yn);
                        }
                    }
                }
            } else if (angulo > -M_PI) {
                // Tercer cuadrante
                int l = i, k = j;
                // abajo y a la izquierda
                while(k >= 0 && l < HEIGHT) {
                    //int[] indx = {l, k};
                    //ilumina.add(indx);
                    yn = (l + 1) * RESOLUTION;
                    xn = (-yn - b)/m;
                    if (xn > k * RESOLUTION && xn < (k + 1) * RESOLUTION) {
                        l++; // ve abajo
                        if (l == HEIGHT || mapa.data[mInd(l, k)] == OCUPADA) {
                            return colision(x, y, xn, yn);
                        }
                    } else if (xn == (k + 1) * RESOLUTION) {
                        k--; // a la izquierda
                        l++; // ve abajo
                        if (l == HEIGHT || k < 0 || mapa.data[mInd(l, k)] == OCUPADA) {
                            return colision(x, y, xn, yn);
                        }
                    } else {
                        k--; // a la izquierda
                        if (k < 0 || mapa.data[mInd(l, k)] == OCUPADA) {
                            xn = (k + 1) * RESOLUTION;
                            yn = y - m * (xn - x);

                            return colision(x, y, xn, yn);
                        }
                    }
                }
            }
        }
        return -1;
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
  ros::Subscriber sub_odom = n.subscribe("/odom", 3, &Mapa::leePosicion, &mapa);
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
