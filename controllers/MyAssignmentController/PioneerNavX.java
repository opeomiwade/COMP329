// PioneerNavX.java
/*
 * PioneerNavigation Class Definition
 * Date: 18th Oct 2022
 * Description: Simple Navigation Class support for 2022 Assignment
 * Author: Terry Payne (trp@liv.ac.uk)
 */

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;

public class PioneerNavX {

  public static enum MoveState {
    STOP,
    FORWARD,
    ARC,
    WANDER,
    NAVIGATE,
    ROTATE};

  private Supervisor robot;       // reference to the robot
  private Node robot_node;        // reference to the robot node
  private Pose robot_pose;        // the robots believed pose, based on real location
  private Motor left_motor;
  private Motor right_motor;
  private MoveState state;

  private double velocity;
  boolean stopRotation = false;
  boolean visited = false;
  boolean leftArc = false;
  boolean stopWandering = false;
  double icr_angle = 0.0;
  boolean finalWaypoint = false;
  double destinationTheta = 0.0;
  double icr_r = 0.5;
  boolean pathCompleted = false;

  private final double WHEEL_RADIUS = 0.0957; // in meters - found using CONFIGURE
  private final double AXEL_LENGTH = 0.323;   // in meters - found using CONFIGURE


  // ==================================================================================
  // Constructor
  // ==================================================================================
  public PioneerNavX(Supervisor robot) {
    this.robot = robot;                       // reference to the robot
    this.robot_node = this.robot.getSelf();   // reference to the robot node
    this.robot_pose = this.get_real_pose();   // the robots believed pose, based on real location
    this.state = MoveState.STOP;

    // enable motors
    this.left_motor = robot.getMotor("left wheel");
    this.right_motor = robot.getMotor("right wheel");
    this.left_motor.setPosition(Double.POSITIVE_INFINITY);
    this.right_motor.setPosition(Double.POSITIVE_INFINITY);

    // Initialise motor velocity
    this.left_motor.setVelocity(0.0);
    this.right_motor.setVelocity(0.0);
  }

  public Pose get_real_pose() {
    if (this.robot_node == null)
      return new Pose(0,0,0);

    double[] realPos = robot_node.getPosition();
    double[] rot = this.robot_node.getOrientation(); // 3x3 Rotation matrix as vector of length 9
    double theta1 = Math.atan2(-rot[0], rot[3]);
    double halfPi = Math.PI/2;
    double theta2 = theta1 + halfPi;
    if (theta1 > halfPi)
        theta2 = -(3*halfPi)+theta1;

    return new Pose(realPos[0], realPos[1], theta2);
  }

  // The following code is based on the avoid obstacle code supplied by the Webots
  // platform for the ePuck and allows the robot to wander randomly around the arena
  public void wander(PioneerProxSensors prox_sensors, double robot_linearvelocity,Pose pose) {

    double leftVel, rightVel;
    double wheel_av = (robot_linearvelocity/this.WHEEL_RADIUS);
    double left_vel = wheel_av;
    double right_vel = wheel_av;

    // detect obstacles
    boolean right_obstacle =
        prox_sensors.get_value(4) < 0.30 ||
        prox_sensors.get_value(5) < 0.25 ||
        prox_sensors.get_value(6) < 0.20 ||
        prox_sensors.get_value(7) < 0.15;
    boolean left_obstacle =
        prox_sensors.get_value(0) < 0.15 ||
        prox_sensors.get_value(1) < 0.20 ||
        prox_sensors.get_value(2) < 0.25 ||
        prox_sensors.get_value(3) < 0.30;

    if (left_obstacle){

      right_vel = -left_vel;

    }

    else if (right_obstacle){

      left_vel = -right_vel;
    }



    //no obstacle
    this.left_motor.setVelocity(left_vel);
    this.right_motor.setVelocity(right_vel);
    this.state = MoveState.WANDER;
  }

  public int forward(double target_dist, double robot_linearvelocity) {
    double wheel_av = (robot_linearvelocity/this.WHEEL_RADIUS);
    double target_time = target_dist/robot_linearvelocity;

    this.left_motor.setVelocity(wheel_av);
    this.right_motor.setVelocity(wheel_av);
    this.state = MoveState.FORWARD;

    // return target_time as millisecs
    return (int) (1000.0*target_time);
  }

  public int arc(double icr_angle, double icr_r, double icr_omega) {
    double target_time = icr_angle / icr_omega;

    // Calculate each wheel velocity around ICR
    double vl = icr_omega * (icr_r - (this.AXEL_LENGTH / 2));
    double vr = icr_omega * (icr_r + (this.AXEL_LENGTH / 2));

    double leftwheel_av = (vl/this.WHEEL_RADIUS);
    double rightwheel_av = (vr/this.WHEEL_RADIUS);

    this.left_motor.setVelocity(leftwheel_av);
    this.right_motor.setVelocity(rightwheel_av);
    this.state = MoveState.ARC;

    // return target_time as millisecs
    return (int) (1000.0*target_time);
  }


  public void stop() {
    this.left_motor.setVelocity(0.0);
    this.right_motor.setVelocity(0.0);
    this.state = MoveState.STOP;
  }


  /////////////////////////////////////////////////////////////////////


  /** checks if the robot is approaching a current pose so the movestate can be changed appropriately. Calculates the distance from the currentpose to the destination
   pose using the euclidean distance formula.If the distance is approaching a value less than 0.1 teh robot is close to the destination pose.
  **/
  public boolean atDestination(Pose currentPose , double destinationX , double destinationY){
    boolean approaching = false;
    double intermediate = Math.pow((currentPose.getX() - destinationX), 2) + Math.pow((currentPose.getY() - destinationY), 2);
    double distance = Math.sqrt(intermediate);
    if(distance < 0.1){
      approaching = true;
    }

    else{
      approaching = false;
    }
    return approaching;
  }

  /** calculate icr angle for rotation using odometry and then checks if the difference between rot2 and destinationTheta is approaching
  zero return this value otherwise return zero.
  **/

  public double calculateIcrAngleRotate(Pose currentPose , double destinationTheta,double destinationX,double destinationY){
    double rot1 = Math.atan2(destinationY - currentPose.getY() , destinationX - currentPose.getX()) - currentPose.getTheta();
    double rot2 = destinationTheta - currentPose.getTheta() - rot1;
    if((rot2 - destinationTheta) < 0.02){
      return rot2;
    }
    return 0;
  }

  //helper method to stop the arcing movement when the robot reaches the desired orientation.
  public boolean atTheta(double destinationTheta , double currentTheta){
    boolean atTheta = false;
    if(Math.abs(destinationTheta - currentTheta) <= 0.03){
      atTheta = true;
    }

    return atTheta;

  }


  /** state machine to help the robot navigate around the arena.When the robot is approaching a desired pose the movestate is changed appropriately, boolean values are set or reset,
  varables are initialised such as the icr radius, the destination theta, etc.
  **/
  public void stateMachine(Pose currentPose, double linearVelocity){
    //calculate av
    double wheel_av = (linearVelocity * 0.2/this.WHEEL_RADIUS);
    double left_vel = wheel_av;
    double right_vel = wheel_av;
    double x = currentPose.getX();
    double y = currentPose.getY();

    //move robot
    this.right_motor.setVelocity(wheel_av);
    this.left_motor.setVelocity(wheel_av);

    //waypoint 1
    if(atDestination(currentPose, 0.58, 2.00 )){
      double rot2 = calculateIcrAngleRotate(currentPose , -1.57 , 0.58 , 2.00);
      if ( rot2 != 0){
        icr_angle = rot2;
      }
      this.state = MoveState.ROTATE;

    }

    //waypoint 2
    else if(atDestination(currentPose,0.392,-0.025)){
      stopRotation = false; 
      destinationTheta = -0.016;
      leftArc = true;
      visited = false;
      this.state = MoveState.ARC;

    }



    //waypoint 3
    else if(atDestination(currentPose, 2.002,-0.491) && visited == false){
      stopRotation = false; //helps to terminate rotation of the robot when teh target time has been exceeded.
      double rot2 = calculateIcrAngleRotate(currentPose, -1.49, 2.002, -0.491 );
      if (rot2 != 0){
        icr_angle = rot2;
      }
      this.state = MoveState.ROTATE;
    }



    //waypoint 4, wanders round the triangle in the mao to get good coverage.

    else if(atDestination(currentPose, 1.880 , -0.873)){
      this.state = MoveState.WANDER;
    }


    // //waypoint 5
    else if (atDestination(currentPose , -1.437 , -0.615) && stopWandering == true){ // start arcing motion earlier
      stopRotation = false;
      leftArc = true;
      destinationTheta = 1.57;
      this.state = MoveState.ARC;
    }

    // //waypoint 6

    else if(atDestination(currentPose , -1.006 , -0.118)){
      stopRotation = false;
      leftArc = true;
      destinationTheta = -3.12;
      this.state = MoveState.ARC;
    }



    //Waypoint 7
    else if(atDestination(currentPose, -1.595, 0.426)){
      stopRotation = false;
      destinationTheta = 0.049;
      icr_r = 0.5;
      this.state = MoveState.ARC;
    }


    //Waypoint 8
    else if(atDestination(currentPose , -0.705 , 1.361)){
      stopRotation = false;
      icr_r = 0.5;
      destinationTheta = -1.509;
      this.state = MoveState.ARC;
    }


    //Waypoint 9
    else if(atDestination(currentPose , -0.213 , 0.358)){
      stopRotation = false;
      leftArc = true;
      icr_r = 0.4;
      destinationTheta = 1.646;
      finalWaypoint = true; // indidcates robot is appraoching finalwaypoint next
      this.state = MoveState.ARC;
    }

    //Waypoint 10
    else if(atDestination(currentPose, 0.487, 1.556) && finalWaypoint == true){
      stopRotation = false;
      icr_r = 0.5;
      destinationTheta = 0.00;
      this.state = MoveState.ARC;
    }

    else if(atDestination(currentPose, 2.257 , 1.954)){
      pathCompleted = true;// indicates the path has been completed so the rbot can stop at the top right.
      this.state = MoveState.STOP;
    }
    
    //simulation finishes a couple seconds over three minutes.

  }


  public MoveState getState() {
    return this.state;
  }
}
