package main.controllers.testControllers.rrt;

import com.cyberbotics.webots.controller.*;
import main.controllers.testControllers.node.Node;
import main.java.kak3.shp.algorithms.RRT;
import main.java.kak3.shp.algorithms.samplers.RandomSampler;
import main.java.kak3.shp.framework.ControlParameters;
import main.java.kak3.shp.framework.Position;
import main.java.kak3.shp.framework.PositionAndHeading;
import main.java.kak3.shp.framework.distance.DistanceCalculator;
import main.java.kak3.shp.framework.distance.Euclidean;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.*;

public class TestRRT {

  private static final double DEFAULT_SENSOR_DISTANCE = 4;//meters
  private static final int DEFAULT_MUPDATE_FREQUENCY = 1;
  private static final double DEFAULT_GOAL_RADIUS = 1; // meters.
  private static final double DEFAULT_OBSTACLE_POTENTIAL_RATIO = 2.5;
  private static final double[] DEFAULT_CONTROL_PARAMETERS = { 1 , 1 };
  private static final double[] DEFAULT_GOAL_PARAMETERS = { 1 , 1 }; // goal {x, y}
  private static final double DEFAULT_MAX_WHEEL_VALUE = 3;
  private static final double DEFAULT_MAX_SPEED = 4;
  private static final double DEFAULT_AXLE_LENGTH = 2;
  private static final int DEFAULT_MAX_ITERATIONS = 100;

  //Sampling
  private final int numSamples = 5;
  private final double  stepSize = 3; // meters.
  private final double minVal = 0; // meters. -- Try a negative number and check the results
  private final double maxVal = 2.5; // meters.

  //Annealing values
  private static final double energyCoefficient = 0.5;
  private static double temperature = 100.0;
  private static final double coolingRate = 0.01;

  //Devices
  private final Motor[] motors = new Motor[4];
  private final GPS gps;
  private final Compass compass;
  private final Emitter emitter;

  //Positions
  Position targetPosition, currentPosition, nextPosition;
  private List<ControlParameters> bestNeighbors = new LinkedList<>();
  private boolean isRunning = false;

  //Directions
  private Double currentHeading = null;
  private double turnAngle = 0;

  //Other
  private DistanceCalculator distanceCalculator;

  //Obstacles
  private LidarPoint[] obstacleCoordinates;

  //Algorithms
  private final Random random = new Random();
  private final Euclidean euclidean = new Euclidean();
  private RandomSampler randomSampler = new RandomSampler(numSamples, stepSize, minVal, maxVal);
  RRT rrt = new RRT(randomSampler, minVal, maxVal);


  //Control Parameters
  private ControlParameters currentParameters, nextParameters;

  //Nodes
  private Node currentParamNode, nextParameterNode;
  private List<Node> currentTree = new LinkedList<>();
  private List<Node> bestTree = new LinkedList<>();

  //Constructor of RRT Controller
  public TestRRT(GPS gps, Compass compass, Emitter emitter,
                       Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {


    this.motors[0] = frontLeft;
    this.motors[1] = frontRight;
    this.motors[2] = backLeft;
    this.motors[3] = backRight;

    this.gps = gps;
    this.compass = compass;
    this.emitter = emitter;
  }

  public static void main(String[] args){

    // Create the Robot instance.
    Robot robot = new DifferentialWheels();

    // Get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Get references to, and enable, all required devices on the robot.
    Lidar lidar = robot.getLidar("lms291");
    lidar.enable(timeStep);
    lidar.enablePointCloud();

    GPS gps = robot.getGPS("gps");
    gps.enable(timeStep);

    Emitter emitter = robot.getEmitter("emitter");
    emitter.setChannel(0);

    Compass compass = robot.getCompass("compass");
    compass.enable(timeStep);

    Motor frontLeft = robot.getMotor("front_left_wheel");
    frontLeft.setPosition(Double.POSITIVE_INFINITY);
    Motor frontRight = robot.getMotor("front_right_wheel");
    frontRight.setPosition(Double.POSITIVE_INFINITY);
    Motor backLeft = robot.getMotor("back_left_wheel");
    backLeft.setPosition(Double.POSITIVE_INFINITY);
    Motor backRight = robot.getMotor("back_right_wheel");
    backRight.setPosition(Double.POSITIVE_INFINITY);

    //SHPController controller = new SHPController(ns, gps, compass, frontLeft, frontRight, backLeft, backRight, emitter);
    TestRRT rrtController = new TestRRT(gps, compass, emitter, frontLeft,
        frontRight, backLeft, backRight);
    System.out.println("current param  : " + rrtController.toString());

    rrtController.initAlgorithm();
    robot.step(timeStep);
    rrtController.updateSensorReadings(lidar.getPointCloud());
    rrtController.isRunning = true;

    //Iteration Count for frequency
    int iterationCount = 0;

    while (robot.step(timeStep*100) != -1) {

      while (iterationCount <DEFAULT_MAX_ITERATIONS){
        //Update Turning Angles
        rrtController.updateTurnAngles();

        //Update Sensor Readings
        rrtController.updateSensorReadings(lidar.getPointCloud());

        //Check if goal is reached
        if (rrtController.isGoalReached() || !rrtController.isRunning) {
          rrtController.terminate();

          if (!rrtController.isRunning) {
            System.out.println("Stopped");
            System.out.println("Controller Stopped");
            return;
          }

          else if(rrtController.isGoalReached()) {
            System.out.println("Goal Reached Successfully");
            return;
          }
        }

        else if(!rrtController.isGoalReached() && rrtController.isRunning ) {
          //Iteratively Simulate the algorithm
          rrtController.buildTree();
          rrtController.driveToGoal();
          System.out.println("Building Trees...");

        }
      }
    }
  }

  private void driveToGoal() {
    for (Node node: bestTree){
      setSpeed(node.getX(), node.getY());
      System.out.println("Setting speeds and driving...");
      //Might need to add a move sequence later
    }
  }

  private void buildTree() {
    for(Node node: currentTree){
      for (Node child: node.getChildren()){
        child.setChildren(getChildren(child));
        child.setCost(getChildCost(child.getPosition()));
        bestTree.add(getBestNode(child.getChildren()));

      }
    }
  }

  private void initAlgorithm(){
    setSpeed(0, 0);
    currentPosition = getCurrentPosition();
    targetPosition = new Position(DEFAULT_GOAL_PARAMETERS[0], DEFAULT_GOAL_PARAMETERS[1]);

    currentParameters = new ControlParameters(DEFAULT_CONTROL_PARAMETERS);
    initRRT();
    updateTurnAngles();

  }

  private void initRRT() {
    //Generate initial tree of the space
    currentParamNode = new Node(getCurrentPosition(), 0, null, null);

    currentParamNode.setChildren(getChildren(currentParamNode));
    currentTree.add(currentParamNode);
    bestTree.add(currentParamNode);
    System.out.println("Tree size : " + currentTree.size());

    if(currentTree.size() == 1){
      System.out.println("Start Node : " + currentTree.get(0));
      System.out.println("Start Node Parent : " + currentTree.get(0).getParent().toString());
      System.out.println("Start Node Children : " + currentTree.get(0).getChildren().size());

    }
  }

  private boolean isGoalReached(){
    if(getDistanceToGoal() < DEFAULT_GOAL_RADIUS){
        return true;
    } else {
      return false;
    }
  }

  private double getChildCost(Position childPosition) {
    double cost = 0;
    for (LidarPoint obstacle:
        obstacleCoordinates) {
      Position obstaclePosition = new Position(obstacle.getX(), obstacle.getZ());

      double distanceToNeighbor = euclidean.getDistanceBetween(childPosition, obstaclePosition);

      cost += distanceToNeighbor;
    }
    return cost;
  }

  private List<Node> getChildren(Node parameterNode){

    List<ControlParameters> samples = new ArrayList<>(
        randomSampler.getSamplesFrom(parameterNode.getNodeParameters()));

    List<Node> possibleChildren = new ArrayList<>(samples.size());

    for (int i = 0; i < samples.size() ; i++) {
      double[] vals = samples.get(i).getArray();
      Position childPosition = new Position(vals[0], vals[1]);

      possibleChildren.add(new Node(childPosition, getChildCost(childPosition),parameterNode, null));
    }
    return possibleChildren;
  }

  private Position getCurrentPosition() {
    //return the current location on the gps
    return new Position(this.gps.getValues()[0], this.gps.getValues()[2]);
  }

  private double getDistanceToGoal(){
    double[] gpsValues = gps.getValues();
    for (int i = 0 ; i < gpsValues.length ; i++){
      System.out.println ("gpsValues["+i+"]: "+gpsValues[i] );
    System.out.println  ( "target " + targetPosition ) ;
    }

    System.out.println  ( "Distance " + getDistBetween(new PositionAndHeading(new Position(gpsValues[0], gpsValues[2]), 0), targetPosition) ) ;

    return getDistBetween(new PositionAndHeading(new Position(gpsValues[0], gpsValues[2]), 0), targetPosition);
  }

  private double getDistBetween(PositionAndHeading origin, Position destination) {
    Position x  = new Position(0,0);
    PositionAndHeading absolute = new PositionAndHeading(x, 0);
    // absolute = relativeToAbsolute(absolute);

    return distanceCalculator.getDistanceBetween(absolute, origin.position, destination);
  }

  private void sendRenderMessage(String msg) {
    emitter.send((msg + "\0").getBytes());
  }

  private Node getBestNode(List<Node> children) {
    Node bestNode = children.get(0);
    for (Node node:
        children){
      if(bestNode.getCost() > node.getCost()){
        bestNode = node;
      }
    }
    System.out.println("Getting all best nodes");
    return bestNode;
  }

  /**
   * Calculates a position and heading given the current position and heading, the wheel speeds, and the time step,
   * using forwards kinematics equations for a differential drive robot, as described here
   * https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
   * @param origin the starting point of the robot.
   * @param leftWheel the left wheel speed.
   * @param rightWheel the right wheel speed.
   * @param timeStep the time step to move for in seconds.
   * @return the new estimated position and heading.
   */
  private static PositionAndHeading getForwardKinematics(PositionAndHeading origin, double leftWheel,
                                                               double rightWheel, double timeStep) {
    double R;

    leftWheel = (leftWheel / DEFAULT_MAX_WHEEL_VALUE) * DEFAULT_MAX_SPEED;
    rightWheel = (rightWheel / DEFAULT_MAX_WHEEL_VALUE) * DEFAULT_MAX_WHEEL_VALUE;

    if (rightWheel == leftWheel) {
      return new PositionAndHeading(
          new Position(origin.position.x + ((leftWheel) * timeStep) * Math.sin(origin.heading),
              (origin.position.y + ((leftWheel) * timeStep) * Math.cos(origin.heading))),
          origin.heading);
    }
    else {
      R = (DEFAULT_AXLE_LENGTH * (leftWheel + rightWheel)) / ((rightWheel - leftWheel) * 2d);
    }

    double w = (rightWheel - leftWheel) / DEFAULT_AXLE_LENGTH;

    double iccx = origin.position.x - R * Math.sin(origin.heading);
    double iccy = origin.position.y + R * Math.cos(origin.heading);

    double[][] matrixData1 = {
        {Math.cos(w * timeStep), -Math.sin(w * timeStep), 0},
        {Math.sin(w * timeStep), Math.cos(w * timeStep), 0},
        {0, 0, 1}
    };
    RealMatrix matrix1 = MatrixUtils.createRealMatrix(matrixData1);

    double[][] matrixData2 = {
        {origin.position.x - iccx},
        {origin.position.y - iccy},
        {origin.heading}};
    RealMatrix matrix2 = MatrixUtils.createRealMatrix(matrixData2);

    double[][] matrixData3 = {{iccx}, {iccy}, {w * timeStep}};
    RealMatrix matrix3 = MatrixUtils.createRealMatrix(matrixData3);

    RealMatrix result = matrix1.multiply(matrix2).add(matrix3);
    double[] resultArray = result.getColumn(0);

    return new PositionAndHeading(new Position(resultArray[1], resultArray[0]), resultArray[2]);
  }

  /**
   * Sets the wheel speeds for the robot. Note that as we're simulating a differential drive robot, we set both left
   * and both right speeds to be the same.
   * @param left the left wheel speeds to set.
   * @param right the right wheel speeds to set.
   */
  private void setSpeed(double left, double right) {
    motors[0].setVelocity(left);
    motors[1].setVelocity(right);
    motors[2].setVelocity(left);
    motors[3].setVelocity(right);
  }

  private synchronized void updateSensorReadings(LidarPoint[] sensorImage) {
    obstacleCoordinates = sensorImage;
    System.out.println ("image " + sensorImage[0]);
  }

  private void updateTurnAngles() {
    double[] compassValues = compass.getValues();
    double direction = Math.PI + Math.atan2(compassValues[2], compassValues[0]);
    if (currentHeading != null) {
      double angle = currentHeading - direction;
      if (angle > Math.PI) {
        angle = -1 * (2*Math.PI - angle);
      }
      else if (angle < -Math.PI) {
        angle = 2*Math.PI + angle;
      }
      turnAngle += angle;
    }
    currentHeading = direction;
  }

  private void terminate() {
    setSpeed(0, 0);
    isRunning = false;
    System.out.println("Controller Terminated");
    //Maybe add some stats
    return;
  }
}
