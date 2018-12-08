package main.controllers.testControllers.potBug;

import com.cyberbotics.webots.controller.*;
import main.controllers.testControllers.node.Node;
import main.java.kak3.shp.algorithms.samplers.RandomSampler;
import main.java.kak3.shp.framework.ControlParameters;
import main.java.kak3.shp.framework.Position;
import main.java.kak3.shp.framework.PositionAndHeading;
import main.java.kak3.shp.framework.distance.DistanceCalculator;
import main.java.kak3.shp.framework.distance.Euclidean;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;


public class TestPotBug{

  //Ratios and Ranges
  private static final double DEFAULT_SENSOR_RANGE = 10.0; // meters
  private static final double DEFAULT_ROBOT_RADIUS = 2.0; //meters
  private static final double DEFAULT_GOAL_RADIUS = 1.0; //meters

  private static final double DEFAULT_BAND_A = 10.0; //
  private static final double DEFAULT_BAND_B = 2.0; //
  private static final double DEFAULT_BAND_C = 1.0; //

  private static final double DEFAULT_OBSTACLE_COEFFICIENT = 0.001;
  private static final double DEFAULT_GOAL_COEFFICIENT = 0.002;

  private static final double DEFAULT_MOTOR_VELOCITY = 0.2;
  private static final double DEFAULT_APROACH_RATIO = 0.25;
  private static final double DEFAULT_ENGAGE_RATIO = 0.2;
  private static final int DEFAULT_APROACH_SAMPLES = 3;
  private static final int DEFAULT_MAX_ITERATION_COUNT = 1000;
  private static double DEFAULT_LOOKAHEAD_TIME;

  //Sampling
  private static final int DEFAULT_SAMPLE_SIZE = 1; //might change later
  private static final int DEFAULT_STEP_SIZE = 2; //might change later
  private static final double DEFAULT_MIN_SAMPLE_VAL = 0; // meters. -- Try a negative number and check the results
  private static final double DEFAULT_MAX_SAMPLE_VAL = 2.5; // meters.

  //Default goal positions
  private static final double DEFAULT_GOAL_POSITION_X = 20.0;
  private static final double DEFAULT_GOAL_POSITION_Y = 20.0;

  //Max Potential Limits
  private static final double DEFAULT_MAX_APPROACH_POTENTIAL = 20; //Made up - will be tested for
  private static final double DEFAULT_MAX_ENGAGE_POTENTIAL = 50; //

  //Robot's devices
  private final Motor[] motors = new Motor[4];
  private LidarPoint[] obstaclePoints;
  private final GPS gps;
  private final Compass compass;
  private final Emitter emitter;

  //Positions
  private Position goalPosition, startPosition, currentPosition;

  //
  private boolean isRunning = false;
  private double hitPointDistance;
  //Nodes
  Node currentNode, nextNode, startNode;
  private List<Node> currentTree = new LinkedList<>();
  private List<Node> bestTree = new LinkedList<>();

  //Math
  private DistanceCalculator distanceCalculator;
  private Euclidean euclideanCalculator;

  //Algorithms
  private Random random = new Random();
  private RandomSampler randomSampler = new RandomSampler(
      DEFAULT_SAMPLE_SIZE, DEFAULT_STEP_SIZE, DEFAULT_MIN_SAMPLE_VAL, DEFAULT_MAX_SAMPLE_VAL);

  public TestPotBug(GPS gps, Compass compass, Emitter emitter, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {

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

    TestPotBug potBugController = new TestPotBug(gps, compass, emitter, frontLeft, frontRight, backLeft, backRight);
    System.out.println("current param  : " + potBugController.toString());

    //Initialise Start and Goal Coordinates
    potBugController.initPositions();
    potBugController.initAlgorithm();
    robot.step(timeStep);
    potBugController.updateObstacleReadings(lidar.getPointCloud());
    potBugController.isRunning = true;

    //Iteration Count
    int itCount = 0;

    while (robot.step(timeStep*100) != -1) {
      //Check max iteration
      while (itCount<DEFAULT_MAX_ITERATION_COUNT){
        //Set the sensor range and robot radius
        double sensorRange = DEFAULT_SENSOR_RANGE;
        double robotRadius = DEFAULT_ROBOT_RADIUS;

        //Update Sensor Information
        potBugController.updateObstacleReadings(lidar.getPointCloud());

        //Check if goal is reached and terminate as appropriate
        if (potBugController.isGoalReached()){
          potBugController.isRunning = false;
          potBugController.setRobotSpeed(0,0);
          potBugController.terminatePotBug();
          System.out.println("Goal Reached Successfully!");
        }

        while(!potBugController.isGoalReached() && potBugController.isRunning){
          //Check for goal direction on forwards semi-circle
          while (potBugController.isFreeMode()){
            //Check Heading
            //Face Goal
            potBugController.executeFreeMode();

            //Move 'freely' to the next sample point, towards goal

          }

          while (potBugController.isApproachMode()){
            //'Approach' the obstacle
            //Reduce sampling as hit point i.e. bands are approached and distances reduce
            //Turn in small radians away from the obstacle as approaching closer
            ////10 degrees to either L or R per second - or whatever
            //Move away if line of sight is clear to goal, else 'engage bug'
            potBugController.executeApproachMode();

          }

          while (potBugController.isEngagedMode()){
            //'Engage' the bug algorithm
            //Run along obstacle bounds till goal euclidean distance is less
            potBugController.executeEngageMode();

          }
        }
      }
    }
  }

  private void executeFreeMode() {
    //Set current position
    currentPosition = currentNode.getPosition();
    System.out.println("SIM POS:: "+ currentNode.getX()+" , "+ currentNode.getY());
    System.out.println("ACT POS:: "+ getCurrentRobotPosition().x + " , " + getCurrentRobotPosition().y);

    //Get next sample (POT-BUG will sample 1 per move)
    nextNode = currentNode.getChildren().get(0);

    //nextNode.setPosition(getNextPosition);
    nextNode.setChildren(getChildren(nextNode));
    nextNode.setCost(getCurrentObstaclePotential());
    nextNode.setParent(currentNode);
    currentTree.add(currentNode);

    //Set speed and move/drive
    double[] speedVals = new double[]{nextNode.getX(), nextNode.getY()};
    setRobotSpeed(speedVals[0], speedVals[1]);

    //set current node as next node
    currentNode = nextNode;

  }

  private void executeApproachMode(){
    this.hitPointDistance = getDistanceToGoal();
    System.out.println("Hit Point Distance : " + this.hitPointDistance);
    executeContourMove();
  }

  private void executeEngageMode(){

  }

  private void executeContourMove(){
    System.out.println("Executing Obstacle Contour Move...");
    //Increase sampling for more node alternatives
    this.randomSampler = new RandomSampler(DEFAULT_APROACH_SAMPLES,
        DEFAULT_APROACH_RATIO*randomSampler.getStepSize(),
        DEFAULT_MIN_SAMPLE_VAL, DEFAULT_APROACH_RATIO*DEFAULT_MAX_SAMPLE_VAL);

    currentNode.setPosition(getCurrentRobotPosition());
    currentNode.setChildren(getChildren(currentNode));
    nextNode = getBestChild(currentNode.getChildren());
    currentTree.add(nextNode);

    //ASK ZAHIDA ABOUT B Contour
    //Drive to next best node
    setRobotSpeed(nextNode.getX(),nextNode.getY());

    //After drive, next node becomes current node
    currentNode = nextNode;

  }

  private Node getBestChild(List<Node> children) {
    int i = 1;
    Node bestChild = children.get(random.nextInt(children.size()));
    for (Node child: children) {
      System.out.println("Potential child "+ i + " cost: " + child.getCost());

      if(bestChild.getCost() > child.getCost()){
        bestChild = child;
      }
    }
    /*for (int i = 0; i < children.size(); i++) {
      System.out.println("Potential child "+ i + " cost: " + children.get(i).getCost());
    }*/
    return bestChild;
  }

  private void initAlgorithm() {
    startNode = new Node(startPosition, 0, null, null);
    startNode.setChildren(getChildren(currentNode));
    currentTree.add(currentNode);
    bestTree.add(currentNode);
    //Get first node
    System.out.println("Current Tree size : " + currentTree.size());
    currentNode = currentTree.get(currentTree.size()-1);

  }

  private void initPositions() {
    goalPosition = new Position(DEFAULT_GOAL_POSITION_X, DEFAULT_GOAL_POSITION_Y);
    System.out.println("Goal Position set to : " + goalPosition.toArray().toString());

    startPosition = getCurrentRobotPosition(); // y - value in webots = altitude
    System.out.println("Goal Position set to : " + startPosition.toArray().toString());

    this.currentPosition = startPosition;
    System.out.println("Goal Position set to : " + currentPosition.toArray().toString());
  }


  private boolean isGoalReached(){
    System.out.println("Checking if current position is Goal...");

    if(getDistanceToGoal()<DEFAULT_GOAL_RADIUS){
      System.out.println("Goal Position Reached");
      return true;
    } else{
      return false;
    }
  }


  private boolean isFreeMode(){
    //Check if robot is in free mode
    if(getCurrentObstaclePotential()<DEFAULT_MAX_APPROACH_POTENTIAL) {
      return true;
    } else {
      return false;
    }
  }

  private boolean isApproachMode(){
    //Check if robot is in free mode
    if((getCurrentObstaclePotential()>DEFAULT_MAX_APPROACH_POTENTIAL)
        && (getCurrentObstaclePotential()<DEFAULT_MAX_ENGAGE_POTENTIAL)) {

      return true;
    } else {
      return false;
    }
  }

  private boolean isEngagedMode(){
    if(getCurrentObstaclePotential()>DEFAULT_MAX_ENGAGE_POTENTIAL) {
      return true;
    } else {
      return false;
    }
  }

  private Position getNextPosition(){
    return new Position(0,0);
  }

  private double[] getSpeedValues(Position currentPosition, Position nextPosition){
    double[] speedVals = new double[2];
    return speedVals;
  }

  private List<Node> getChildren(Node parameterNode){

    List<ControlParameters> samples = new ArrayList<>(
        randomSampler.getSamplesFrom(parameterNode.getNodeParameters()));

    List<Node> possibleChildren = new ArrayList<>(samples.size());

    for (int i = 0; i < samples.size() ; i++) {
      double[] vals = samples.get(i).getArray();
      Position childPosition = new Position(vals[0], vals[1]);

      possibleChildren.add(new Node(childPosition, getCurrentObstaclePotential(),parameterNode, null));
    }
    return possibleChildren;
  }

  private double getDistanceToGoal(){
    double[] gpsValues = gps.getValues();
    for (int i = 0 ; i < gpsValues.length ; i++) {
      System.out.println ("gpsValues["+i+"]: "+gpsValues[i] );
    }
    System.out.println  ( "Goal: " + goalPosition.toString() ) ;

    return getDistanceBetween(this.currentPosition, this.goalPosition);
  }

  private double getDistanceBetween(Position origin, Position destination) {

    return distanceCalculator.getDistanceBetween(origin, destination);
  }

  private Position getCurrentRobotPosition(){
    Position position = new Position(gps.getValues()[0],gps.getValues()[1]);
    return position;

  }
  private synchronized double getCurrentObstaclePotential(){
    //Calculate obstacle potential in current position
    //Custom calculation - to be tested
    double obstacleField = 0;
    double obstaclePotential = 0;
    double goalPotential = 0;

    for(LidarPoint obstacle : obstaclePoints){
      //Generate obstacle positions
      Position obstaclePosition = convertRelativeToAbsolute(new PositionAndHeading(new Position(-obstacle.getX(),
          -obstacle.getZ()),0)).position;

      //Calculate distance from obstacle to eliminate out of range o's
      double obstacleDistance = distanceCalculator.getDistanceBetween(currentPosition, obstaclePosition);

      if((obstacleDistance < DEFAULT_SENSOR_RANGE) && !(DEFAULT_OBSTACLE_COEFFICIENT == 0)){ // might add obst coeff
        //Calculate field: increases if close to obstacle
        obstacleField += Math.pow(Math.E, -1/(DEFAULT_SENSOR_RANGE-obstacleDistance))/obstacleDistance;
        System.out.println("Obstacle Field: " + obstacleField);
      }
    }

    //Calculate Potentials
    goalPotential = DEFAULT_GOAL_COEFFICIENT * Math.pow(getDistanceBetween(currentPosition, goalPosition), 2);
    obstaclePotential = DEFAULT_OBSTACLE_COEFFICIENT * obstacleField;

    return goalPotential + obstacleField;
  }

  private double getHeadingtoPosition(Position destination){

    //If no destination given, assume
    if(destination == null){
      destination = this.goalPosition;
    }
    double[] compassMeasurements = compass.getValues();

    //double heading = distanceCalculator


    return compassMeasurements[0];
  }

  private double getBearing(Position destination){

    return 0;
  }


  private void setRobotSpeed(double leftMotorSpeed, double rightMotorSpeed){
    motors[0].setVelocity(leftMotorSpeed);
    motors[1].setVelocity(rightMotorSpeed);
    System.out.println();
    motors[2].setVelocity(leftMotorSpeed);
    motors[3].setVelocity(rightMotorSpeed);
    //Robot speeds
    System.out.println("Motor Speeds Set: L = " + leftMotorSpeed + " R = " + rightMotorSpeed);
  }

  private synchronized void updateObstacleReadings(LidarPoint[] obstacleImage){
    this.obstaclePoints = obstacleImage;
    System.out.println("Obstacle Readings Updated ");
  }

  private PositionAndHeading convertRelativeToAbsolute(PositionAndHeading position) {
    double[] gpsValues = gps.getValues();
    double[] compassValues = compass.getValues();
    double direction = Math.PI + Math.atan2(compassValues[2], compassValues[0]);

    double translatedX = position.position.x * Math.cos(direction) + position.position.y * Math.sin(direction);
    double translatedY = -position.position.x * Math.sin(direction) + position.position.y * Math.cos(direction);

    return new PositionAndHeading(new Position(translatedX + gpsValues[0], translatedY + gpsValues[2]),
        (direction + position.heading) % (Math.PI * 2));
  }

  private void terminatePotBug() {
    setRobotSpeed(0, 0);
    isRunning = false;
    System.out.println("Controller Terminated");
    //Maybe add some stats
    return;
  }

}
