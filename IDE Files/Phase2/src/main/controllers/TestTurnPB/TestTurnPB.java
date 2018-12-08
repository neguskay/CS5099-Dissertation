// File:          TestTurnPB.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.LED;
//import com.cyberbotics.webots.controller.Robot;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
// File:          TestPotBug.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.LED;

import com.cyberbotics.webots.controller.*;
import main.controllers.testControllers.metrics.MetricsCalculator;
import main.controllers.testControllers.node.Node;
import main.java.kak3.shp.algorithms.samplers.RandomSampler;
import main.java.kak3.shp.framework.ControlParameters;
import main.java.kak3.shp.framework.Position;
import main.java.kak3.shp.framework.distance.DistanceCalculator;
import main.java.kak3.shp.framework.distance.Euclidean;

import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

public class TestTurnPB{

  //Ratios and Ranges
  private static final double DEFAULT_SENSOR_RANGE = 1.0; // meters
  private static final double DEFAULT_ROBOT_RADIUS = 2.0; //meters
  private static final double DEFAULT_GOAL_RADIUS = 1.0; //meters
  private static final double DEFAULT_HIT_POINT_DISTANCE = 1.0; // Meters

  //Drive Modes
  private static final int DEFAULT_DRIVE_MODE_TOWARDS_GOAL = 1; //Free Mode
  private static final int DEFAULT_DRIVE_MODE_TOWARDS_CONTOUR = 2;  //Towards Hit Point
  private static final int DEFAULT_DRIVE_MODE_ALONG_CONTOUR = 3;  //Along Obstacle Boundary

  //Cost funtion Types
  private static final String DEFAULT_COST_TYPE_POT = "POT"; // Meters
  private static final String DEFAULT_COST_TYPE_DIST = "DIST"; // Meters
  private String currentCostType = "";

  private static final double DEFAULT_BAND_A = 10.0; //
  private static final double DEFAULT_BAND_B = 2.0; //
  private static final double DEFAULT_BAND_C = 1.0; //

  private static final double DEFAULT_OBSTACLE_COEFFICIENT = 0.001;
  private static final double DEFAULT_GOAL_COEFFICIENT = 0.002;

  private static final double DEFAULT_MOTOR_VELOCITY = 0.2;
  private static final double DEFAULT_APROACH_STEP_SIZE = 3;
  private static final double DEFAULT_ENGAGE_STEP_SIZE = 2;
  //private static final int DEFAULT_APROACH_SAMPLES = 3;
  private static final int DEFAULT_MAX_ITERATION_COUNT = 1000;
  private static double DEFAULT_LOOKAHEAD_TIME;

  //Sampling
  private static final int DEFAULT_SAMPLE_SIZE = 5; //might change later
  private static final int DEFAULT_STEP_SIZE = 5; //might change later
  private static final double DEFAULT_MIN_SAMPLE_VAL = 0; // meters. -- Try a negative number and check the results
  private static final double DEFAULT_MAX_SAMPLE_VAL = 10; // meters.

  //Default goal positions
  private static final double DEFAULT_GOAL_POSITION_X = 3.0;
  private static final double DEFAULT_GOAL_POSITION_Y = 15.0;

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
  private Position hitPoint, hitObstacle;

  //Direction Angles
  private double currentHeading = 0;
  private double turnAngle = 0;
  private double bearingAngle = 0;

  //
  private boolean isRunning = false;
  private double hitPointDistance;

  //Nodes
  private Node currentNode, nextNode, startNode, goalNode;
  private List<Node> currentTree = new LinkedList<>();
  private List<Node> bestTree = new LinkedList<>();

  //Math
  private DistanceCalculator distanceCalculator;
  private Euclidean euclidean = new Euclidean();
  private MetricsCalculator metricsCalculator;

  //Algorithms
  private Random random = new Random();
  private RandomSampler randomSampler = new RandomSampler(
      DEFAULT_SAMPLE_SIZE, DEFAULT_STEP_SIZE, DEFAULT_MIN_SAMPLE_VAL, DEFAULT_MAX_SAMPLE_VAL);

  //Metrics
  private long nodeGenTime;
  private long nodeDriveTime;
  private double distanceTravelled;
  private double cumulativeEuclidean;
  private int nodeCount;
  private int nodeExploredCount;
  private double cumulativeTurnAngle;

  //Thread
  private Thread initThread, nodeGenThread, nodeDriveThread;

  //Thread Manager
  private ThreadMXBean threadMXBean;


  public TestTurnPB(GPS gps, Compass compass, Emitter emitter, Motor frontLeft,
                    Motor frontRight, Motor backLeft, Motor backRight) {

    this.motors[0] = frontLeft;
    this.motors[1] = frontRight;
    this.motors[2] = backLeft;
    this.motors[3] = backRight;

    this.gps = gps;
    this.compass = compass;
    this.emitter = emitter;

    this.threadMXBean = ManagementFactory.getThreadMXBean();

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

    TestTurnPB turnPBController = new TestTurnPB(gps, compass, emitter, frontLeft, frontRight, backLeft, backRight);
    System.out.println("current param  : " + turnPBController.toString());

    //Initialise Start and Goal Coordinates
    robot.step(timeStep);
    turnPBController.updateObstacleReadings(lidar.getPointCloud());
    turnPBController.isRunning = true;
    turnPBController.initMetrics();
    turnPBController.initPositions();
    //potBugController.initAlgorithm();
    turnPBController.runInit();

    //Iteration Count
    int itCount = 0;

    while (robot.step(timeStep*100) != -1) {
      //Check max iteration
      //while (itCount<DEFAULT_MAX_ITERATION_COUNT){
      //Update Turn Angles
      turnPBController.updateAngles();

      //Update Sensor Information
      turnPBController.updateObstacleReadings(lidar.getPointCloud());

      //Check if goal is reached and terminate as appropriate
      if (turnPBController.isGoalReached()){
        turnPBController.isRunning = false;
        turnPBController.setRobotSpeed(0,0);
        turnPBController.terminatePotBug();
        System.out.println("Goal Reached Successfully!");
      }

      if(!turnPBController.isGoalReached() && turnPBController.isRunning){
        //Check for goal direction on forwards semi-circle
        if (!turnPBController.isObstruction()){

          turnPBController.setDriveMode(DEFAULT_DRIVE_MODE_TOWARDS_GOAL);
          //Move 'freely' to the next sample point, towards goal
          turnPBController.updateMetrics();

        } else {

          if (turnPBController.isClearToLeave()){

            if (turnPBController.isProgressAttained() &&
                turnPBController.isClearToLeave()){

              turnPBController.setDriveMode(DEFAULT_DRIVE_MODE_TOWARDS_GOAL);
              turnPBController.updateMetrics();

            }
          } else {

            if (turnPBController.hitPoint == null){

              turnPBController.setHitPoint();
              turnPBController.setDriveMode(DEFAULT_DRIVE_MODE_TOWARDS_CONTOUR);
              turnPBController.updateMetrics();

            } else {

              // engaged mode
              turnPBController.setDriveMode(DEFAULT_DRIVE_MODE_ALONG_CONTOUR);
              turnPBController.updateMetrics();

            }
          }
        }
      }
      //}
    }
  }

  private void runInit(){

    this.initThread = new Thread(this::initAlgorithm);
    long initID = this.initThread.getId();
    this.initThread.run();

    this.nodeGenTime += this.threadMXBean.getThreadCpuTime(initID);
    System.out.println("TRYYYYYYYPOTInit: "+ this.nodeGenTime);

  }

  private void runNodeGenFree(){
    this.nodeGenThread = new Thread(this::generateFreeNode);
    long nodeGenID = this.nodeGenThread.getId();
    this.nodeGenThread.run();

    this.nodeGenTime += this.threadMXBean.getThreadCpuTime(nodeGenID);
    System.out.println("TRYYYYYYYYYYYTPOT-NODE-GENFREE: "+ this.nodeGenTime);
  }

  private void runNodeGen(){
    this.nodeGenThread = new Thread(this::generateNextNode);
    long nodeGenID = this.nodeGenThread.getId();
    this.nodeGenThread.run();

    this.nodeGenTime += this.threadMXBean.getThreadCpuTime(nodeGenID);
    System.out.println("TRYYYYYYYYYYYTPOT-NODE-GEN: "+ this.nodeGenTime);
  }

  private void runNodeDrive(){
    this.nodeDriveThread = new Thread(this::driveToNextNode);
    long nodeDriveId = this.nodeDriveThread.getId();
    this.nodeDriveThread.run();

    this.nodeDriveTime += this.threadMXBean.getThreadCpuTime(nodeDriveId);
    System.out.println("TRYYYYYYYYYYYTPOT-noDEdrIVE: "+ this.nodeDriveTime);
  }


  private void computeDriveMechanics() {
    double angleInDeg = turnAngle * (180/Math.PI);
    double timerRatio = angleInDeg/10;
    System.out.println("Angle in DEG : " + angleInDeg);
    System.out.println("Timer Ratio : " + timerRatio);

  }

  private void initAlgorithm() {
    setRobotSpeed(0, 0);

    long start1 = System.nanoTime();
    System.out.println("sssssss:: "+startPosition);
    startNode = new Node(startPosition, 0, null, null);
    System.out.println(startNode);
    startNode.setChildren(getChildren(startNode,DEFAULT_COST_TYPE_POT));
    long end1 = System.nanoTime();
    currentTree.add(currentNode);
    bestTree.add(currentNode);

    //Get first node
    System.out.println("Current Tree size : " + currentTree.size());
    currentNode = startNode;

    //Update Metrics
    this.nodeCount = currentTree.size();
    this.nodeExploredCount = bestTree.size();
    this.nodeGenTime += end1 - start1;
    //this.nodeDriveTime += end2 - start2;
  }

  private void initPositions() {
    goalPosition = new Position(DEFAULT_GOAL_POSITION_X, DEFAULT_GOAL_POSITION_Y);
    System.out.println("Goal Position set to :: X= " + goalPosition.x + ", Y = "+ goalPosition.y );

    startPosition = getCurrentRobotPosition(); // y - value in webots = altitude
    System.out.println("Start Position set to :: X= " + startPosition.x + ", Y = "+ startPosition.y );

    this.currentPosition = startPosition;
    System.out.println("Current Position set to :: X= " + currentPosition.x + ", Y = "+ currentPosition.y );

    this.goalNode = new Node(this.goalPosition,0, null, null);
  }

  private void initMetrics(){
    //Metrics
    this.nodeCount = 0;
    this.nodeGenTime = 0;
    this.distanceTravelled = 0;
    this.nodeDriveTime = 0;
    this.cumulativeEuclidean = 0;
    this.nodeExploredCount = 0;
    this.cumulativeTurnAngle = 0;
  }


  private boolean isGoalReached(){
    System.out.println("Checking if current position is Goal...");
    System.out.println(this.goalNode.getPosition()+ "Goalsssss");
    double distanceToGoal = getEuclidean(getCurrentRobotPosition(), this.goalNode.getPosition());
    if(distanceToGoal < DEFAULT_GOAL_RADIUS){
      return true;
    } else{
      return false;
    }
  }

  private boolean isClearToLeave(){
    if(getCurrentObstaclePotential()< DEFAULT_MAX_APPROACH_POTENTIAL){
      return true;
    } else {
      return false;
    }
  }

  private boolean isProgressAttained(){
    this.currentPosition = getCurrentRobotPosition();
    double progressDist = getEuclidean(this.currentPosition, this.goalPosition);

    if(progressDist < this.hitPointDistance){
      return true;
    } else {
      return false;
    }
  }

  private boolean isObstruction(){
    double currentPotential = getCurrentObstaclePotential();
    if((currentPotential < DEFAULT_MAX_ENGAGE_POTENTIAL) &&
        (currentPotential > DEFAULT_MAX_APPROACH_POTENTIAL)){

      return true;

    } else {
      return false;
    }
  }

  private double[] getSpeedValues(){
    double speed = 0.230; //needs changing
    double length = 0.250; //""
    double radius = 0.110; //""

    System.out.println("Current Heading: "+ this.bearingAngle);
    double leftSpeed = ((2*speed) + (this.bearingAngle * length)) / (2*radius);
    double rightSpeed = ((2*speed) - (this.bearingAngle * length)) / (2*radius);

    System.out.println("L-SPEED: " +leftSpeed+ "RIGHT: "+ rightSpeed);

    return new double[]{leftSpeed,rightSpeed};
  }

  private List<Node> getChildren(Node parameterNode, String costType){
    System.out.println(parameterNode);

    List<ControlParameters> samples = new ArrayList<>(
        this.randomSampler.getSamplesFrom(parameterNode.getNodeParameters()));

    System.out.println(samples.size());
    List<Node> possibleChildren = new ArrayList<>(samples.size());

    for (int i = 0; i < samples.size() ; i++) {
      double[] vals = samples.get(i).getArray();
      Position childPosition = new Position(vals[0], vals[1]);

      if(costType.equalsIgnoreCase( "POT")){

        possibleChildren.add(new Node(
            childPosition, getCurrentObstaclePotential(),parameterNode, null));

      } else if (costType.equalsIgnoreCase( "DIST")) {

        possibleChildren.add(new Node(
            childPosition, getEuclidean(childPosition, this.goalPosition),parameterNode, null));
      }
    }
    return possibleChildren;
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

  private double getDistanceToGoal(){
    double[] gpsValues = gps.getValues();
    for (int i = 0 ; i < gpsValues.length ; i++) {
      System.out.println ("gpsValues["+i+"]: "+gpsValues[i] );
    }
    System.out.println  ( "Goal: " + goalPosition.toString() ) ;

    return getDistanceBetween(this.currentPosition, this.goalPosition);
    
  }

  private double getDistanceBetween(Position origin, Position destination) {

    return getEuclidean(origin, destination);
  }

  private Position getCurrentRobotPosition(){
    Position position = new Position(gps.getValues()[0], gps.getValues()[2]);
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
      Position obstaclePosition = new Position(-obstacle.getX(), -obstacle.getZ());
      //System.out.println("Current: "+currentPosition);
      //System.out.println("Obstacle: "+obstaclePosition);
      //Calculate distance from obstacle to eliminate out of range o's
      double obstacleDistance = getEuclidean(this.currentPosition, obstaclePosition);
      //System.out.println("Distance = "+obstacleDistance);

      if((obstacleDistance < DEFAULT_SENSOR_RANGE) && !(DEFAULT_OBSTACLE_COEFFICIENT == 0)){ // might add obst coeff
        //Calculate field: increases if close to obstacle
        obstacleField += Math.pow(Math.E, -1/(DEFAULT_SENSOR_RANGE-obstacleDistance))/obstacleDistance;
        //System.out.println("Obstacle Field: " + obstacleField);
      }
    }

    //Calculate Potentials
    goalPotential = DEFAULT_GOAL_COEFFICIENT * Math.pow(getEuclidean(currentPosition, goalPosition), 2);
    obstaclePotential = DEFAULT_OBSTACLE_COEFFICIENT * obstacleField;

    double totalPot = goalPotential + obstaclePotential;
    return totalPot;
  }

  private double getEuclidean(Position pos1, Position pos2) {
    double[] point1 = pos1.toArray();
    double[] point2 = pos2.toArray();

    double sum = 0;

    sum = Math.pow(point2[1]-point1[1],2) + Math.pow(point2[0]-point1[0],2);
  /*  for (int i = 0; i < point1.length; i++) {
      sum += Math.pow(point1[i] - point2[i], 2);
    }*/

    return Math.sqrt(sum);
  }

  private void setDriveMode(int driveMode){
    switch (driveMode) {
      case 1: //free mode drive
        this.randomSampler = new RandomSampler(
            DEFAULT_SAMPLE_SIZE, DEFAULT_STEP_SIZE,
            DEFAULT_MIN_SAMPLE_VAL, DEFAULT_MAX_SAMPLE_VAL);

        this.currentCostType = DEFAULT_COST_TYPE_POT;

        runNodeGen();   //Swapsies
        //runNodeGenFree();
        runNodeDrive();

        break;

      case 2: //approach mode drive
        this.randomSampler = new RandomSampler(
            DEFAULT_SAMPLE_SIZE,
            DEFAULT_APROACH_STEP_SIZE,
            DEFAULT_MIN_SAMPLE_VAL,
            DEFAULT_MAX_SAMPLE_VAL);

        this.currentCostType = DEFAULT_COST_TYPE_POT;

        runNodeGen();
        runNodeDrive();

        break;


      case 3: //engage mode drive
        this.randomSampler = new RandomSampler(
            DEFAULT_SAMPLE_SIZE,
            DEFAULT_ENGAGE_STEP_SIZE,
            DEFAULT_MIN_SAMPLE_VAL,
            DEFAULT_MAX_SAMPLE_VAL);

        this.currentCostType = DEFAULT_COST_TYPE_DIST;

        runNodeGen();
        runNodeDrive();

        break;

      default:
        this.randomSampler = new RandomSampler(
            DEFAULT_SAMPLE_SIZE, DEFAULT_STEP_SIZE,
            DEFAULT_MIN_SAMPLE_VAL, DEFAULT_MAX_SAMPLE_VAL);

        this.currentCostType = DEFAULT_COST_TYPE_POT;

        runNodeGen();
        runNodeDrive();

        break;
    }

    System.out.println("Drive Move Set : " + driveMode );
  }

  private void generateNextNode(){
    //Set current position
    long start1 = System.nanoTime();
    currentPosition = getCurrentRobotPosition();
    //System.out.println("currenenenen:: "+ currentNode.getPosition());
    //System.out.println("currenenenen:: "+ currentPosition);

    if(currentPosition != currentNode.getPosition()){
      currentNode.setPosition(currentPosition);

      if (currentCostType.equalsIgnoreCase(DEFAULT_COST_TYPE_POT)){
        currentNode.setChildren(getChildren(currentNode, DEFAULT_COST_TYPE_POT));

      } else if (currentCostType.equalsIgnoreCase(DEFAULT_COST_TYPE_DIST)){

        currentNode.setChildren(getChildren(currentNode, DEFAULT_COST_TYPE_DIST));
      }

      System.out.println("SIM POS:: "+ currentNode.getX()+" , "+ currentNode.getY());
      System.out.println("ACT POS:: "+ getCurrentRobotPosition().x + " , " + getCurrentRobotPosition().y);
    }

    //currentPosition = currentNode.getPosition();
    //Get next sample (POT-BUG will sample 1 per move)

    nextNode = getBestChild(currentNode.getChildren());

    if (currentCostType.equalsIgnoreCase(DEFAULT_COST_TYPE_DIST)){

      nextNode.setChildren(getChildren(nextNode, DEFAULT_COST_TYPE_DIST));

    } else if (currentCostType.equalsIgnoreCase(DEFAULT_COST_TYPE_POT)){

      nextNode.setChildren(getChildren(nextNode, DEFAULT_COST_TYPE_POT));

    }

    //Add to trees
    currentTree.add(currentNode);
    currentTree.addAll(currentNode.getChildren());
    bestTree.add(currentNode);
    long end1 = System.nanoTime();

  }

  private void generateFreeNode(){
    //Set current position
    long start1 = System.nanoTime();
    currentPosition = getCurrentRobotPosition();

    if(currentPosition != currentNode.getPosition()){
      currentNode.setPosition(currentPosition);

      System.out.println("SIM POS:: "+ currentNode.getX()+" , "+ currentNode.getY());
      System.out.println("ACT POS::: "+ getCurrentRobotPosition().x + " , " + getCurrentRobotPosition().y);
    }

    nextNode = goalNode;

    //Add to trees
    currentTree.add(currentNode);
    //currentTree.addAll(currentNode.getChildren());
    bestTree.add(currentNode);
    long end1 = System.nanoTime();
  }

  private void driveToNextNode(){
    //long start2 = System.nanoTime();

    //Set speed and move/drive
    //Compute Drive Mechanics
    //initTurnAngles();
    updateAngles();
    computeDriveMechanics();

    //Set Speeds and drive
    double[] speedValues = getSpeedValues();
    setRobotSpeed(speedValues[0], speedValues[1]);
    //long end2 = System.nanoTime();

    //Update Metrics
    this.nodeCount = currentTree.size();
    this.nodeExploredCount = bestTree.size();
    //this.nodeGenTime += end1 - start1;
    //this.nodeDriveTime += end2 - start2;

    System.out.println("currenenenen:: "+ currentNode.getPosition());
    System.out.println("currenenenen:: "+ nextNode.getPosition());

    this.cumulativeEuclidean += getDistanceBetween(
        currentNode.getPosition(), nextNode.getPosition());

    //updateMetrics();

    //set current node as next node
    currentNode = nextNode;

    this.cumulativeTurnAngle += this.bearingAngle;
  }


  private void setHitPoint(){
    Position currObst = new Position(this.obstaclePoints[0].getX(), this.obstaclePoints[0].getY());
    double lowestObstDist = getEuclidean(this.currentPosition, this.hitObstacle);

    for (LidarPoint obstacle:
        this.obstaclePoints) {
      Position currObstPos = new Position(-obstacle.getX(), -obstacle.getZ());
      double currentObstDist = getEuclidean(this.currentPosition, currObstPos);

      if (currentObstDist < lowestObstDist){
        currObst = currObstPos;
        lowestObstDist = currentObstDist;
      }
    }

    if(lowestObstDist < DEFAULT_HIT_POINT_DISTANCE) {
      //IN HIT POINT RANGE
      this.hitPoint = currObst;
      this.hitPointDistance = getEuclidean(this.hitPoint, this.goalPosition);

    } else{
      this.hitPoint = null;
    }
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

  private void updateMetrics(){
    //init Metrics
    this.metricsCalculator = new MetricsCalculator(this.nodeGenTime,this.nodeDriveTime,
        this.distanceTravelled, this.nodeCount, this.nodeExploredCount, this.cumulativeEuclidean,
        this.cumulativeTurnAngle);

    //Print update to console
    this.metricsCalculator.getMetricsSummary();
  }

  private void updateAngles(){
    //Update Heading from compass
    double xCompassVal, yCompassVal, radCompassVal, compassHeading;
    xCompassVal = compass.getValues()[0];
    yCompassVal = compass.getValues()[2];
    radCompassVal = Math.atan2(yCompassVal, xCompassVal);
    compassHeading = (radCompassVal - 1.5708);

    if(compassHeading < 0.0){
      compassHeading = compassHeading + (2 * Math.PI);
    }
    this.currentHeading = compassHeading;


    //Update difference to position
    double dx = 0.0;
    double dy = 0.0;
    double bearing = 0;
    if(this.currentNode != null && this.nextNode != null ){
      System.out.println("Nodes Not Null");
      dx = this.nextNode.getX() - this.currentNode.getX();
      dy = this.nextNode.getY() - this.currentNode.getY();

    }

    if (dx > 0 ){
      System.out.println("RADD CHECK " + Math.atan(dy/dx));
      bearing = (Math.PI/2) - Math.atan(dy/dx);

    }

    if (dx > 0 ){
      System.out.println("RADD " + Math.atan(dy/dx));
      bearing = (0.75 * Math.PI) - Math.atan(dy/dx);

    }

    if (dx == 0){
      System.out.println("dx == 0; Avoiding divide by 0...");

      if (dy > 0) bearing = 0;
      if (dy < 0) bearing = Math.PI;
      if (dy == 0) bearing = 0;
    }

    this.bearingAngle = bearing;

  }

  private void terminatePotBug() {
    setRobotSpeed(0, 0);
    isRunning = false;
    System.out.println("Controller Terminated");
    //Maybe add some stats
    return;
  }
}