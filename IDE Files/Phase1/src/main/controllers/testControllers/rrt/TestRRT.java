package main.controllers.testControllers.rrt;


// File:          TestRRT.java
// Date:
// Description:
// Author: 170027939@ST-ANDREWS UNIVERSITY
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.LED

//package main.controllers.testControllers.rrt;


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
import java.util.*;


/**
 * Controller used to Implement the RRT Algorithm within the WEBOTS application
 */
public class TestRRT {

  private static final double DEFAULT_GOAL_RADIUS = 1; // meters.
  private static final double[] DEFAULT_GOAL_PARAMETERS = { 3.0 , 15.0 }; // goal {x, y}
  private static final double DEFAULT_MAX_SPEED = 4;
  private static final double DEFAULT_AXLE_LENGTH = 2;
  private static final int DEFAULT_MAX_ITERATIONS = 100;

  //Sampling
  private final int numSamples = 5;
  private final double  stepSize = 5; // meters.
  private final double minVal = 0; // meters. -- Try a negative number and check the results
  private final double maxVal = 10; // meters.

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
  private double currentHeading = 0;
  private double turnAngle = 0;
  private double bearingAngle = 0;

  //Other
  private DistanceCalculator distanceCalculator;

  //Obstacles
  private LidarPoint[] obstacleCoordinates;

  //Timers
  private Timer timer;
  private TimerTask timerTask;

  //Algorithms
  private final Random random = new Random();
  private final Euclidean euclidean = new Euclidean();
  private MetricsCalculator metricsCalculator;
  private RandomSampler randomSampler = new RandomSampler(numSamples, stepSize, minVal, maxVal);

  //Control Parameters
  private ControlParameters currentParameters, nextParameters;

  //Nodes
  private Node currentNode, nextNode;
  private List<Node> currentTree = new LinkedList<>();
  private List<Node> bestTree = new LinkedList<>();

  //Metrics
  private long nodeGenTime;
  private long nodeDriveTime;
  private double distanceTravelled;
  private double cumulativeEuclidean;
  private int nodeCount;
  private int nodeExploredCount;
  private double cumulativeTurnAngle;

  //Threads
  private Thread initThread, nodeGenThread, nodeDriveThread;

  //Thread Manager
  private ThreadMXBean threadMXBean;

  //Constructor of RRT Controller
  /**
   * Rapidly-Exploring Random Tree constructor implementation for WEBOTS
   * @param gps Robots GPS object
   * @param compass Robots compass object
   * @param emitter Robots emitter object
   * @param frontLeft Robots frontLeft Motor object
   * @param frontRight Robots frontRight Motor object
   * @param backLeft Robots backLeft Motor object
   * @param backRight Robots backRight Motor object
   */
  public TestRRT(GPS gps, Compass compass, Emitter emitter,
                 Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {


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
    timeStep = (timeStep*1000)/32;

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


    robot.step(timeStep);
    rrtController.updateSensorReadings(lidar.getPointCloud());
    rrtController.initMetrics();
    //rrtController.initAlgorithm();
    rrtController.runInit();
    rrtController.isRunning = true;

    //Iteration Count for frequency
    int iterationCount = 0;

    while (robot.step(timeStep) != -1) {

      //while (iterationCount <DEFAULT_MAX_ITERATIONS){
      //Update Turning Angles
      rrtController.updateAngles();

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
        rrtController.runNodeGen();
        rrtController.runNodeDrive();
        //rrtController.driveToGoal();

        System.out.println("Building Trees...");

        rrtController.updateMetrics();

      }
      // }
    }
  }

  /**
   * runs thread for algorithm initialisation
   */
  private void runInit(){

    this.initThread = new Thread(this::initAlgorithm);
    long initID = this.initThread.getId();
    this.initThread.run();

    this.nodeGenTime = this.threadMXBean.getThreadCpuTime(initID);
    //System.out.println("TRYYYYYYYInit: "+ this.nodeGenTime);

  }

  /**
   * runs thread for algorithm next node generation
   */
  private void runNodeGen(){
    this.nodeGenThread = new Thread(this::generateNextNode);
    long nodeGenID = this.nodeGenThread.getId();
    this.nodeGenThread.run();

    this.nodeGenTime += this.threadMXBean.getThreadCpuTime(nodeGenID);
    //System.out.println("TRYYYYYYYYYYYTNODE-GEN: "+ this.nodeGenTime);
  }

  /**
   * runs thread for controller drive to next node
   */
  private void runNodeDrive(){
    this.nodeDriveThread = new Thread(this::driveToNextNode);
    long nodeDriveId = this.nodeDriveThread.getId();
    this.nodeDriveThread.run();

    this.nodeDriveTime += this.threadMXBean.getThreadCpuTime(nodeDriveId);
    //System.out.println("TRYYYYYYYYYYYTnoDEdrIVE: "+ this.nodeDriveTime);
  }

  /**
   * generates nodes based on current node
   */
  private void generateNextNode(){
    if (this.currentNode == null){
      System.out.println("NULL NODES");
      terminate();
    }

    Node current = this.currentNode;


    currentPosition = getCurrentPosition();
    long start1 = System.nanoTime();
    if(current.getPosition() != currentPosition ){
      //currentPosition = new Position(gps.getValues()[0], gps.getValues()[2]);
      current.setPosition(currentPosition);
      current.setChildren(getChildren(current));
      bestTree.add(current);
      currentTree.add(current);

      currentTree.addAll(current.getChildren());

    }

    Node next = getBestNode(current.getChildren());

    if (next != getBestNode(current.getChildren())){
      next = getBestNode(current.getChildren());
    }

    this.nextNode = next;
    //this.currentNode = current;
    this.nextNode.setParent(this.currentNode);

    long end1 = System.nanoTime();
    //this.nodeGenTime += end1 - start1;
  }


  /**
   * implements controller drive to successive nodes
   */
  private void driveToNextNode() {

    long start2 = System.nanoTime();
    //Compute Drive Mechanics
    //updateTurnAngles();
    updateAngles();
    computeDriveMechanics();

    //SetSpeeds
    double[] speedValues = getSpeeds();
    setSpeed(speedValues[0], speedValues[1]);

    //Set Next Node
    this.currentNode = this.nextNode;
    currentNode.setChildren(getChildren(currentNode));
    this.nextNode = getBestNode(currentNode.getChildren());
    long end2 = System.nanoTime();


    //Update Metrics
    this.nodeCount = currentTree.size();
    this.nodeExploredCount = bestTree.size();
    //this.nodeDriveTime += end2 - start2;
    this.cumulativeEuclidean+= euclidean.getDistanceBetween(currentNode.getPosition(), nextNode.getPosition());
    this.cumulativeTurnAngle += this.bearingAngle;
    //updateMetrics();

  }


  /**
   * Build trees in controller
   */
  private void buildTree() {
    long start1 = System.nanoTime();
    for(int i=0; i < currentNode.getChildren().size(); i++){
      Node node = currentNode.getChildren().get(i);
      node.setChildren(getChildren(node));
      currentTree.addAll(node.getChildren());

      System.out.println("Node Count = " + currentTree.size());

    }

    //if(currentParamNode.getPosition() == currentPosition){
    nextNode = getBestNode(currentNode.getChildren());
    if(!bestTree.contains(nextNode)) {
      bestTree.add(nextNode);

    }

    if(!currentTree.contains(nextNode)){
      currentTree.add(nextNode);
    }

    nextNode.setChildren(getChildren(nextNode));
    nextNode.setParent(currentNode);
    long end1 = System.nanoTime();

    currentTree.addAll(nextNode.getChildren());


    //Update Metrics
    this.nodeCount = currentTree.size();
    this.nodeExploredCount = bestTree.size();
    //this.nodeGenTime += end1 - start1;
    //this.cumulativeEuclidean+= euclidean.getDistanceBetween(currentNode.getPosition(), nextNode.getPosition());

  }

  /**
   * Converts angles
   */
  private void computeDriveMechanics() {
    double angleInDeg = turnAngle * (180/Math.PI);
    int timerRatio = (int)angleInDeg/10;
    System.out.println("Angle in DEG : " + angleInDeg);
    System.out.println("Timer Ratio : " + timerRatio);

  }

  /**
   * Starts the Algorithm
   * sets motor speeds to 0
   * Sets all locations
   * Generates initial sets of nodes
   */
  private void initAlgorithm(){
    setSpeed(0, 0);
    currentPosition = getCurrentPosition();
    targetPosition = new Position(DEFAULT_GOAL_PARAMETERS[0], DEFAULT_GOAL_PARAMETERS[1]);

    //currentParameters = new ControlParameters(DEFAULT_CONTROL_PARAMETERS);
    initRRT();

    //Build Tree
    buildTree();
    updateAngles();

  }

  /**
   * Initialises the first tree of te RRT
   */
  private void initRRT() {
    //Generate initial tree of the space
    this.currentNode = new Node(this.currentPosition, 0, null, null);

    long startTime1 = System.nanoTime();
    currentNode.setChildren(getChildren(currentNode));
    currentTree.addAll(currentNode.getChildren());
    long endTime1 = System.nanoTime();

    System.out.println("Getting inital Node...");
    currentTree.add(currentNode);
    bestTree.add(currentNode);
    System.out.println("Tree size : " + currentTree.size());

    if(currentTree.size() == 1){
      System.out.println("Start Node : " + currentTree.get(0));
      //System.out.println("Start Node Parent : " + currentTree.get(0).getParent().toString());
      System.out.println("Start Node Children : " + currentTree.get(0).getChildren().size());

    }

    //Update metrics
    //this.nodeGenTime += endTime1 - startTime1;
    this.nodeCount += currentNode.getChildren().size();
    this.nodeExploredCount+= 1;
  }

  /**
   * initialises metrics values
   */
  private void initMetrics(){
    //Metrics
    this.nodeGenTime = 0;
    this.distanceTravelled = 0;
    this.nodeDriveTime = 0;
    this.cumulativeEuclidean = 0;
    this.nodeCount = 0;
    this.nodeExploredCount = 0;
    this.cumulativeTurnAngle = 0;
  }

  /**
   * @return true if the current position is in goal range and vice versa
   */
  private boolean isGoalReached(){

    double distanceToGoal = euclidean.getDistanceBetween(currentPosition, targetPosition);

    if(distanceToGoal < DEFAULT_GOAL_RADIUS){
      return true;
    } else{
      return false;
    }


  }

  /**
   * @return double array of speeds for left and right motors
   * calculated from the current bearing to next sub goal
   */
  private double[] getSpeeds(){
    //add calculation for current to next node

    double speed = 0.230; //needs changing
    double length = 0.250; //""
    double radius = 0.110; //""

    System.out.println("Heading: "+ this.bearingAngle);
    double leftSpeed = ((2*speed) + (this.bearingAngle * length)) / (2*radius);
    double rightSpeed = ((2*speed) - (this.bearingAngle * length)) / (2*radius);

    //System.out.println("LSPEES: " +leftSpeed+ "RIRI: "+ rightSpeed);

    return new double[]{leftSpeed,rightSpeed};
  }

  /**
   * @param childPosition surrounding nodes
   * @return a numeric cost representing how viable a position is.
   */
  private double getChildCost(Position childPosition) {
    //System.out.println(this.obstacleCoordinates);
    double cost = 1;
    /*for (LidarPoint obstacle:
        obstacleCoordinates) {*/
    for(int i=0; i<obstacleCoordinates.length; i++){
      Position obstaclePosition = new Position(obstacleCoordinates[i].getX(), obstacleCoordinates[i].getZ());

      double distanceToNeighbor = euclidean.getDistanceBetween(childPosition, obstaclePosition);

      cost += distanceToNeighbor;
    }
    return cost;
  }

  /**
   * @param parameterNode node of the current position
   * @return list of possible child nodes from the current node as children
   */
  private List<Node> getChildren(Node parameterNode){


    List<ControlParameters> samples = new ArrayList<>(
        randomSampler.getSamplesFrom(parameterNode.getNodeParameters()));
    System.out.println(samples);

    List<Node> possibleChildren = new ArrayList<>(samples.size());

    for (int i = 0; i < samples.size() ; i++) {
      double[] vals = samples.get(i).getArray();
      Position childPosition = new Position(vals[0], vals[1]);
      double cost = getChildCost(childPosition);
      possibleChildren.add(new Node(childPosition, cost, parameterNode, new ArrayList<>(samples.size())));
    }
    return possibleChildren;
  }

  /**
   * @return current position of the robot, from the GPS object
   */
  private Position getCurrentPosition() {
    System.out.println("Getting Current GPS Location...");
    //return the current location on the gps
    System.out.println(this.gps.getValues()[0] + "gps " + this.gps.getValues()[2]);
    return new Position(this.gps.getValues()[0], this.gps.getValues()[2]);
  }

  private double getDistanceToGoal(){
    double[] gpsValues = gps.getValues();
    for (int i = 0 ; i < gpsValues.length ; i++){
      System.out.println ("gpsValues["+i+"]: "+gpsValues[i] );
      System.out.println  ( "target " + targetPosition ) ;
    }

    System.out.println  ( "Distance " + getDistBetween(new Position(gpsValues[0], gpsValues[2]), targetPosition) ) ;

    return getDistBetween(new Position(gpsValues[0], gpsValues[2]), targetPosition);
  }

  /**
   * @param origin current position
   * @param destination other position
   * @return euclidean distance between the two positions
   */
  private double getDistBetween(Position origin, Position destination) {
    //Position x  = new Position(0,0);
    //PositionAndHeading absolute = new PositionAndHeading(x, 0);
    // absolute = relativeToAbsolute(absolute);

    return euclidean.getDistanceBetween(origin, destination);
  }


  /**
   * Chooses the best next node
   * @param children List of all children from the current node
   * @return the node with the least cost
   */
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
    System.out.println ("Image : " + sensorImage[0]);
  }

  private void updateAngles() {
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
      System.out.println("RAD " + Math.atan(dy/dx));
      bearing = (0.75 * Math.PI) - Math.atan(dy/dx);

    }

    if (dx == 0){
      System.out.println("Dx == 0; Avoiding divide by 0...");

      if (dy > 0) bearing = 0;
      if (dy < 0) bearing = Math.PI;
      if (dy == 0) bearing = 0;
    }

    this.bearingAngle = bearing;
  }

  /**
   * Instantiates the metrics calculator and prints the metrics Summary
   */
  private void updateMetrics(){
    //init Metrics
    this.metricsCalculator = new MetricsCalculator(this.nodeGenTime,this.nodeDriveTime,
        this.distanceTravelled, this.nodeCount, this.nodeExploredCount, this.cumulativeEuclidean,
        this.cumulativeTurnAngle);

    //Print update to console
    this.metricsCalculator.getMetricsSummary();
  }
  private void terminate() {
    setSpeed(0, 0);
    isRunning = false;
    System.out.println("Controller Terminated");
    //Maybe add some stats
    return;
  }
}
