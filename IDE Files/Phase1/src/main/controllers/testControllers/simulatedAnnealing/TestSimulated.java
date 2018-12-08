package main.controllers.testControllers.simulatedAnnealing;
// File:          TestSimulated.java
// Date:
// Description:
// Author: 170027939@ST-ANDREWS UNIVERSITY
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.*;
import main.controllers.testControllers.metrics.MetricsCalculator;
import main.java.kak3.shp.algorithms.samplers.RandomSampler;
import main.java.kak3.shp.framework.*;
import main.java.kak3.shp.framework.distance.DistanceCalculator;
import main.java.kak3.shp.framework.distance.Euclidean;
import main.controllers.testControllers.node.Node;

import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

/**
 * Controller used to Implement the Simulated Annealing Algorithm within the WEBOTS application
 */
public class TestSimulated {

  private static final double DEFAULT_MAX_SPEED = 5;
  private static final int DEFAULT_MUPDATE_FREQUENCY = 1;
  private static final double DEFAULT_GOAL_RADIUS = 1; // meters.
  private static final double[] DEFAULT_CONTROL_PARAMETERS = { 1 , 1 };
  private static final double[] DEFAULT_GOAL_PARAMETERS = { 3.0 , 15.0 }; // goal {x, y}

  //Sampling
  private final int numSamples = 5;
  private final double  stepSize = 5; // meters.
  private final double minVal = 0; // meters.
  private final double maxVal = 10; // meters.

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

  //Angles
  private double currentHeading = 0;
  private double turnAngle = 0;
  private double bearing = 0;

  //Other
  private DistanceCalculator distanceCalculator;

  //Obstacles
  private LidarPoint[] obstacleCoordinates;

  //Algorithms
  private final Random random = new Random();
  private final Euclidean euclidean = new Euclidean();
  private RandomSampler randomSampler = new RandomSampler(numSamples, stepSize, minVal, maxVal);
  private MetricsCalculator metricsCalculator;

  //Nodes
  private Node startNode, currentNode, nextNode;
  private List<Node> currentTree = new LinkedList<>();
  private List<Node> bestTree = new LinkedList<>();

  //Control Parameters
  private ControlParameters currentParameters, nextParameters;

  //Metrics
  private long nodeGenTime;
  private long nodeDriveTime;
  private double distanceTravelled;
  private double cumulativeEuclidean;
  private double cumulativeTurnAngle;
  private int nodeCount;
  private int nodeExploredCount;

  //Threads
  private Thread initThread, nodeGenThread, nodeDriveThread;

  //Thread Manager
  ThreadMXBean threadMXBean;

  /**
   * Simulated Annealing implementation for WEBOTS
   * @param gps Robots GPS object
   * @param compass Robots compass object
   * @param emitter Robots emitter object
   * @param frontLeft Robots frontLeft Motor object
   * @param frontRight Robots frontRight Motor object
   * @param backLeft Robots backLeft Motor object
   * @param backRight Robots backRight Motor object
   */
  public TestSimulated(GPS gps, Compass compass, Emitter emitter,
                       Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {

    //Motors
    this.motors[0] = frontLeft;
    this.motors[1] = frontRight;
    this.motors[2] = backLeft;
    this.motors[3] = backRight;

    //Devices
    this.gps = gps;
    this.compass = compass;
    this.emitter = emitter;

    //Thread Manager
    this.threadMXBean = ManagementFactory.getThreadMXBean();

  }


  public static void main(String[] args){

    // Create the Robot instance.
    Robot robot = new DifferentialWheels();

    // Get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    System.out.println("BASIC TIME STEP: " + timeStep + "ms.");

    // Get references to, and enable, all required devices on the robot.
    Lidar lidar = robot.getLidar("lms291");
    lidar.enable(timeStep);
    lidar.enablePointCloud();

    GPS gps = robot.getGPS("gps");
    gps.enable(timeStep);


    Emitter emitter = robot.getEmitter("emitter");
    emitter.setChannel(0);

    //Compass
    Compass compass = robot.getCompass("compass");
    compass.enable(timeStep);

    //Motors
    Motor frontLeft = robot.getMotor("front_left_wheel");
    frontLeft.setPosition(Double.POSITIVE_INFINITY);
    Motor frontRight = robot.getMotor("front_right_wheel");
    frontRight.setPosition(Double.POSITIVE_INFINITY);
    Motor backLeft = robot.getMotor("back_left_wheel");
    backLeft.setPosition(Double.POSITIVE_INFINITY);
    Motor backRight = robot.getMotor("back_right_wheel");
    backRight.setPosition(Double.POSITIVE_INFINITY);

    //Controller setup
    TestSimulated controller = new TestSimulated(gps, compass, emitter, frontLeft,frontRight, backLeft, backRight);

    robot.step(timeStep);
    controller.initSensorReadings(lidar.getPointCloud());
    controller.initMetrics();
    controller.runInit();
    //controller.initAlgorithm();

    //Threads
    //InitThread initThread = new InitThread(controller);
    //NodeGen nodeGen = new NodeGen(controller);
    //NodeDrive nodeDrive = new NodeDrive(controller);

    //initThread.start();

    //System.out.println("TTTTTTTT::::: "+ controller.threadMXBean.getThreadCpuTime(initThread.getId()) );

    //
    controller.isRunning = true;

    //Iteration Count for frequency
    int iterationCount = 0;

    while (robot.step(timeStep*100) != -1) {
      System.out.println("BASIC TIME STEP: " + timeStep + "ms.");

      if(temperature <1 ){
        System.out.println("Temperature is Low.");
        System.out.println("System Terminating...");
        controller.terminate();

      }

      if(!controller.isRunning ){
        System.out.println("Controller is not running.");
        System.out.println("System Terminating...");
        controller.terminate();

      }

      if(controller.isGoalReached()) {

        System.out.println("Goal Point Reached");
        System.out.println("System Terminating...");
        System.out.println("Iterations Made : " + iterationCount);
        controller.terminate();

      }

      if (!controller.isGoalReached()){
        //Generate Random Solution
        controller.initSensorReadings(lidar.getPointCloud());

        controller.updateAngles();
        controller.runNodeGen();
        controller.runNodeDrive();
        //controller.generateNextNode();
        //controller.driveToNextNode();

        //Metrics
        controller.updateMetrics();
        iterationCount++;
      }
    }
  }

  /**
   * runs thread for algorithm initialisation
   */
  private void runInit(){

    this.initThread = new Thread(this::initAlgorithm);
    long initID = this.initThread.getId();
    this.initThread.run();

    this.nodeGenTime += this.threadMXBean.getThreadCpuTime(initID);
    //System.out.println("TRYYYYYYYYYYYTIIINIT: "+ this.nodeGenTime);

  }

  /**
   * Runs thread for node Generation
   */
  private void runNodeGen(){
    this.nodeGenThread = new Thread(this::generateNextNode);
    long nodeGenID = this.nodeGenThread.getId();
    this.nodeGenThread.run();

    this.nodeGenTime += this.threadMXBean.getThreadCpuTime(nodeGenID);
    //System.out.println("TRYYYYYYYYYYYTIIINODE-GEN: "+ this.nodeGenTime);
  }

  /**
   * Runs Thread for Controller to implement drive current node to the next
   */
  private void runNodeDrive(){
    this.nodeDriveThread = new Thread(this::driveToNextNode);
    long nodeDriveId = this.nodeDriveThread.getId();
    this.nodeDriveThread.run();

    this.nodeDriveTime += this.threadMXBean.getThreadCpuTime(nodeDriveId);
    //System.out.println("TRYYYYYYYYYYYTIIInoDEdrIVE: "+ this.nodeDriveTime);
  }

  /**
   * Starts the Algorithm
   * sets motor speeds to 0
   * Sets all locations
   * Generates initial sets of nodes
   */
  public void initAlgorithm(){

    setSpeed(0, 0);
    this.currentPosition = getCurrentLocation();

    System.out.println("Positions:  "+this.currentPosition);
    targetPosition = new Position(DEFAULT_GOAL_PARAMETERS[0], DEFAULT_GOAL_PARAMETERS[1]);
    //currentParameters = new ControlParameters(DEFAULT_CONTROL_PARAMETERS);
    //getNextSolution(currentPosition);

    //init Nodes
    this.startNode = new Node(this.currentPosition,0, null,null);
    this.startNode.setPosition(this.currentPosition);
    System.out.println(startNode.getPosition());
    System.out.println(startNode.getNodeParameters());
    startNode.setChildren(getChildren(startNode));

    //Add to travelled/best nodes list
    bestTree.add(startNode);

    //set/init current node
    if((this.currentNode == null) || (this.currentNode != this.startNode)){
      this.currentNode = this.startNode;
      this.currentPosition = currentNode.getPosition();
    }

    if((this.nextNode == null) || (this.nextNode != currentNode.getChildren().get(0))){
      this.nextNode = currentNode.getChildren().get(0);

    }

    updateAngles();
  }

  /**
   * Updates the Obstacle reading in a given operational space
   * @param sensorImage Lidar image array from the Lidar Object
   */
  private synchronized void initSensorReadings(LidarPoint[] sensorImage) {
    obstacleCoordinates = sensorImage;
    System.out.println ("Lidar Sensor Image " + sensorImage[0].toString());
  }

  /**
   * Initiates all arguments for instantiating the metrics calculator
   */
  private void initMetrics(){
    //Metrics
    this.nodeGenTime = 0;
    this.nodeDriveTime = 0;
    this.distanceTravelled = 0;
    this.cumulativeEuclidean = 0;
    this.nodeCount = 0;
    this.nodeExploredCount = 0;
    this.cumulativeTurnAngle = 0;
  }

  /**
   * @return double array of speeds for left and right motors
   * calculated from the current bearing to next sub goal
   */
  private double[] getSpeeds(){
    double speed = 0.230; //needs changing
    double length = 0.250; //""
    double radius = 0.110; //""

    updateAngles();

    System.out.println("Heading: "+ this.bearing); // might change heading to calculate for a new turn angle
    double leftSpeed = ((2*speed) + (this.bearing * length)) / (2*radius);
    double rightSpeed = ((2*speed) - (this.bearing * length)) / (2*radius);

    System.out.println("L-Speed: " +leftSpeed+ "R-Speed: "+ rightSpeed);

    return new double[]{leftSpeed,rightSpeed};
  }

  /**
   * @param currentPosParameters current node's position parameters
   * @return a collection of control parameters as samples for a possible next node
   */
  private ControlParameters getNextPosParameters(ControlParameters currentPosParameters){
    List<ControlParameters> samples = new LinkedList<>(randomSampler.getSamplesFrom(currentPosParameters));

    ControlParameters bestNeighbor = samples.get(random.nextInt(samples.size()));
    double bestCost = getCost(bestNeighbor.getArray());

    for (int i = 0; i < samples.size(); i++) {
      double currentCost = getCost(samples.get(i).getArray());

      if((currentCost < bestCost) || (isCostAcceptable(bestCost, currentCost))){
        //Add to final list if cost
        bestNeighbor = samples.get(i);
      }
    }

    if(!bestNeighbors.contains(bestNeighbor)) {
      this.bestNeighbors.add(bestNeighbor);
    }
    return bestNeighbor;
  }

  /**
   * @param current node of the current position
   * @return list of possible child nodes from the current node as children
   */
  private List<Node> getChildren(Node current){
    List<Node> children = new LinkedList<>();

    long start = System.nanoTime();
    children.add(getNextNode(current));
    long end = System.nanoTime();

    this.nodeGenTime += (end - start);

    return children;
  }

  /**
   * @param current node of the current position
   * @return next sub goal node from the current node
   */
  private Node getNextNode(Node current){
    this.nextParameters = getNextPosParameters(current.getNodeParameters());
    this.nextPosition = new Position(this.nextParameters.getArray()[0],this.nextParameters.getArray()[1]);

    Node next = new Node(this.nextPosition, getCost(nextPosition.toArray()),current, null);

    return next;
  }

  /**
   * @param neighbor surrounding nodes
   * @return a numeric cost representing how viable a position is
   */
  private double getCost(double[] neighbor){
    Position neighborPosition = new Position(neighbor[0], neighbor[1]);
    //Cost function euclidean for now
    double cost = 0;
    for (LidarPoint obstacle:
        obstacleCoordinates) {
      Position obstaclePosition = new Position(obstacle.getX(), obstacle.getZ());

      double distanceToNeighbor = euclidean.getDistanceBetween(neighborPosition, obstaclePosition);

      cost += distanceToNeighbor;
    }
    return cost;
  }

  /**
   * @return current position of the robot, from the GPS object
   */
  private Position getCurrentLocation(){
    Position location = new Position(gps.getValues()[0], gps.getValues()[2]);
    System.out.println(location);

    return location;
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

    System.out.println("Motor Speeds Set: L = " + left + " R = " + right);
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
   *
   * @param bestCost Current Best Cost
   * @param nextCost Next Possible
   * @return
   */
  private boolean isCostAcceptable(double bestCost, double nextCost){
    boolean accept = false;
    //Compare Costs
    if(nextCost < bestCost){
      //Accept
      accept = true;

    } else {
      //calculate the probability
      double acceptProb = Math.exp(energyCoefficient * (bestCost - nextCost) / temperature);
      //Decide on acceptance
      if(acceptProb > random.nextDouble()){
        accept = true;

      }
    }
    return accept;
  }

  /**
   * generates the next node
   */
  private void generateNextNode(){
    //Re-adjust current position
    long nodeGen1 = System.nanoTime();
    currentPosition = getCurrentLocation();
    if(this.currentNode.getPosition() != currentPosition){
      //this.currentPosition = getCurrentLocation();
      this.currentNode.setPosition(currentPosition);
      this.currentNode.setChildren(getChildren(currentNode));
    }

    //Re-sample to get new next neighbor
    nextNode = this.currentNode.getChildren().get(0);
    long nodeGen2 = System.nanoTime();


    //Add nodes to list
    bestTree.add(currentNode);
    System.out.println("NEW Best Tree Size: "+bestTree.size());

    //Metrics
    //this.nodeGenTime += nodeGen2-nodeGen1;


  }

  /**
   * implements controller dive to next node
   */
  private void driveToNextNode(){
    //Get new speeds for next destination
    long nodeDrive1 = System.nanoTime();
    double[] speeds = getSpeeds();

    //Set speeds
    setSpeed(speeds[0], speeds[1]);
    System.out.println("Driving...");
    long nodeDrive2 = System.nanoTime();

    //Update the metrics counts

    //this.nodeDriveTime += nodeDrive2 - nodeDrive1;
    this.nodeExploredCount +=1;
    this.nodeCount = bestTree.size();
    this.cumulativeTurnAngle += this.bearing;
    System.out.println("Current Bearing (Randians):: " + this.bearing);
    this.cumulativeEuclidean += euclidean.getDistanceBetween(currentNode.getPosition(), nextNode.getPosition());


    //Update Metrics
    //updateMetrics();
  }

  /**
   * implements the update of metrics using the MetricsCalculator class
   */
  private void updateMetrics(){
    //init Metrics
    this.metricsCalculator = new MetricsCalculator(this.nodeGenTime,this.nodeDriveTime,
        this.distanceTravelled, this.nodeCount, this.nodeExploredCount, this.cumulativeEuclidean, this.cumulativeTurnAngle);

    //Print update to console
    this.metricsCalculator.getMetricsSummary();
  }

  /**
   * Updates all Turn Angles
   */
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

    System.out.println("Angles-1 Check: "+ this.currentHeading);


    //Update difference to position
    double dx = 0.0;
    double dy = 0.0;
    double bearing = 0;
    if(this.currentNode != null && this.nextNode != null ){
      //System.out.println("Nodes Not Null");
      dx = this.nextNode.getX() - this.currentNode.getX();
      dy = this.nextNode.getY() - this.currentNode.getY();

    }

    if (dx > 0 ){
      System.out.println("RADD CHECK " + Math.atan(dy/dx));
      bearing = (Math.PI/2) - Math.atan(dy/dx);

    }

    if (dx > 0 ){
      System.out.println("RADD CHECK " + Math.atan(dy/dx));
      bearing = (0.75 * Math.PI) - Math.atan(dy/dx);

    }

    if (dx == 0){
      if (dy > 0) bearing = 0;
      if (dy < 0) bearing = Math.PI;
      if (dy == 0) bearing = 0;
    }

    this.bearing = bearing;
    //System.out.println("Angles-2Check: "+ this.bearing);
  }

  /**
   * Terminates the controller and stops the robot.
   */
  private void terminate(){
    setSpeed(0, 0);
    System.out.println("Controller Terminated");
    //Maybe add some stats

  }

}


