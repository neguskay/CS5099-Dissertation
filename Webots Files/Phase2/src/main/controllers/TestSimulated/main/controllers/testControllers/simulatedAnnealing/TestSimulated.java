package main.controllers.testControllers.simulatedAnnealing;


import com.cyberbotics.webots.controller.*;
import main.java.kak3.shp.algorithms.samplers.RandomSampler;
import main.java.kak3.shp.framework.*;
import main.java.kak3.shp.framework.distance.DistanceCalculator;
import main.java.kak3.shp.framework.distance.Euclidean;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;


public class TestSimulated {

  private static final double DEFAULT_MAX_SPEED = 5;
  private static final double DEFAULT_SENSOR_DISTANCE = 4;//meters
  private static final int DEFAULT_MUPDATE_FREQUENCY = 1;
  private static final double DEFAULT_GOAL_RADIUS = 1; // meters.
  private static final double DEFAULT_OBSTACLE_POTENTIAL_RATIO = 2.5;
  private static final double[] DEFAULT_CONTROL_PARAMETERS = { 1 , 1 };
  private static final double[] DEFAULT_GOAL_PARAMETERS = { 1 , 1 }; // goal {x, y}

  //Sampling
  private final int numSamples = 5;
  private final double  stepSize = 3; // meters.
  private final double minVal = -2.5; // meters.
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


  //Control Parameters
  private ControlParameters currentParameters, nextParameters;

  public TestSimulated(GPS gps, Compass compass, Emitter emitter,
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

    controller.initAlgorithm();
    robot.step(timeStep);
    controller.initSensorReadings(lidar.getPointCloud());
    controller.isRunning = true;


    //Iteration Count for frequency
    int iterationCount = 0;

    while (robot.step(timeStep*1000) != -1) {

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
        controller.terminate();

      }

      if (!controller.isGoalReached()){
        //Generate Random Solution
        controller.initSensorReadings(lidar.getPointCloud());
        controller.nextParameters = controller.getNextPosParameters(controller.currentParameters);

        //Keep moving even without a solution
        if(controller.nextParameters == controller.currentParameters){
          //randomly select a new location
          controller.nextParameters = controller.bestNeighbors.get(
              controller.random.nextInt(controller.bestNeighbors.size()));

          System.out.print("Changed location");
        }

        //Set speed as control parameters
        controller.setSpeed(controller.nextParameters.getArray()[0],controller.nextParameters.getArray()[1]);

      }
    }
  }

  private void initAlgorithm(){
    setSpeed(0, 0);
    currentPosition = getCurrentLocation();
    targetPosition = new Position(DEFAULT_GOAL_PARAMETERS[0], DEFAULT_GOAL_PARAMETERS[1]);

    currentParameters = new ControlParameters(DEFAULT_CONTROL_PARAMETERS);
    //getNextSolution(currentPosition);
    initWind();

  }

  private void initWind(){
    //Calculate heading in radians
    double nextHeading = Math.PI + Math.atan2(compass.getValues()[2], compass.getValues()[0]);

    //Check and set new heading/wind
    if(currentHeading != null){

      double angleDifference = currentHeading - nextHeading;

      if(angleDifference > Math.PI){
        angleDifference = -1 * (2 * Math.PI - angleDifference);

      } else if (angleDifference < - Math.PI){
        angleDifference = 2 * Math.PI + angleDifference;

      }

      turnAngle += angleDifference;
      System.out.println("Turn Angles initialised");
    }
    currentHeading = nextHeading;
  }


  private synchronized void initSensorReadings(LidarPoint[] sensorImage) {
    obstacleCoordinates = sensorImage;
    System.out.println ("Lidar Sensor Image " + sensorImage[0].toString());
  }

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

  private Position getCurrentLocation(){
    Position location = new Position(gps.getValues()[0],gps.getValues()[1]);
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


  private void terminate(){
    setSpeed(0, 0);
    System.out.println("Controller Terminated");
    //Maybe add some stats
  }


}
