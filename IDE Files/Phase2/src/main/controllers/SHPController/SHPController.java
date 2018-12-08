/import com.cyberbotics.webots.controller.*;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.Namespace;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.MoveSequence;
import uk.ac.st_andrews.cs.host.kak3.SHP.application.Application;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.BasicFramework;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Position;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.PositionAndHeading;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.distance.*;
import java.util.*;
import java.util.Arrays;
import java.util.List;

public class SHPController extends BasicFramework implements Application {
    private static final double AXLE_LENGTH = 0.6;
    private static final double MAX_SPEED = 5; //default = 2.08
    private static final double MAX_WHEEL_VALUE = 20;

    /**
     * How many frames of simulation should occur between each call of the algorithm.
     */
    private static final int UPDATE_FREQUENCY = 1;

    /**
     * Distance from the goal that we consider to be solved.
     */
    private static final double GOAL_RADIUS = 1; // meters.

    /**
     * Maximum range of the sensors.
     */
    private static final double SENSOR_DISTANCE = 4;

    private static double OBSTACLE_COEFFICIENT;
    private static double GOAL_COEFFICIENT;
    private static double LOOKAHEAD_TIME;

    private double wind = 0;
    private Double previousDirection = null;

    // Robot components required for gathering data or setting commands.
    private final Motor[] motors = new Motor[4];
    private final GPS gps;
    private final Compass compass;

    // Used for communication with the physics plugin.
    private final Emitter emitter;

    private Position targetPosition;
    private boolean running = false;

    private DistanceCalculator distanceCalculator;

    /**
     * The point cloud of detected obstacles, in (x, y, z) relative to the robot.
     */
    private LidarPoint[] obstaclePoints;

    public static ArgumentParser getParser() {
        ArgumentParser parser = BasicFramework.getParser();
        parser.addArgument("--obstacle-coefficient")
                .type(Double.class)
                .setDefault(1d)
                .help("Coefficient for obstacle potential.");
        parser.addArgument("--goal-coefficient")
                .type(Double.class)
                .setDefault(1.8d)
                .help("Coefficient for goal potential.");
        parser.addArgument("--lookahead-time")
                .type(Double.class)
                .setDefault(1.5d)
                .help("Time period to look ahead for forwards kinematics cost calculations.");
        parser.addArgument("--target")
                .type(String.class)
                .setDefault("0,0")
                .help("The target x,y co-ordinate.");
        parser.addArgument("--distance-metric")
                .choices("euclidean", "two-arc", "three-arc")
                .help("The distance metric to use for calculating cost to the goal.")
                .setDefault("euclidean");
        return parser;
    }

    private void handleArgs(Namespace args) {
        OBSTACLE_COEFFICIENT = args.getDouble("obstacle_coefficient");
        GOAL_COEFFICIENT = args.getDouble("goal_coefficient");
        LOOKAHEAD_TIME = args.getDouble("lookahead_time");

        String target = args.getString("target");
        String[] components = target.split(",");
        targetPosition = new Position(Double.parseDouble(components[0]), Double.parseDouble(components[1]));

        String metric = args.getString("distance_metric");
        DistanceCalculator calculator;
        switch (metric) {
            case "two-arc":
                calculator = new TwoArc();
                break;
            case "three-arc":
                calculator = new ThreeArc();
                break;
            default:
                calculator = new Euclidean();
        }

        distanceCalculator = calculator;
    }

    /**
     * Instantiates a new Webots controller to interface with the framework.
     * @param args the command line arguments passed in the controllerArgs field in webots.
     * @param gps the GPS device.
     * @param compass the compass device.
     * @param frontLeft the front left motor.
     * @param frontRight the front right motor.
     * @param backLeft the back left motor.
     * @param backRight the back right motor.
     * @param emitter the radio emitter, for communicating with the physics plugin.
     */
    private SHPController(Namespace args, GPS gps, Compass compass, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, Emitter emitter) {
        super(args);
        handleArgs(args);

        this.gps = gps;
        this.compass = compass;
        motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;
        this.emitter = emitter;
    }

    @Override
    public void run() {

    }

    public static void main(String[] args) {
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

        // Parse the arguments and instantiate a controller for interfacing with the framework.
        Namespace ns = parseArgs(args, getParser());
        //System.out.println("ns : " + ns);
        SHPController controller = new SHPController(ns, gps, compass, frontLeft, frontRight, backLeft, backRight, emitter);
        System.out.println("current param  : " +controller.currentParams );
        double []  a = { 1 , 1 } ; 
        controller.currentParams = new ControlParameters  (a) ;
        controller.setSpeed(0, 0);
        robot.step(timeStep);
        controller.updateSensorReadings(lidar.getPointCloud());
        controller.running = true;
        //for ( int  k = 0 ; k < 1 ; k++)
        // robot.step(timeStep);
        //Scanner s = new Scanner ( System.in ) ; 
        //String g = s.next();

        // Send the goal position to the physics plugin, for rendering purposes.
        Position target = controller.targetPosition;
        controller.sendRenderMessage(String.format("G,%f,%f\n", target.x, target.y));

        controller.estimateCostAtPosition(new PositionAndHeading(new Position(0, 0), 0));
        
        int i = 0;

        // Main loop, reads sensor data, calls framework and updates wheel speeds based on the result.
        while (robot.step(timeStep*100) != -1) {
            controller.updateWinding();

            // Update sensor readings.
            controller.updateSensorReadings(lidar.getPointCloud());

            if (controller.distToGoal() < GOAL_RADIUS) {
                controller.algorithm.terminate(controller);
            }

            if (i % UPDATE_FREQUENCY == 0) {
                controller.iteration();
                System.out.println("init");
            }

            if (!controller.running) {
                controller.setSpeed(0, 0);
                System.out.println("Stopped");
                return;
            }
            System.out.println ( " prob " ) ; 
            double[] speeds = controller.currentParams.getArray();
            controller.setSpeed(speeds[0], speeds[1]);
            System.out.println("Timestep: " + i + ". Running with speed: " + Arrays.toString(controller.currentParams.getArray()));
            System.out.println("Current cost: " + controller.getCurrentCost());
            PositionAndHeading relative = calculateForwardKinematics(new PositionAndHeading(new Position(0, 0), 0), speeds[0], speeds[1], LOOKAHEAD_TIME);
            System.out.println("Expected (relative) location in " + LOOKAHEAD_TIME + " seconds: " + relative + ", absolute: " + controller.relativeToAbsolute(relative));

            i++;
        }

        controller.finished();
    }

    @Override
    public boolean canMoveDirectly() {
        return false;
    }

    @Override
    public ControlParameters initialise() {
        double[] vals = {0.1, 0.1};
        return new ControlParameters(vals);
    }

    private void updateWinding() {
        double[] compassValues = compass.getValues();
        double direction = Math.PI + Math.atan2(compassValues[2], compassValues[0]);
        if (previousDirection != null) {
            double angle = previousDirection - direction;
            if (angle > Math.PI) {
                angle = -1 * (2*Math.PI - angle);
            }
            else if (angle < -Math.PI) {
                angle = 2*Math.PI + angle;
            }
            wind += angle;
        }

        previousDirection = direction;
    }

    private double distToGoal() {
        double[] gpsValues = gps.getValues();
        for (int i = 0 ; i < gpsValues.length ; i++) 
        System.out.println ("gpsValues["+i+"]: "+gpsValues[i] );
        System.out.println  ( "target " + targetPosition ) ; 
                System.out.println  ( "Distance " + distBetween(new PositionAndHeading(new Position(gpsValues[0], gpsValues[2]), 0), targetPosition) ) ; 

        return distBetween(new PositionAndHeading(new Position(gpsValues[0], gpsValues[2]), 0), targetPosition);
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

    /**
     * Convenience method for communicating with the physics plugin. Null pads and the string, and sends it through
     * the radio emitter.
     * @param msg the message string to send.
     */
    private void sendRenderMessage(String msg) {
        emitter.send((msg + "\0").getBytes());
    }

    private double distBetween(PositionAndHeading origin, Position destination) {
    Position x  = new Position(0,0);
    PositionAndHeading absolute = new PositionAndHeading(x, 0); 
   // absolute = relativeToAbsolute(absolute);

        return distanceCalculator.getDistBetween(absolute, origin.position, destination);
    }

    private synchronized void updateSensorReadings(LidarPoint[] image) {
        obstaclePoints = image;
        System.out.println ("image " + image[0] );
    }

    /**
     * Gets the cost of a position and heading relative to the robot.
     * @param position the position and heading relative to the robot's current position and heading.
     * @return the estimated cost.
     */
    private synchronized double estimateCostAtPosition(PositionAndHeading position) {
        PositionAndHeading absolute = relativeToAbsolute(position);

        // If we're close enough to the goal, we consider our cost to be 0. This ensures that the algorithm can always
        // know it's finished.
        if (Euclidean.euclideanDistance(relativeToAbsolute(position).position.toArray(), targetPosition.toArray()) < GOAL_RADIUS) {
            return 0;
        }

        // Calculate the obstacle potential field, by taking the distance to all known obstacles.
        double obstacleField = 0;
        for (LidarPoint obstacle : obstaclePoints) {
            Position obstaclePos = relativeToAbsolute(new PositionAndHeading(new Position(-obstacle.getX(), -obstacle.getZ()), 0)).position;
            double dist = Euclidean.euclideanDistance(obstaclePos.toArray(), absolute.position.toArray());

            // Exclude obstacles beyond our sensor range, or all obstacles if the coefficient is 0.
            if (dist >= SENSOR_DISTANCE || OBSTACLE_COEFFICIENT == 0) {
                continue;
            }

            // Cost goes up to infinity at 0 distance.
            double cost = Math.pow(Math.E, -1 / (SENSOR_DISTANCE - dist)) / dist;
            obstacleField += cost;
        }

        // The goal field is simply the distance remaining to the goal, by whatever metric we're using.
        double goalField = distBetween(absolute, targetPosition);

        // Multiply both components by a coefficient supplied by the user.
        double goalPotential = GOAL_COEFFICIENT * Math.pow(goalField, 2);
        double obstaclePotential = OBSTACLE_COEFFICIENT * obstacleField;

        return goalPotential + obstaclePotential;
    }

    /**
     * Converts a position relative to the robot, to be in absolute co-ordinates (and heading).
     * @param position the relative position and heading.
     * @return the absolute position and heading.
     */
    private PositionAndHeading relativeToAbsolute(PositionAndHeading position) {
        double[] gpsValues = gps.getValues();
        double[] compassValues = compass.getValues();
        double direction = Math.PI + Math.atan2(compassValues[2], compassValues[0]);

        double translatedX = position.position.x * Math.cos(direction) + position.position.y * Math.sin(direction);
        double translatedY = -position.position.x * Math.sin(direction) + position.position.y * Math.cos(direction);

        return new PositionAndHeading(new Position(translatedX + gpsValues[0], translatedY + gpsValues[2]), (direction + position.heading) % (Math.PI * 2));
    }

    @Override
    public double getCurrentCost() {
        return estimateCostAtPosition(new PositionAndHeading(new Position(0, 0), 0));
    }

    @Override
    public void doMove(ControlParameters parameters) {
        currentParams = parameters;
    }

    /**
     * Estimates the cost of the given move parameters from the given origin.
     * @param parameters the move parameters to cost.
     * @param origin the start co-ordinate of the robot.
     * @return the cost of that point.
     */
    private double getCost(ControlParameters parameters, PositionAndHeading origin) {
        double[] params = parameters.getArray();
        PositionAndHeading positionAndHeading = calculateForwardKinematics(origin, params[0], params[1], LOOKAHEAD_TIME);
        return estimateCostAtPosition(positionAndHeading);
    }

    @Override
    public double simulateMove(ControlParameters parameters) {
        // Simply calls getCost with the provided moves, and the origin, as this will give a move relative to the robot.
        return getCost(parameters, new PositionAndHeading(new Position(0, 0), 0));
    }

    /**
     * To cost a move sequence, execute them in turn and determine where we'd end up after each - passing that
     * as the start to the next move.
     * @param sequence the sequence.
     * @return the cost of the sequence.
     */
    @Override
    public double simulateMoveSequence(MoveSequence sequence) {
        PositionAndHeading current = new PositionAndHeading(new Position(0, 0), 0);
        for (ControlParameters parameters : sequence.getParameters()) {
            double[] speeds = parameters.getArray();
            current = calculateForwardKinematics(current, speeds[0], speeds[1], LOOKAHEAD_TIME);
        }

        return estimateCostAtPosition(current);
    }

    @Override
    public void finished() {
        running = false;
        if (stats != null) {
            stats.dump();
        }
    }

    @Override
    public int getParamCount() {
        return 2;
    }

    /**
     * Combining a move sequence in control space can be approximated by simply adding the components of the vectors.
     * @param parameters the list of parameters to combine.
     * @return the combined parameters.
     */
    @Override
    public ControlParameters combineMoves(List<ControlParameters> parameters) {
        double[] vals = new double[parameters.get(0).getArray().length];

        for (ControlParameters params : parameters) {
            for (int i = 0; i < params.getArray().length; i++) {
                vals[i] += params.getArray()[i];
            }
        }

        return new ControlParameters(vals);
    }

    /**
     * Calculates a position and heading given the current position and heading, the wheel speeds, and the time step,
     * using forwards kinematics equations for a differential drive robot, as described here
     * https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
     * @param origin the starting point of the robot.
     * @param leftWheel the left wheel speed.
     * @param rightWheel the right wheel speed.
     * @param timestep the time step to move for in seconds.
     * @return the new estimated position and heading.
     */
    private static PositionAndHeading calculateForwardKinematics(PositionAndHeading origin, double leftWheel, double rightWheel, double timestep) {
        double R;

        leftWheel = (leftWheel / MAX_WHEEL_VALUE) * MAX_SPEED;
        rightWheel = (rightWheel / MAX_WHEEL_VALUE) * MAX_SPEED;

        if (rightWheel == leftWheel) {
            return new PositionAndHeading(
                    new Position(origin.position.x + ((leftWheel) * timestep) * Math.sin(origin.heading),
                            (origin.position.y + ((leftWheel) * timestep) * Math.cos(origin.heading))),
                    origin.heading);
        }
        else {
            R = (AXLE_LENGTH * (leftWheel + rightWheel)) / ((rightWheel - leftWheel) * 2d);
        }

        double w = (rightWheel - leftWheel) / AXLE_LENGTH;

        double iccx = origin.position.x - R * Math.sin(origin.heading);
        double iccy = origin.position.y + R * Math.cos(origin.heading);

        double[][] matrixData1 = {
                {Math.cos(w * timestep), -Math.sin(w * timestep), 0},
                {Math.sin(w * timestep), Math.cos(w * timestep), 0},
                {0, 0, 1}
        };
        RealMatrix matrix1 = MatrixUtils.createRealMatrix(matrixData1);

        double[][] matrixData2 = {
                {origin.position.x - iccx},
                {origin.position.y - iccy},
                {origin.heading}};
        RealMatrix matrix2 = MatrixUtils.createRealMatrix(matrixData2);

        double[][] matrixData3 = {{iccx}, {iccy}, {w * timestep}};
        RealMatrix matrix3 = MatrixUtils.createRealMatrix(matrixData3);

        RealMatrix result = matrix1.multiply(matrix2).add(matrix3);
        double[] resultArray = result.getColumn(0);

        return new PositionAndHeading(new Position(resultArray[1], resultArray[0]), resultArray[2]);
    }
}
