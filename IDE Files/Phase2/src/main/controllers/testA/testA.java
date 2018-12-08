// File:          testA.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.*;
import java.util.*;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class testA {
  //Timers
  private Timer timer;
  private TimerTask timerTask;
  
  
  //Motor
  private final Motor[] motors = new Motor[4];
  private final Compass compass;
  
  public testA(Motor fl,Motor fr,Motor bl,Motor br, Compass compass){
    this.motors[0] = fl;
    this.motors[1] = fr;
    this.motors[2] = bl;
    this.motors[3] = br;
    this.compass = compass;
    
  }
   
  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {
    
    
    Robot robot = new DifferentialWheels();
    System.out.println("Diff Wheels Has been setup");

    // Get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    timeStep = (timeStep*1000)/32; 
     
    //
    Motor frontLeft = robot.getMotor("front_left_wheel"); 
    Motor frontRight = robot.getMotor("front_right_wheel");
    Motor backLeft = robot.getMotor("back_left_wheel");
    Motor backRight = robot.getMotor("back_right_wheel");
    
    //Motor frontLeft = robot.getMotor("front_left_wheel");
    frontLeft.setPosition(Double.POSITIVE_INFINITY);
    //Motor frontRight = robot.getMotor("front_right_wheel");
    frontRight.setPosition(Double.POSITIVE_INFINITY);
    //Motor backLeft = robot.getMotor("back_left_wheel");
    backLeft.setPosition(Double.POSITIVE_INFINITY);
    //Motor backRight = robot.getMotor("back_right_wheel");
    backRight.setPosition(Double.POSITIVE_INFINITY);
    
    System.out.println("Motors Has been setup");
    
    
    //
    Lidar lidar = robot.getLidar("lms291");
    lidar.enable(timeStep);
    lidar.enablePointCloud();

    GPS gps = robot.getGPS("gps");
    gps.enable(timeStep);

    Emitter emitter = robot.getEmitter("emitter");
    emitter.setChannel(0);

    Compass compass = robot.getCompass("compass");
    compass.enable(timeStep);

    //Motors
    testA ta = new testA(frontLeft, frontRight, backLeft, backRight, compass);
    //initMotors();
    double dist=0;
    int i = 20;
    while (robot.step(timeStep) != -1) {

//robot.step(timeStep) != -1
        //frontLeft.setVelocity(3.5);
        //frontRight.setVelocity(5.0);
        //backLeft.setVelocity(3.5);
        //backRight.setVelocity(0.5);
        //ta.setSpeed(0.5, -0.5);
        //ta.setSpeed(1, 0.8);
        //ta.setSpeed(2, 1);
        System.out.println(timeStep);
        System.out.println(robot.step(timeStep));
        System.out.println(robot.step(1));
        System.out.println(robot.step(0));
        //ta.setSpeed(0.0,0.0);
        //ta.setSpeed(3.0,2.5);
        //ta.setSpeed(0.0,0.0);
        
        //ta.setSpeed(1,1);
        System.out.println("Wheels have moved");
        dist+= 4*3.142*0.111/3; // every second - roughly
        System.out.println("Distance: "+ dist );
        
        //Timer Try
        ta.tryTimer();
        System.out.println("Distance: "+ dist );
        
        //Compass Test
        ta.getTurnAngle();
        
        //i-=1;
      }
    
    

    // Enter here exit cleanup code.
  }
  
  private void setSpeed(double l, double r){
    motors[0].setVelocity(l);
    motors[1].setVelocity(r);
    motors[2].setVelocity(l);
    motors[3].setVelocity(r);
    
    System.out.println("Wheels are left and right");
  
  }
  
  private void getTurnAngle(){
    double angle = 0; //In degrees

    double[] compassValues = this.compass.getValues();
    System.out.println(compassValues);

    double heading = Math.PI + Math.atan2(compassValues[2], compassValues[0]);
    System.out.println(heading);
    
    
  }
  
  private void tryTimer(){
    this.timer = new Timer();
    timerTask = new TimerTask() {
      @Override
      public void run() {
        setSpeed(2,1);
      }
    };
    
    timer.schedule(timerTask, 1000);
    
  }
  
}
