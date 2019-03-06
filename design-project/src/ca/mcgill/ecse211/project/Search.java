package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.MotorSweep;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import static ca.mcgill.ecse211.project.project.SENSOR_MOTOR;
import java.util.Arrays;


/*this class will be responsible for searching the zone for cans*/
public class Search extends Thread{

    private static final int URx = 10; //upper right corner of search zone x coord (modify if needed)
    private static final int URy = 10; //upper right corner of search zone y coord (modify if needed)
    private static final int LLx = 3; //lower left corner x coord (modify if needed)
    private static final int LLy = 3; //lower left corner y coord (modify if needed)
    private Odometer odo;
    private SampleProvider usDistance;
    private float[] usData;
    private static final float SPEED = 100; //TODO: TWEAK
    private static final float CAN = 30; //distance recorded by the us sensor at which there must be a can, TODO: TWEAK
    private static final double WHEEL_RAD = 0; //TODO: TWEAK
    private static final double TRACK = 0; //TODO: TWEAK
    private static final double CAN_CLOSE = 0; //closer distance at which there must be a can. robot must stop as to not hit can. TODO: TWEAK
    private float angleTacho; //tacho count for angle at which can detected
    
    public Search(Odometer odo, SampleProvider usDistance, float[] usData) {
      this.odo = odo;
      this.usDistance = usDistance;
      this.usData = usData;
    }
    
    //the search algorithm works as follows: the robot advances in a straight line til the end of search zone
    //while sweeping with us sensor. if can found, robot goes towards tht can, else robot turns 90 degrees right and then backtracks 
    //to the beginning of search zone. repeat as needed.
    public void run() {
      MotorSweep motorSweep = new MotorSweep(SENSOR_MOTOR);
      motorSweep.start();
      motorSweep.startSensor(); //NOT SURE THIS IS HOW TO START A THREAD
      
      LEFT_MOTOR.setSpeed(SPEED);
      RIGHT_MOTOR.setSpeed(SPEED);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
      
      while (true) { //TODO: CHANGE THIS TO LIMIT THE ROBOT TO STAY IN SEARCH ZONE
        double distance = medianFilter();
        if (distance < CAN) { //can is detected
          motorSweep.stopSensor(); //motor stops sweeping
          angleTacho = SENSOR_MOTOR.getTachoCount();
          float angle = angleTacho - motorSweep.straightTacho; //find angle at which robot must rotate to keep going straight to find can
          
          LEFT_MOTOR.stop();
          RIGHT_MOTOR.stop();
          
          RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), true);
          LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, angle), false);
          //turns to that angle
          
         break;    
          
        }
      }
     
      LEFT_MOTOR.setSpeed(SPEED);
      RIGHT_MOTOR.setSpeed(SPEED);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
      
      while (true) {
        double distance = medianFilter();
        if (distance < CAN_CLOSE) { //can is detected
          LEFT_MOTOR.stop();
          RIGHT_MOTOR.stop();
          break;
        }     
      }
    }
    /**
     * This is a median filter. The filter takes 5 consecutive readings from the ultrasonic sensor,
     * amplifies them to increase sensor sensitivity, sorts them, and picks the median to minimize the
     * influence of false negatives and false positives in sensor readings, if any. The sensor is very
     * likely to report false negatives.
     * 
     * @return the median of the five readings, sorted from small to large
     */
    private double medianFilter() {
      double[] arr = new double[5]; // store readings
      for (int i = 0; i < 5; i++) { // take 5 readingss
        usDistance.fetchSample(usData, 0); // store reading in buffer
        arr[i] = usData[0] * 100.0; // signal amplification
      }
      Arrays.sort(arr); // sort readingss
      return arr[2]; // take median value
    }
    
    /**
     * This is a static method allows the conversion of a distance to the total rotation of each wheel
     * need to cover that distance.
     * 
     * (Distance / Wheel Circumference) = Number of wheel rotations. Number of rotations * 360.0
     * degrees = Total number of degrees needed to turn.
     * 
     * @param radius - Radius of the wheel
     * @param distance - Distance of path
     * @return an integer indicating the total rotation angle for wheel to cover the distance
     */
    private static int convertDistance(double radius, double distance) {
      return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
     * This is a static method that converts the angle needed to turn at a corner to the equivalent
     * total rotation. This method first converts the degrees of rotation, radius of wheels, and width
     * of robot to distance needed to cover by the wheel, then the method calls another static method
     * in process to convert distance to the number of degrees of rotation.
     * 
     * @param radius - the radius of the wheels
     * @param width - the track of the robot
     * @param angle - the angle for the turn
     * @return an int indicating the total rotation sufficient for wheel to cover turn angle
     */
    private static int convertAngle(double radius, double width, double angle) {
      return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
  }


    

