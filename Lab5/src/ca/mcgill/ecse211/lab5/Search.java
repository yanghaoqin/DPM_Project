package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Lab5.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.BLUE_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.YELLOW_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.GREEN_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.RED_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.SENSOR_MOTOR;
import java.util.Arrays;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Navigator;
import ca.mcgill.ecse211.lab5.Navigation;

/*
 * import static ca.mcgill.ecse211.lab5.Lab5.SENSOR_MOTOR; import
 * ca.mcgill.ecse211.lab5.UltrasonicMotor;
 */
public class Search extends Thread {


  private final int SCAN_TIME = 100;
  // {R, G, B} values based on in lab measurements and multiplied by 100
  public static final int LLx = 1; // lower left x coordinate of searching area, modify during demo
  public static final int LLy = 1; // lower left y coordinate of searching area, modify during demo
  public static final int URx = 4; // lower left x coordinate of searching area, modify during demo
  public static final int URy = 4; // lower left y coordinate of searching area, modify during demo

  private static final int RED_INDEX = 4;
  private static final int GREEN_INDEX = 2;
  private static final int BLUE_INDEX = 1;
  private static final int YELLOW_INDEX = 3;

  private static final int TR = 1; // colour of target can: must be changed during demo

  private static final double WHEEL_RAD = 2.15;
  private static final int SPEED = 100;
  public static final double CAN_EXISTS = 15; // distance to show there is a can at that
                                              // intersection, TODO: tweak in lab

  private static double[] threshold; // colour threshold to identify correct can
  private Odometer odometer; // odometer
  private float[] usData;
  private SampleProvider usDistance;
  private float[] lightData;
  private SampleProvider lightColor;
  private int filterSum;
  private int std;
  private boolean isCan;
  private Navigation nav;
  private double[] colorData;
  private CanCalibrator calibrator;

  public static double distDisplay;

  public Search(Odometer odometer, SampleProvider usDistance, float[] usData,
      SampleProvider lightColor, float[] lightData) {
    this.odometer = odometer;
    this.usData = usData;
    this.usDistance = usDistance;
    this.lightData = lightData;
    this.lightColor = lightColor;
    this.isCan = false;
    this.colorData = new double[4];
    nav = new Navigation(odometer, usDistance, usData);

    // determine which color from TR given
    switch (TR) {
      case 1: // TR = 1 --> blue can
        threshold = BLUE_COLOR;
        break;
      case 2: // TR = 2 --> green can
        threshold = GREEN_COLOR;
        break;
      case 3: // TR = 3 --> yellow can
        threshold = YELLOW_COLOR;
        break;
      case 4: // TR = 4 --> red can
        threshold = RED_COLOR;
    }
  }

  public void run() {
    // scan grid
    boolean canFound = false;
    
    for (int y = LLy; y <= URy; y++) {
      
      canFound = false;
      
      if ((y - LLy) % 2 == 0) {
        for (int x = LLx; x <= URx; x++) {
          canFound = false;
          while (canFound == false) {
            isCan = nav.travelTo(x, y);
            if (isCan) {
              canFound = canFound();
            } else {
              canFound = true;
            }
          }
        }
      } else {
        if ((y - LLy) % 2 == 1) {
          for (int x = URx; x >= LLx; x--) {
            canFound = false;
            while (canFound == false) {
              isCan = nav.travelTo(x, y);
              if (isCan) {
                canFound = canFound();
              } else {
                canFound = true;
              }
            }
          }
        }
      }
    }
  }

  // /**
  // *
  // * @return
  // */
  // private boolean isCan() {
  // double distance = medianFilter();
  // if (distance < CAN_EXISTS) {
  // Sound.beepSequenceUp();
  //
  // LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, 12), true);
  // RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, 12), false);
  // return true;
  // }
  // return false;
  // }

  private boolean canFound() {
    // double[] red_array = new double[100]; // array for reds
    // double[] green_array = new double[100]; // array for greens
    // double[] blue_array = new double[100]; // array for blues

    calibrator = new CanCalibrator(lightColor, lightData);
    // double starting_angle = odometer.getXYT()[2];

    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, 10), true);
    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, 10), false);

    // check if color reading is correct
    int color = rotateSensorDetect();

    LEFT_MOTOR.rotate(-convertDistance(WHEEL_RAD, 9), true);
    RIGHT_MOTOR.rotate(-convertDistance(WHEEL_RAD, 9), false);

    if (color == colorconvert(TR)) {
      // navigate to the end position
      Sound.beep();
      hitIt(true);
      nav.travelTo(URx, URy);
      System.exit(0);
      return true;
    } else {
      // navigate to the next position
      Sound.twoBeeps();
      hitIt();
      return false;
    }
  }

  private int colorconvert(int tr) {
    switch (tr) {
      case 1:
        return 2;
      case 2:
        return 1;
      case 3:
        return 3;
      case 4:
        return 0;
    }
    return -1;
  }

  private int rotateSensorDetect() {
    int[] colorResult = new int[7];
    for (int i = 0; i < 7; i++) {
      colorResult[i] = calibrator.Calibrate();
      Lab5.SENSOR_MOTOR.setAcceleration(300);
      Lab5.SENSOR_MOTOR.setSpeed(300);
      Lab5.SENSOR_MOTOR.rotate(30, false);
    }
    SENSOR_MOTOR.rotateTo(0, false);
    Arrays.sort(colorResult);
    int prev = colorResult[0];
    int count = 1;
    for (int i = 1; i < colorResult.length; i++) {
      if (colorResult[i] == prev && colorResult[i] != -1) {
        count++;
        if (count > colorResult.length / 2 - 1) {
          return colorResult[i];
        }
      } else {
        count = 1;
        prev = colorResult[i];
      }
    }
    return -1;
  }

  /**
   * oh yeah
   */
  private void hitIt() {
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);
    
//    SENSOR_MOTOR.setSpeed(SENSOR_MOTOR.getMaxSpeed());
//    SENSOR_MOTOR.setAcceleration(5000);
    
    SENSOR_MOTOR.rotate(220, false);
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);
    SENSOR_MOTOR.rotateTo(0, false);
  }
  
  private void hitIt(boolean flag) {
    SENSOR_MOTOR.setSpeed(SENSOR_MOTOR.getMaxSpeed());
    SENSOR_MOTOR.setAcceleration(5000);
    
    SENSOR_MOTOR.rotate(220, false);
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);
    SENSOR_MOTOR.rotateTo(0, false);
  }

  /**
   * initial readings taken (100 times) and the average is used to distinguish between the wooden
   * board and the black line. Each reading is amplified to enhance the sensitivity of the sensor
   * 
   * @return
   */
  private double initialReading(int color_id) {
    for (int i = 0; i < 100; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, color_id);
      // amplifies and sums the sample data
      std += lightData[color_id] * 100.0;
    }
    // take average of the standard
    return std /= 100.0;
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

  private double meanFilter() {
    filterSum = 0;
    // take 5 readings
    for (int i = 0; i < 5; i++) {
      // acquire sample data and read into array with no offset
      lightColor.fetchSample(lightData, 0);
      // amplify signal for increased sensitivity
      filterSum += lightData[0] * 100;
    }
    // return an amplified average
    return filterSum / 5.0;
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
