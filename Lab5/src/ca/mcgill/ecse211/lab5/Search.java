package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Lab5.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.RIGHT_MOTOR;
import java.util.Arrays;
import ca.mcgill.ecse211.odometer.*;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.lab5.Navigation;

public class Search extends Thread{

  //{R, G, B} values based on in lab measurements and multiplied by 100
  private static final double[] blue = {0.39, 1.38, 0.79} ; //value of blue colour
  private static final double[] green = {1.0, 1.5, 0.5}; //value of green colour
  private static final double[] yellow = {2.25, 1.66, 0.29}; //value of yellow colour
  private static final double[] red = {2.84, 1.17, 0.58}; //value of red colour
  private static final int TR = 0; //colour of target can: must be changed during demo
  private static double[] threshold; //colour threshold to identify correct can
  private static final int CAN_THERE = 50; //value of us sensor when there is a can in front of it TODO: tweak in lab
  private static final double WHEEL_RAD = 2.15;;
  Odometer odometer; //odometer
  private float[] usData;
  private SampleProvider usDistance;
  private float[] lightData;
  private SampleProvider lightColor;
  private int filterSum;
  
  public Search(Odometer odometer, SampleProvider usDistance, float[] usData, SampleProvider lightColor, float[] lightData) {
    this.odometer = odometer;
    this.usData = usData;
    this.usDistance = usDistance;
    this.lightData = lightData;
    this.lightColor = lightColor;
    switch(TR) {
      case 1: threshold = blue; //TR = 1 --> we are looking for a blue can
        break;
      case 2: threshold = green; //TR = 2 --> we are looking for a green can
        break;
      case 3: threshold = yellow; //TR = 3 --> we are looking for a yellow can
        break;
      case 4: threshold = red; //TR = 4 --> we are looking for a red can
    }
  }
  
  public void run() {
    double distance;
    double angle;
    while(true) {
      distance = medianFilter();
      if (distance <= CAN_THERE) { 
        angle = sensorMotor.getAngle; //TODO: add sensor motor and find the angle at which it is
        break;
      }
    }
    canFound(angle, distance);
  }
  
  private void canFound(double angle, double distance) {
    Navigation nav = new Navigation(odometer);
    double[] r1, r2, r3, r4; //4 RGB readings
    nav.turnTo(angle);
 // rotates both motors for a fixed number of degrees equivalent to ds, the
    // distance from the robot's current location to the next destination point,
    // equivalent as travelling straight. The boolean flag parameter indicates
    // whether method returns immediately, to allow simultaneous execution of both
    // rotate() methods. The method waits for the right motor to complete.
    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, distance - 10), true); //TODO: tweak -10 in lab --> we dont want the robot to crash into the can
    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, distance - 10), false);
    //TODO: MAKE THE ROBOT TURN AROUND THE CAN AND TAKE 4 READINGS
    //then we must calculate the means for r, g, and b, and then the euclidean distance
    //compare that distance with the mean of the colour we r looking for to determine if it is the right can or not
    
    
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
}