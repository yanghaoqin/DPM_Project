package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Lab5.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.BLUE_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.YELLOW_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.GREEN_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.RED_COLOR;

import java.util.Arrays;
import ca.mcgill.ecse211.odometer.*;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Navigator;
import ca.mcgill.ecse211.lab5.Navigation;
/*
import static ca.mcgill.ecse211.lab5.Lab5.SENSOR_MOTOR;
import ca.mcgill.ecse211.lab5.UltrasonicMotor;
*/
public class Search extends Thread{


  private final int SCAN_TIME = 100;  
  //{R, G, B} values based on in lab measurements and multiplied by 100
  private static final int LLx = 5; //lower left x coordinate of searching area, modify during demo
  private static final int LLy = 5; //lower left y coordinate of searching area, modify during demo
  private static final int URx = 7; //lower left x coordinate of searching area, modify during demo
  private static final int URy = 7; //lower left y coordinate of searching area, modify during demo

  private static final int RED_INDEX = 4;
  private static final int GREEN_INDEX = 2;
  private static final int BLUE_INDEX = 1;
  private static final int YELLOW_INDEX = 3;
  
  private static final int TR = 0; //colour of target can: must be changed during demo
  private static final int CAN_THERE = 50; //value of us sensor when there is a can in front of it TODO: tweak in lab
  private static final double WHEEL_RAD = 2.15;
  private static final int SPEED = 150;
  public static final double CAN_EXISTS = 50; //distance to show there is a can at that intersection, TODO: tweak in lab
  
  private static double[] threshold; //colour threshold to identify correct can
  private Odometer odometer; //odometer
  private float[] usData;
  private SampleProvider usDistance;
  private float[] lightData;
  private SampleProvider lightColor;
  private int filterSum;
  private int std;
  private boolean isCan;
  
  public Search(Odometer odometer, SampleProvider usDistance, float[] usData, SampleProvider lightColor, float[] lightData) {
    this.odometer = odometer;
    this.usData = usData;
    this.usDistance = usDistance;
    this.lightData = lightData;
    this.lightColor = lightColor;
    this.isCan = false;
    
    // determine which color from TR given
    switch(TR) {
      case 1: //TR = 1 --> blue can
        threshold = BLUE_COLOR;
        break;
      case 2: //TR = 2 --> green can
        threshold = GREEN_COLOR; 
        break;
      case 3: //TR = 3 --> yellow can
        threshold = YELLOW_COLOR; 
        break;
      case 4: //TR = 4 --> red can
        threshold = RED_COLOR; 
    }
  }
  
  public void run() {
    Navigation nav = new Navigation(odometer, usDistance, usData);
  
    // scan grid
    for (int x = LLx; x <= URx; x++) { //iterate over all x in search zone
      for (int y = LLy; y <= URy; y++) {
        isCan = nav.travelTo(x, y);
        if (isCan()) {
          canFound();
        }
      }      
    }
  }
  
  /**
   * 
   * @return
   */
  private boolean isCan() {
    double distance = medianFilter();
    if (distance < CAN_EXISTS) {
      return true;
    }
    return false;
  }

  private void canFound() {
    double[] red_array = new double[100]; //array for reds
    double[] green_array = new double[100]; //array for greens
    double[] blue_array = new double[100]; //array for blues
   
    CanCalibrator calibrator = new CanCalibrator(lightColor, lightData);
    double starting_angle = odometer.getXYT()[2];

     //TODO: Decide to use bang bang and do the wall follower here and stop every once in a while to take a reading

    //then we must calculate the means for r, g, and b, and then the euclidean distance
    //compare that distance with the mean of the colour we r looking for to determine if it is the right can or not  
    boolean color = calibrator.Calibrate(threshold);
    if(color) {
      //navigate to the end position
      
    }
    else {
      //navigate to the next position
    }
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