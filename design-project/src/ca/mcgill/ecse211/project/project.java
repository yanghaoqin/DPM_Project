package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Display;

import ca.mcgill.ecse211.project.LightLocalizer;
import ca.mcgill.ecse211.project.Search;
import ca.mcgill.ecse211.project.USLocalizer;
import ca.mcgill.ecse211.project.CanCalibrator;
import ca.mcgill.ecse211.project.ColorDetection;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.remote.nxt.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * this is the main class from which the whole project is run.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 */

public class project {

  public static long time = 0;
  
  /**
   * Size of one tile in cm
   */
  public static final double TILE = 30.48;

  /**
   * A constant factor that can be applied to convert angle units in radians to degrees
   */
  public static final double TO_DEG = 180.0 / Math.PI;

  /**
   * A constant factor that can be applied to convert angular units in degrees to radians
   */
  public static final double TO_RAD = Math.PI / 180.0;

  /**
   * The radius (in cm) of the left/right wheels of the EV3 robot.
   */
  public static final double WHEEL_RAD = 2.15; //TODO: FIND WHEELRAD FOR NEW ROBOT

  /**
   * The width (in cm) of the robot measured from the center of the left wheel to the center of the
   * right wheel
   */
  public static final double TRACK = 15.4; //TODO: FIND TRACK FOR NEW ROBOT

  /**
   * A value for motor acceleration that prevents the wheels from slipping on the demo floor by
   * accelerating and decelerating slowly
   */
  public static final int SMOOTH_ACCELERATION = 500;

  /**
   * Specifies the speed of the left and right EV3 Large motors
   */
  public static final int SPEED = 100;

  /**
   * The heading/Theta value of the robot initially
   */
  public static final int INITIAL_ANGLE = 0;

  /**
   * A revolution of half of a circle in degrees
   */
  public static final int HALF_CIRCLE = 180;

  /**
   * A full revolution of a circle in degrees
   */
  public static final int FULL_CIRCLE = 360;

  // r, g, b in order
  public static final double[] BLUE_COLOR = {0.19, 0.40, 0.87}; // value of blue colour
  public static final double[] GREEN_COLOR = {0.35, 0.85, 0.39}; // value of green colour
  public static final double[] YELLOW_COLOR = {0.85, 0.52, 0.09}; // value of yellow colour
  public static final double[] RED_COLOR = {0.98, 0.24, 0.07}; // value of red colour

  /**
   * The instance of the left wheel large EV3 motor. The left motor is connected to port A on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The instance of the right wheel large EV3 motor. The right motor is connected to port D on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * Whether robot is currently in color detection
   */
  public static boolean isColorDetection; //TODO: NOT SURE IF NEEDED

  /**
   * The instance of the medium motor that turns the sensor. The motor is connected to port B on the
   * EV3 brick.
   */
  public static final NXTRegulatedMotor SENSOR_MOTOR =
      new NXTRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * Port for ultrasonic sensor.
   */
  private static final Port US_PORT = LocalEV3.get().getPort("S4");

  
  private static final Port RIGHT_PORT = LocalEV3.get().getPort("S2");  
  
  /**
   * Port for color sensor for localization.
   */
  private static final Port LEFT_PORT = LocalEV3.get().getPort("S1");

  /**
   * Port for color sensor for can classification.
   */
  private static final Port COLOR_PORT = LocalEV3.get().getPort("S3");

  /**
   * LCD display instance.
   */
  private static final TextLCD LCD = LocalEV3.get().getTextLCD();

  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  /**
   * The main method. This method is used to start threads and execute the main function of the
   * robot. Mainly localization and search.
   * 
   * @param args - arguments to pass in
   * @throws OdometerExceptions - multiple odometer instances
   */
  public static void main(String[] args) throws OdometerExceptions {

 // variable to record button clicked by user
    int buttonChoice;

    // set up odometer
    Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

    // set up display
    Display EV3Display = new Display(LCD);

    // US sensor initialization
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    // light sensor initialization, for can classification
    @SuppressWarnings("resource")
    SensorModes lightSensor = new EV3ColorSensor(COLOR_PORT);
    SampleProvider lightColor = ((EV3ColorSensor) lightSensor).getRGBMode();
    float[] lightData = new float[3];

    // color sensor for localization initialization
    @SuppressWarnings("resource")
    SensorModes Left_Sensor = new EV3ColorSensor(LEFT_PORT);
    SampleProvider left = Left_Sensor.getMode("Red");
    float[] leftcsData = new float[left.sampleSize()];

    // color sensor for localization initialization
    @SuppressWarnings("resource")
    SensorModes Right_Sensor = new EV3ColorSensor(RIGHT_PORT);
    SampleProvider right = Right_Sensor.getMode("Red");
    float[] rightcsData = new float[right.sampleSize()];

    // Option to choose light sensor test or start search routine
    do {
      LCD.clear();
      LCD.drawString("< Left  |  Right >", 0, 0);
      LCD.drawString("        |         ", 0, 1);
      LCD.drawString("   LS   |  Start  ", 0, 2);
      LCD.drawString("  TEST  |  Search ", 0, 3);
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ESCAPE);
    

    Display display = new Display(LCD);


      // exit system after esc pressed
      // clear display
      LCD.clear();  
      //at this point our robot will be on the closest gridline
      
      //TODO: LOCALIZATION
      (new Thread(odometer)).start();
      (new Thread(display)).start();
      Navigation navi = new Navigation(odometer);
      DoubleLightLocalization dll = new DoubleLightLocalization(odometer,left, right, leftcsData, rightcsData);
      dll.DoubleLocalizer();
      
      //TODO: MAKE IT GO THROUGH TUNNEL (NAVIGATION)
      
      //TODO: REACH SEARCH ZONE (NAVIGATION) AT LOWER LEFT CORNER
      Search search = new Search(odometer, usDistance, usData);
      search.run();
      
      
      //TODO: START SEARCH THREAD (INSIDE SEARCH, WE WILL START CAN ID AND WEIGHING AND HANDLING AND WHEN SEARCH TERMINATES WE GET BACK HERE)
      
      //TODO: GO BACK TO START (NAVIGATION)
      
      //TODO: DROP CAN (HANDLING)
      
      //TODO: RESTART (WHILE LOOP?)
      

     
      // exit when esc pressed
      while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      }
      System.exit(0); // exit program after esc pressed
    }
  }

