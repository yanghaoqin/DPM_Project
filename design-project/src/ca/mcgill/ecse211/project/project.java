package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.ColourID;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.LightLocalizer;
import ca.mcgill.ecse211.project.Search;
import ca.mcgill.ecse211.project.USLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/* this is the main class*/

public class project {
  
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
  public static final double TRACK = 13.3; //TODO: FIND TRACK FOR NEW ROBOT

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
  public static final EV3MediumRegulatedMotor SENSOR_MOTOR =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * Port for ultrasonic sensor.
   */
  private static final Port US_PORT = LocalEV3.get().getPort("S2");

  /**
   * Port for color sensor for localization.
   */
  private static final Port CS_PORT = LocalEV3.get().getPort("S4");

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

  
  }
}

