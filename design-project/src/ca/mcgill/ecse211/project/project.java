package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Display;

import ca.mcgill.ecse211.project.LightLocalizer;
import ca.mcgill.ecse211.project.Search;
import ca.mcgill.ecse211.project.USLocalizer;
import ca.mcgill.ecse211.project.CanCalibrator;
import ca.mcgill.ecse211.project.ColorDetection;
import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import static ca.mcgill.ecse211.project.project.SENSOR_MOTOR;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
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
  public static final double[] BLUE_COLOR = {0.19, 0.60, 0.65}; // value of blue colour
  public static final double[] GREEN_COLOR = {0.35, 0.85, 0.39}; // value of green colour
  public static final double[] YELLOW_COLOR = {0.85, 0.52, 0.09}; // value of yellow colour
  public static final double[] RED_COLOR = {0.98, 0.24, 0.07}; // value of red colour

  //params
  public static int corner;
  public static int team_LL_x;
  public static int team_LL_y;
  public static int team_UR_x;
  public static int team_UR_y;
  //island is the same fr both teams so no need for new var
  public static int tunnel_LL_x;
  public static int tunnel_LL_y;
  public static int tunnel_UR_x;
  public static int tunnel_UR_y;
  public static int zone_LL_x;
  public static int zone_LL_y;
  public static int zone_UR_x;
  public static int zone_UR_y;
  /**
   * The instance of the left wheel large EV3 motor. The left motor is connected to port A on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The instance of the right wheel large EV3 motor. The right motor is connected to port D on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * Whether robot is currently in color detection
   */
  public static boolean isColorDetection; 

  /**
   * The instance of the medium motor that turns the sensor. The motor is connected to port B on the
   * EV3 brick.
   */
  public static final EV3MediumRegulatedMotor SENSOR_MOTOR =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

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
    LCD.clear();  
    
    Display display = new Display(LCD);

      //TODO: WIFI ACQUISITION OF DATA
   Wifi.main(args);

      if (Wifi.RedTeam == 23) {
        corner = Wifi.RedCorner;
        team_LL_x = Wifi.Red_LL_x;
        team_LL_y = Wifi.Red_LL_y;
        team_UR_x = Wifi.Red_UR_x;
        team_UR_y = Wifi.Red_UR_y;
        tunnel_LL_x = Wifi.TNR_LL_x;
        tunnel_LL_y = Wifi.TNR_LL_y;
        tunnel_UR_x = Wifi.TNR_UR_x;
        tunnel_UR_y = Wifi.TNR_UR_y;
        zone_LL_x = Wifi.SZR_LL_x;
        zone_LL_y = Wifi.SZR_LL_y;
        zone_UR_x = Wifi.SZR_UR_x;
        zone_UR_y = Wifi.SZR_UR_y;
      }
      else if (Wifi.GreenTeam == 23) {
        corner = Wifi.GreenCorner;
        team_LL_x = Wifi.Green_LL_x;
        team_LL_y = Wifi.Green_LL_y;
        team_UR_x = Wifi.Green_UR_x;
        team_UR_y = Wifi.Green_UR_y;
        tunnel_LL_x = Wifi.TNG_LL_x;
        tunnel_LL_y = Wifi.TNG_LL_y;
        tunnel_UR_x = Wifi.TNG_UR_x;
        tunnel_UR_y = Wifi.TNG_UR_y;
        zone_LL_x = Wifi.SZG_LL_x;
        zone_LL_y = Wifi.SZG_LL_y;
        zone_UR_x = Wifi.SZG_UR_x;

        zone_UR_y = Wifi.SZG_UR_y;

      }
    
      //LOCALIZATION
      (new Thread(odometer)).start();
      (new Thread(display)).start();
      USLocalizer ul = new USLocalizer(odometer, LEFT_MOTOR, RIGHT_MOTOR, buttonChoice, usDistance);

      ul.localize();
      DoubleLightLocalization dll = new DoubleLightLocalization(odometer,left, right, leftcsData, rightcsData);
      dll.DoubleLocalizer();

      Sound.beep(); //to signal robot in place (beta demo requirement)
      
      //now that the robot is at the gridline, update odometer based on corner number
      switch (corner) {
        case 0: 
          odometer.setXYT(1 * TILE, 1 * TILE, 0);        
          break;
        case 1:
          odometer.setXYT(14 * TILE, 1 * TILE, 270);
          break;
        case 2:
          odometer.setXYT(14 * TILE, 8 * TILE, 180);
          break;
        case 3:
          odometer.setXYT(1 * TILE, 8 * TILE, 90);        
      } //theta might have to be changed if not the same reference system as in lab 5
      
      Navigation nav = new Navigation(odometer);

      //TODO: READCH LOWER LEFT OF TUNNEL (NAVIGATION)
      int tunnelLength = 0;
      
      if ((tunnel_UR_x - tunnel_LL_x) < (tunnel_UR_y - tunnel_LL_y)) { //tunnel is vertical        
        tunnelLength = tunnel_UR_y - tunnel_LL_y + 1; //+1 to take into account half a tile before and after the tunnel
        if ((corner == 0) || (corner == 1)) {
          //we enter the tunnel by LL corner side
          nav.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.5);
          nav.turnTo(0);
        }
        else { //corner is 2 or 3
          //we enter the tunnel by UR corner side
          nav.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y + 0.5);
          nav.turnTo(180);
        }
      }
      else { //tunnel is horizontal
        tunnelLength = tunnel_UR_x - tunnel_LL_x + 1;
        if ((corner == 0) || (corner == 3)) {
          //we enter tunnel by LL corner side
          nav.travelTo(tunnel_LL_x - 0.5, tunnel_LL_y + 0.5);
          nav.turnTo(90);
        }
        else { //corner is 1 or 2
          //we enter tunnel by UR corner side
          nav.travelTo(tunnel_UR_x + 0.5, tunnel_UR_y - 0.5);
          nav.turnTo(270);
        }
      }      
      
      //MAKE IT GO THROUGH TUNNEL (NAVIGATION)
      RIGHT_MOTOR.rotate(Navigation.convertDistance(WHEEL_RAD, tunnelLength*TILE), true);
      LEFT_MOTOR.rotate(Navigation.convertDistance(WHEEL_RAD, tunnelLength*TILE), false);
      
      //REACH SEARCH ZONE (NAVIGATION) AT LOWER LEFT CORNER
      nav.travelTo(zone_LL_x, zone_LL_y);
      Sound.beep();
      Sound.beep();
      Sound.beep();
      Sound.beep();
      Sound.beep(); //beeps 5 times (beta demo requirement)
      
      //START SEARCH THREAD (INSIDE SEARCH, WE WILL START CAN ID AND WEIGHING AND WHEN SEARCH TERMINATES WE GET BACK HERE)
      Search search = new Search(odometer, usDistance, usData, lightColor, lightData, LCD);
      search.run();
      
      //BETA DEMO ONLY --> GO TO UPPER RIGHT CORNER
      nav.travelTo(zone_UR_x, zone_UR_y); //goes to upper right corner
      Sound.beep();
      Sound.beep();
      Sound.beep();
      Sound.beep();
      Sound.beep(); //beep 5 times (beta demo requirement)
      //end of beta demo
     
    
      WeightID weight = new WeightID(left, leftcsData); //TODO: THIS WILL BE PLACED IN SEARCH ALGORITHM AFTERWARDS maybe?
      weight.weight(); //TODO: COMMENT OUT FOR BETA DEMO
      
      //RETRIEVE CAN
      Handling handling = new Handling(odometer);
      Thread handlingThread = new Thread(handling);   
      handlingThread.start();
      
      //GO BACK TO START (NAVIGATION)
      switch (corner) { //navigates to the middle of the starting corner tile
        case 0: 
          nav.travelTo(0.5, 0.5);   
          break;
        case 1:
          nav.travelTo(14.5, 0.5);
          break;
        case 2:
          nav.travelTo(14.5, 8.5);
          break;
        case 3:
          nav.travelTo(0.5, 8.5);
      } //now robot is back at its starting corner
      
      //TODO: DROP CAN (HANDLING)
      handlingThread.stop();
      handling.dispose();
      
      //TODO: RESTART (WHILE LOOP?)

     
      // exit when esc pressed
      while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      }
      System.exit(0); // exit program after esc pressed
    }  
 
  }

