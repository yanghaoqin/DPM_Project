package ca.mcgill.ecse211.project;

/* this class will be responsible for navigating to a point and turning to minimal angles*/

// non-static imports
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

// static imports from Lab3 class
import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import static ca.mcgill.ecse211.project.project.TILE;
import static ca.mcgill.ecse211.project.project.TRACK;
import static ca.mcgill.ecse211.project.project.WHEEL_RAD;

/**
 * <p>
 * This class implements the simple navigator. Some of the existing motor instances and a few
 * constants not dependent on navigation. This class extends the Thread class to allow simultaneous
 * execution, so that other classes can work alongside this class. Helper methods are added at the
 * end to make conversions easier.
 * 
 * <p>
 * The robot completes a path with a total of 5 points, specified using a coordinate system relative
 * to the start of the robot. The robot is (0,0), with the front set as 0 degrees Theta, the right
 * as the +x direction, and the front as the +y direction. Theta increases when turning in the
 * clockwise direction. The robot first turns to face the point it is travelling to, and then
 * travels to that point.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 */
public class NavigationWithCorr extends Thread {

  // -----------------------------------------------------------------------------
  // Constants
  // -----------------------------------------------------------------------------

  /**
   * The width (in cm) of the robot measured from the center of the left wheel to the center of the
   * right wheel. Not imported from Lab3 because track needs to be tuned specifically for simple
   * navigation
   */
 // private static final double TRACK = 15.4;

  /**
   * The radius (in cm) of the left/right wheels of the EV3 robot. Wheel rad not imported from Lab3
   * class specifically for simple navigation adjustments. TWEAKING THIS VALUE SIGNIFICANTLY CHANGES
   * HOW MUCH OF AN ANGLE THE ROBOT TURNS.
   */
 // private static final double WHEEL_RAD = 2.15;

  /**
   * A constant factor that can be applied to convert angular units in degrees to radians
   */
  private static final double TO_RAD = Math.PI / 180.0;

  /**
   * A constant factor that can be applied to convert angular units in radians to degrees
   */
  private static final double TO_DEG = 180.0 / Math.PI;

  /**
   * The speed at which the robot moves straight (in deg/sec)
   */
  private static final int FWDSPEED = 200;

  /**
   * The speed at which the robot turns in a stationary fashion (in deg/sec)
   */
  private static final int TURNSPEED = 150;

  /**
   * Angle correction for Quadrant 1 and 4. Arctan returns the correct angle and the only adjustment
   * needed is to turn it into an angle that starts from Theta = 0 and increases clockwise
   */
  private static final int Q1Q4COR = 90;

  /**
   * Angle correction for Quadrant 2 and 3. Arctan returns the incorrect angle and the adjustment
   * needed is to add pi to the angle and turn it into an angle that starts from Theta = 0 and
   * increases clockwise
   */
  private static final int Q2Q3COR = 270;

  /**
   * The center of the board platform that the EV3 runs on. This is used in determining which
   * quadrant the destination coordinate is in relative to the robot's current location and whether
   * arctan needs correction
   */
  private static final int CENTER = 0;

  /**
   * A value for motor acceleration that prevents the wheels from slipping on the demo floor by
   * accelerating and decelerating slowly
   */
  private static final int SMOOTH_ACCELERATION = 5000;

  /**
   * A value for motor acceleration that prevents the wheels from slipping on the demo floor by
   * accelerating and decelerating slowly
   */
  private static final int TURN_ACCELERATION = 250;

  /**
   * A revolution of half of a circle in degrees
   */
  private static final int HALF_CIRCLE = 180;

  /**
   * A full revolution of a circle in degrees
   */
  private static final int FULL_CIRCLE = 360;

  /**
   * The heading/Theta value of the robot initially
   */
  private static final int INITIAL_ANGLE = 0;

  /**
   * threashold to trigger odometer correction
   */
  private static final int THREASHOLD = 2;
  
  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------

  /**
   * The odometer instance
   */
  private Odometer odo;

  /**
   * Records odometer data returned, in a double precision array, specifying X, Y, and Theta values
   */
  private double position[];

  /**
   * A volatile boolean variable to indicate whether robot is currently travelling along a path,
   * from one way point to the next. The volatile keyword tells the thread to check the current
   * value of this variable in the main memory first
   */
  private volatile boolean isNavigating;
  
  private SampleProvider LeftidColour;
  private SampleProvider RightidColour;
  private float[] LeftcolorValue;
  private float[] RightcolorValue;
  private RegulatedMotor[] motorArrR= { RIGHT_MOTOR};
  private RegulatedMotor[] motorArrL= { LEFT_MOTOR};
  private int correctionCounter = 0;


  // -----------------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------------

  /**
   * Constructor for this class, sets up the odometer instance to allow access to position data and
   * initializes the isNavigating flag to false
   * 
   * @param odometer - the odometer instance passed from Lab3, gives access to retrieve position
   *        data
   */
  public NavigationWithCorr(Odometer odometer, SampleProvider left, SampleProvider right, float[] leftdata, float[] rightdata) {
	  
    this.odo = odometer;
    this.isNavigating = false;
    this.LeftidColour = left;
    this.RightidColour = right;
    this.LeftcolorValue = leftdata;
    this.RightcolorValue = rightdata;
  }

  // -----------------------------------------------------------------------------
  // Public Methods
  // -----------------------------------------------------------------------------

  /**
   * Controls the robot to travel to the coordinate (x,y) with the robot's initial starting location
   * as the origin. This is done by retrieving current position data of the robot and calculating
   * the new heading the robot needs to have, as well as the distance the robot needs to travel to
   * reach its next destination. A minimum angle approach is taken, meaning that the robot will turn
   * the smallest angle possible to adjust its heading.
   * 
   * <p>
   * There is a logic implemented in this method to determine the angle the robot needs to turn,
   * clockwise, to reach its new heading. This logic is necessary largely due to the values returned
   * by arctan function. The arctan function only returns values ranging from -pi/2 to pi/2, and
   * real values can be + or - pi from the returned value.
   * 
   * @param x - the x coordinate with the robot as the origin (0,0)
   * @param y - the y coordinate with the robot as the origin (0,0)
   */
  public void travelTo(double x, double y) {
	  
	  
	  	double xCoor = x;
	  	double yCoor = y;
	  	

	    // convert input coordinates x and y into distances in cm
	    x = x * TILE;
	    y = y * TILE;

	    isNavigating = true; // update navigating status
	    position = odo.getXYT(); // get current position data from odometer

	    // position[0] = x, position[1] = y, position[2] = theta
	    double dx = x - position[0]; // displacement in x
	    double dy = y - position[1]; // displacment in y
	    double ds = Math.hypot(dx, dy); // calculates the hypotenuse of dx and dy --> gives the
	                                    // displacement robot will
	                                    // need to travel to get to destination
	    double dTheta = Math.atan(dy / dx) * TO_DEG;// calculates the angle the new displacement will
	                                                // be. Value in the
	                                                // range of [-90,90] degrees

	    // Convention: north = 0 degrees; Theta increases clockwise
	    // The following logic is used to determine the angle the robot needs to turn
	    if (dTheta >= CENTER && dx >= CENTER) {
	      // 1st quadrant
	      dTheta = Q1Q4COR - dTheta; // clockwise angle robot needs to turn
	    } else if (dTheta >= CENTER && dx < CENTER) {
	      // 3rd quadrant, need to correct arctan value
	      dTheta = Q2Q3COR - dTheta; // clockwise angle robot needs to turn
	    } else if (dTheta < CENTER && dx >= CENTER) {
	      // 4th quadrant
	      dTheta = Q1Q4COR - dTheta; // clockwise angle robot needs to turn
	    } else if (dTheta < CENTER && dx < CENTER) {
	      // 2nd quadrant, need to correct arctan value
	      dTheta = Q2Q3COR - dTheta; // absolute angle
	    }

	    turnTo(dTheta); // robot turns at minimum angle
	    try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    project.LEFT_MOTOR.setAcceleration(6000);
	    project.RIGHT_MOTOR.setAcceleration(6000);
	    
	    project.LEFT_MOTOR.setSpeed(200);
	    project.RIGHT_MOTOR.setSpeed(200);

	    if (correctionCounter %3  == 0){
	    	LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, 7), true);
		    project.LEFT_MOTOR.synchronizeWith(motorArrR);
		    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, 7), false);
		    
		    project.LEFT_MOTOR.stop(true);
	        project.RIGHT_MOTOR.stop(false);

	        try {
				Thread.sleep(250);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
	    boolean counter_left = true;
	    boolean counter_right = true;
	    while (counter_left || counter_right) {
	    	
	        
	      if(counter_left) {
	        project.LEFT_MOTOR.forward();
	        
	      }
	      
	      if(counter_right) {
	        project.RIGHT_MOTOR.forward();
	      
	      }
	      
	       
	      if(fetchSampleLeft() < 0.38 && counter_left) {
	        project.LEFT_MOTOR.stop(true);
	        project.RIGHT_MOTOR.stop(false);
	        Sound.beep();
	        counter_left = false;
	        try {
	          Thread.sleep(200);
	        }
	        catch(Exception e) {
	          
	        }
	        //project.RIGHT_MOTOR.rotate(-90);
	        
	        project.RIGHT_MOTOR.forward();
	        if(fetchSampleLeft() < 0.38 && !counter_right) {
	          project.LEFT_MOTOR.stop(true);
	          project.RIGHT_MOTOR.stop(false);
	          break;      
	        }
	      }
	       
	      if(fetchSampleRight() < 0.38 && counter_right) {
	        project.RIGHT_MOTOR.stop(true);
	        project.LEFT_MOTOR.stop(false);
	        Sound.buzz();
	        counter_right = false;
	        try {
	          Thread.sleep(200);
	        }
	        catch(Exception e) {
	          
	        }
 
	        project.LEFT_MOTOR.forward();
	        if(fetchSampleRight() < 0.38 && !counter_left) {
	          project.LEFT_MOTOR.stop(true);
	          project.RIGHT_MOTOR.stop(false);
	          break;
	        }
	      }
	    }
	      

	    caliOdoTheta();
	    project.LEFT_MOTOR.stop(true);
        project.RIGHT_MOTOR.stop(false);
        project.LEFT_MOTOR.synchronizeWith(motorArrR);
       
        try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    caliOdoCoor(xCoor, yCoor);
	    position = odo.getXYT(); 
	    dx = x - position[0]; // displacement in x
	    dy = y - position[1]; // displacment in y
	    ds = Math.hypot(dx, dy); // calculates the hypotenuse of dx and dy --> gives the
	                                    // displacement robot will
	                                    // need to travel to get to destination
	   
	    try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    }

	    // smooth acceleration so that wheels do not slip
	    RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
	    LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

	    // sets both motors to forward speed
	    RIGHT_MOTOR.setSpeed(FWDSPEED);
	    LEFT_MOTOR.setSpeed(FWDSPEED);
    
	    try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    // rotates both motors for a fixed number of degrees equivalent to ds, the
	    // distance from the robot's current location to the next destination point,
	    // equivalent as travelling straight. The boolean flag parameter indicates
	    // whether method returns immediately, to allow simultaneous execution of both
	    // rotate() methods. The method waits for the right motor to complete.
	    
	    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), true);
	    project.LEFT_MOTOR.synchronizeWith(motorArrR);
	    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), false);
	    try {
	 			Thread.sleep(50);
	 		} catch (InterruptedException e) {
	 			// TODO Auto-generated catch block
	 			e.printStackTrace();
	 		}

	    correctionCounter++;
	    isNavigating = false; // update navigation status
	  }
  
  public void navigateTo(double cur_x, double cur_y, double final_x, double final_y){
	  if(cur_x > final_x){
		  while(cur_x > final_x){
			  try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			  travelTo(cur_x -1, cur_y);
				  cur_x--;
		  
		  }
	  }else if(cur_x < final_x ){
		  while(cur_x < final_x){
			  try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			  travelTo(cur_x + 1, cur_y);
				  cur_x++;
		 
		  }
	  }else{	//equal
	  }
	  
	  cur_x = final_x;
	  
	  if(cur_y > final_y){
		  while(cur_y > final_y){
			  try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			  travelTo(cur_x, cur_y - 1);
				  cur_y--;
		  
		  }
	  }else if(cur_y < final_y ){
		  while(cur_y < final_y){
			  try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			 travelTo(cur_x, cur_y + 1);
				  cur_y++;
		  
		  }
	  }else{	//equal
	  }
  }

  /**
   * Turns the robot at a fixed position to face the next destination, changes the robot heading.
   * The minimum angle is calculated by determining whether the clockwise angle Theta is greater
   * than half of a full circles
   * 
   * @param Theta - the clockwise angle to turn from Theta = 0
   */
  public void turnTo(double Theta) {

    isNavigating = true; // update navigating status

    // smoother turn acceleration to avoid wheels slipping
    LEFT_MOTOR.setAcceleration(TURN_ACCELERATION);
    RIGHT_MOTOR.setAcceleration(TURN_ACCELERATION);
    
    //position[2] = odo.getXYT()[2]; //get current theta value

    // ensure angle is positive and within 360
    double minTheta = ((Theta - odo.getXYT()[2]) + FULL_CIRCLE) % FULL_CIRCLE;

    if (minTheta > INITIAL_ANGLE && minTheta <= HALF_CIRCLE) {
      // angle is already minimum angle, robot should turn clockwise
      RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), true);
      LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), false);
    } else if (minTheta > HALF_CIRCLE && minTheta < FULL_CIRCLE) {
      // angle is not minimum angle, robot should turn counter-clockwise to the
      // complementary angle of a full circle 360 degrees
      minTheta = FULL_CIRCLE - minTheta;
      RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
      LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false);
    }

    isNavigating = false; // update navigation status
  }

  /**
   * Wrapper method to determine whether robot is currently navigating by checking class variable
   * {@code isNavigating}
   * 
   * @return true if another thread has called travelTo() or turnTo() and the method has yet to
   *         return, false otherwise
   */
  public boolean isNavigating() {
    return isNavigating;
  }

  // -----------------------------------------------------------------------------
  // Private Methods
  // -----------------------------------------------------------------------------

  private void caliOdoTheta(){
	  if (odo.getXYT()[2] > 80 && odo.getXYT()[2] < 100){
		  odo.setTheta(90);
	  }else if (odo.getXYT()[2] > 170 && odo.getXYT()[2] < 190){
		  odo.setTheta(180);
	  }else if (odo.getXYT()[2] > 260 && odo.getXYT()[2] < 280){
		  odo.setTheta(270);
	  }else if (odo.getXYT()[2] > 350 || odo.getXYT()[2] < 10){
		  odo.setTheta(0);

	  }
	  try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
  }
  private void caliOdoCoor(double x, double y){
	  double theta = odo.getXYT()[2] ;
	  if (theta > 80 && theta < 100){
		  odo.setX((x-1)* TILE + DoubleLightLocalization.SENSOR_TOWHEEL);
		  odo.setY(y* TILE);  
	  }else if (theta > 170 && theta < 190){
		  odo.setX(x * TILE);
		  odo.setY((y+1) * TILE - DoubleLightLocalization.SENSOR_TOWHEEL);  
	  }else if (theta > 260 && theta < 280){
		  odo.setX((x+1) * TILE - DoubleLightLocalization.SENSOR_TOWHEEL);
		  odo.setY(y * TILE); 
	  }else if (theta > 350 || theta < 10){
		  odo.setX(x * TILE);
		  odo.setY((y-1) * TILE + DoubleLightLocalization.SENSOR_TOWHEEL); 
	  }
	  try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	 
  }
  
  public void locaAtTunnel(double x, double y){
	  
	  
	  travelToLine();
	  LEFT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), true);
	  project.LEFT_MOTOR.synchronizeWith(motorArrR);
	  RIGHT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), false);
	  
	  DoubleLightLocalization.reorientRobot(Math.PI/2);
	  
	  travelToLine();
	  LEFT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), true);
	  project.LEFT_MOTOR.synchronizeWith(motorArrR);
	  RIGHT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), false);
	  
	  DoubleLightLocalization.reorientRobot(-Math.PI/2);
	  odo.setXYT(x * project.TILE, y * project.TILE, 0);
	  
	  
  }
  /**
   * Travel to the front until the 2 light sensors poll a black line
   */
  private void travelToLine() {


    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, 10), true);
    project.LEFT_MOTOR.synchronizeWith(motorArrR);
    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, 10), false);
    
    project.LEFT_MOTOR.stop(true);
    project.RIGHT_MOTOR.stop(false);
    project.LEFT_MOTOR.setAcceleration(6000);
    project.RIGHT_MOTOR.setAcceleration(6000);
    
    project.LEFT_MOTOR.setSpeed(200);
    project.RIGHT_MOTOR.setSpeed(200);


    try {
		Thread.sleep(250);
	} catch (InterruptedException e1) {
		// TODO Auto-generated catch block
		e1.printStackTrace();
	}
    
    boolean counter_left = true;
    boolean counter_right = true;
    while (counter_left || counter_right) {
      if(counter_left) {
        project.LEFT_MOTOR.forward();
        
      }
      
      if(counter_right) {
        project.RIGHT_MOTOR.forward();
      
      }
       
      if(fetchSampleLeft() < 0.38 && counter_left) {
        project.LEFT_MOTOR.stop(true);
        project.RIGHT_MOTOR.stop();
        Sound.beep();
        counter_left = false;
        try {
          Thread.sleep(200);
        }
        catch(Exception e) {
          
        }
        //project.RIGHT_MOTOR.rotate(-90);
        
        project.RIGHT_MOTOR.forward();
        if(fetchSampleLeft() < 0.38 && !counter_right) {
          project.LEFT_MOTOR.stop(true);
          project.RIGHT_MOTOR.stop();
          break;      
        }
      }
       
      if(fetchSampleRight() < 0.38 && counter_right) {
        project.RIGHT_MOTOR.stop(true);
        project.LEFT_MOTOR.stop();
        Sound.buzz();
        counter_right = false;
        try {
          Thread.sleep(200);
        }
        catch(Exception e) {
          
        }
        //project.LEFT_MOTOR.rotate(-90);
        
        project.LEFT_MOTOR.forward();
        if(fetchSampleRight() < 0.38 && !counter_left) {
          project.LEFT_MOTOR.stop(true);
          project.RIGHT_MOTOR.stop();
          break;
        }
      }
    }
      
  }
  


  
  
  /**
   * polls the left light sensor for the ground
   * @return current sample of the color
   */
  private double fetchSampleLeft() {
    double filterSum = 0;
    for(int i = 0; i < 10; i++) {
      LeftidColour.fetchSample(LeftcolorValue, 0);
      filterSum += LeftcolorValue[0];
    }
    return filterSum / 10;
  }

  /**
   * polls the right light sensor for the ground
   * @return current sample of the color
   */
  private double fetchSampleRight() {
    double filterSum = 0;
    for(int i = 0; i < 10; i++) {
      RightidColour.fetchSample(RightcolorValue, 0);
      filterSum += RightcolorValue[0];
    }
    return filterSum / 10;
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
  public static int convertDistance(double radius, double distance) {
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
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}

