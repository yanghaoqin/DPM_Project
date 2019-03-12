package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/*this class will be responsible for assessing the weight of cans*/
public class WeightID {
	private Navigation nav;
	private Odometer odo;
	private long threashold = 5;
	public WeightID(Navigation nav, Odometer odo){
		this.nav = nav;
		this.odo = odo;
	}
	public boolean weight(){
		
		project.LEFT_MOTOR.setSpeed(150);
		project.RIGHT_MOTOR.setSpeed(150);
		long start = System.currentTimeMillis()/1000;
		nav.travelTo(((this.odo.getXYT()[0]) % project.TILE) - 1, (this.odo.getXYT()[0]) % project.TILE);
		long end = System.currentTimeMillis()/1000;
		
		if(end - start > threashold){
			return true;
		}
		return false;
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
