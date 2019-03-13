package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this class will be responsible for assessing the weight of cans. 
 * 
 * @author YunHao
 */
public class WeightID {
	private Navigation nav;
	private Odometer odo;
	private long threashold = 4180;
	
	/**
	 * The constructor method for WeightID class
	 * 
	 * @param nav  Instance of the class navigation
	 * @param odo  Instance of the class odometer
	 */
	public WeightID(Navigation nav, Odometer odo){
		this.nav = nav;
		this.odo = odo;
	}
	/**
	 * this method will travel back one tile and measure the time used.
	 * If the time is greater than a certain threshold, it is heavy, else it is light.
	 * @return a boolean that determine whether the can is heavy (true) or light (false) 
	 */
	public boolean weight(){
		
		project.LEFT_MOTOR.setSpeed(100);
		project.RIGHT_MOTOR.setSpeed(100);
		long start = System.currentTimeMillis();
		nav.travelTo(((this.odo.getXYT()[0]) % project.TILE), (this.odo.getXYT()[0]) % project.TILE + 1);
		long end = System.currentTimeMillis();
		
		if(end - start > threashold){
			
		  System.out.println(end - start);
		  return true;
			
		}
		System.out.println(end - start);
		return false;
		}
	}
