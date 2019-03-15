package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * this class will be responsible for assessing the weight of cans. 
 * 
 * @author YunHao
 */
public class WeightID {
	private SampleProvider LeftcsSensor;
	private float[] LeftcolorValue;
	private long threashold = 4200;
	
	/**
	 * The constructor method for WeightID class
	 * 
	 * @param nav  Instance of the class navigation
	 * @param odo  Instance of the class odometer
	 */
	public WeightID(SampleProvider LeftcsSensor,float[] LeftcolorValue){
		this.LeftcsSensor = LeftcsSensor;
		this.LeftcolorValue = LeftcolorValue;
	}
	/**
	 * this method will travel back one tile and measure the time used.
	 * If the time is greater than a certain threshold, it is heavy, else it is light.
	 * @return a boolean that determine whether the can is heavy (true) or light (false) 
	 */
	public boolean weight(){
		
	    project.LEFT_MOTOR.setAcceleration(100);
        project.RIGHT_MOTOR.setAcceleration(100);
	  
		project.LEFT_MOTOR.setSpeed(200);
		project.RIGHT_MOTOR.setSpeed(200);
		boolean stop = false;
		boolean first = false;
		long start = 0;
		long end = 0;
		while(!stop) {
		  project.LEFT_MOTOR.forward();
		  project.RIGHT_MOTOR.forward();
		  if(fetchSampleLeft() < 0.38 && !first) {
		    Sound.beep();
		    first = true;
		    start = System.currentTimeMillis();
		  }
		  if(fetchSampleLeft() < 0.38 && first) {
		    Sound.beep();
		    end = System.currentTimeMillis();
		    stop = true;
		  }
		}
        project.LEFT_MOTOR.stop(true);
        project.RIGHT_MOTOR.stop();		

        System.out.println(end - start);
		if(end - start > threashold){
		  Sound.buzz();
		  return true;
			
		}
		return false;
		}
	
	  private double fetchSampleLeft() {
	    double filterSum = 0;
	    for(int i = 0; i < 10; i++) {
	      LeftcsSensor.fetchSample(LeftcolorValue, 0);
	      filterSum += LeftcolorValue[0];
	    }
	    return filterSum / 10;
	  }
	}
