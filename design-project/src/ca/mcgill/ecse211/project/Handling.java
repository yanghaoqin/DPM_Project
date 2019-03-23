package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import static ca.mcgill.ecse211.project.project.SENSOR_MOTOR;


/**
 *  this class is responsible for retrieving the can and dropping it
 *  more description to come when we actually write it
 *  
 *  @author Erica De Petrillo
 *  */
public class Handling extends Thread {

  private Odometer odo;
  private static final float SPEED = 100; //TODO: TWEAK
  
  public Handling(Odometer odometer) {
    this.odo = odometer;
  }
  
  public void run() {   
    
    while(true) {
      SENSOR_MOTOR.setSpeed(SPEED);
      SENSOR_MOTOR.forward();
      try {
        wait(200);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      SENSOR_MOTOR.stop();
      try {
        wait(200);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }     
    }       
  }
  
  public void dispose() {
    SENSOR_MOTOR.setSpeed(SPEED);
    SENSOR_MOTOR.backward();
    try {
      wait(200);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
}
