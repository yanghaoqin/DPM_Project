package Test;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Class for testing motor turning accuracy. Turns the motor degrees specified by user.
 * 
 * @author Raymond H. Yang
 * @Date: 2019/03/04
 */
public class turnMotor {

  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------
  
  /**
   * Motor instance, specify port motor is connected to.
   */
  private static final EV3LargeRegulatedMotor MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The amount of turning in degrees
   */
  private static final int DEG = 1440;
  
  /**
   * The acceleration of the motor
   */
  private static final int ACCELERATION = 6000;

  /**
   * The speed of the motor
   */
  private static final int SPEED = 250;
  
  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------
  
  /**
   * Main method
   * @param args - None
   */
  public static void main(String args[]) {
    
    // set acceleration and speed
    MOTOR.setAcceleration(ACCELERATION);
    MOTOR.setSpeed(SPEED);
    
    // reset count of degrees rotated
    MOTOR.resetTachoCount();
    
    // record current tacho count
    int initialCount = MOTOR.getTachoCount();
    
    // signal status
    System.out.println("Standby...");

    // rotate the motor
    MOTOR.rotate(DEG, false);
    MOTOR.stop();
    
    // record current tacho count
    int finalCount = MOTOR.getTachoCount();
    
    try {
		Thread.sleep(500);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
    // signal completion
    System.out.println("Complete: " + DEG + " degrees");

    // record rotation
    System.out.println("Recorded Rotation: " + (finalCount - initialCount) + " degrees");
    
  }
}
