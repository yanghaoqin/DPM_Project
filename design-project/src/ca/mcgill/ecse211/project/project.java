package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/* this is the main class*/

public class project {
  
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  
  public static final double TILE = 30.48;
  
  public static final double WHEEL_RAD = 0;//TODO: FIND VALUE
  
  public static final double TRACK = 0; //TODO: FIND VALUE

}
