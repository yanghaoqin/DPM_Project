package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import ca.mcgill.ecse211.odometer.*;
// import ca.mcgill.ecse211.localization.*;
// import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.project.*;

public class DoubleLightLocalization {

  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private Navigation navigator;
  private SensorMode LeftidColour;
  private SensorMode RightidColour;

  double[] lineData;

  public DoubleLightLocalization(Odometer odometer, Navigation navigator, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, SensorModes LeftcsSensor, SensorModes RightcsSensor) {

    this.odometer = odometer;
    this.navigator = navigator;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    LeftidColour = ((EV3ColorSensor) LeftcsSensor).getMode("Red");
    LeftidColour = ((EV3ColorSensor) RightcsSensor).getMode("Red");// set the sensor light to red
    lineData = new double[4];
    // navigation = new Navigation(odometer);
  }

  public void DoubleLocalizer() {

    travelToLine();
    navigator.turnTo(90);
    travelToLine();
    navigator.turnTo(0);
    leftMotor.rotate((int) ((180.0 * 12.6) / (Math.PI * project.WHEEL_RAD)), true);
    rightMotor.rotate((int) ((180.0 * 12.6) / (Math.PI * project.WHEEL_RAD)), false);
    odometer.setXYT(0, 0, 0);
  }

  private void travelToLine() {
    this.leftMotor.setAcceleration(2000);
    this.rightMotor.setAcceleration(2000);

    this.leftMotor.setSpeed(150);
    this.rightMotor.setSpeed(150);
    
    
    while(fetchSampleLeft() > 0.38) {
      leftMotor.forward();
      while(fetchSampleRight() > 0.38) {
        rightMotor.forward();
      }
      rightMotor.stop();
    }
    leftMotor.stop();
  }
  
  private float fetchSampleLeft() {
    float[] colorValue = new float[LeftidColour.sampleSize()];
    LeftidColour.fetchSample(colorValue, 0);
    return colorValue[0];
  }

  private float fetchSampleRight() {
    float[] colorValue = new float[RightidColour.sampleSize()];
    RightidColour.fetchSample(colorValue, 0);
    return colorValue[0];
  }

}
