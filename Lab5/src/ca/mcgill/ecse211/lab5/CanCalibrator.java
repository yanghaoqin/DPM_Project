package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class CanCalibrator {

  //RGB indeces
  private final int RED_INDEX = 0;
  private final int GREEN_INDEX = 1;
  private final int BLUE_INDEX = 2;
  
  public boolean Calibrate(double[] target, double[] red_array, double[] green_array, double[] blue_array) {
    double[] mean = new double[3];
    double[] standard_deviation = new double[3]; 

    mean[RED_INDEX] = Find_Mean(red_array);
    mean[GREEN_INDEX] = Find_Mean(green_array);
    mean[BLUE_INDEX] = Find_Mean(blue_array);
    
    mean = Mean_Normalizer(mean[RED_INDEX], mean[GREEN_INDEX], mean[BLUE_INDEX]);
    
    standard_deviation[RED_INDEX] = Find_Standard_Deviation(red_array, mean[RED_INDEX]);
    standard_deviation[GREEN_INDEX] = Find_Standard_Deviation(green_array, mean[GREEN_INDEX]);
    standard_deviation[BLUE_INDEX] = Find_Standard_Deviation(blue_array, mean[BLUE_INDEX]);
    
    if(Compare_Standard_Deviation(target[RED_INDEX], standard_deviation[RED_INDEX], mean[RED_INDEX]) && 
        Compare_Standard_Deviation(target[GREEN_INDEX], standard_deviation[GREEN_INDEX], mean[GREEN_INDEX]) &&
        Compare_Standard_Deviation(target[BLUE_INDEX], standard_deviation[BLUE_INDEX], mean[BLUE_INDEX])) {
      Sound.beep();
      return true;
    }
    Sound.beep();
    Sound.beep();
    return false;
  }

  private double Find_Mean(double[] color_array) {
    double mean = 0;
    for (int i = 0; i < color_array.length - 1; i++) {
      mean = mean + color_array[i];
    }
    mean = mean / color_array.length;
    return mean;
  }

  private double Find_Standard_Deviation(double[] color_array, double mean) {
    double stdr_dev = 0;
    for (int i = 0; i < color_array.length - 1; i++) {
      color_array[i] = Math.sqrt(Math.pow(color_array[i] - mean, 2));
    }
    stdr_dev = Math.sqrt(Find_Mean(color_array));
    return stdr_dev;
  }

  private double[] Mean_Normalizer(double red, double green, double blue) {
    double[] rgb = new double[3];
    double euclidean = (Math.sqrt(red * red + green * green + blue * blue));
    rgb[0] = red / euclidean;
    rgb[1] = green / euclidean;
    rgb[2] = blue / euclidean;
    return rgb;
  }

  private boolean Compare_Standard_Deviation(double target, double strd_dev, double mean) {
    if(Math.abs(mean - target) < Math.abs(mean - 2 * strd_dev)) {
      return true;
    }
    else {
      return false;
    }
  }
}
