package ca.mcgill.ecse211.lab5;

import lejos.robotics.SampleProvider;

public class CanCalibrator {
  
  private SampleProvider lightColor;
  private float[] lightData;
  private double std = 0;
  public static double[] mean = new double[3];
  public static double[] standard_deviation = new double[3];
  
  
  public CanCalibrator(SampleProvider lightColor, float[] lightData) {
    this.lightColor = lightColor;
    this.lightData = lightData;
  }
  
  
  //RGB indexes
  private final int RED_INDEX = 0;
  private final int GREEN_INDEX = 1;
  private final int BLUE_INDEX = 2;
  
  public boolean Calibrate(double[] target) {
    double[] red_array = new double[100];
    double[] green_array = new double[100];
    double[] blue_array = new double[100];
        
    for(int i = 0; i < 100; i++) {
      red_array[i] = initialReading(RED_INDEX);
      green_array[i] = initialReading(GREEN_INDEX);
      blue_array[i] = initialReading(BLUE_INDEX);
    }
   
    mean[RED_INDEX] = Find_Mean(red_array);
    mean[GREEN_INDEX] = Find_Mean(green_array);
    mean[BLUE_INDEX] = Find_Mean(blue_array);

    standard_deviation[RED_INDEX] = Find_Standard_Deviation(red_array, mean[RED_INDEX]);
    standard_deviation[GREEN_INDEX] = Find_Standard_Deviation(green_array, mean[GREEN_INDEX]);
    standard_deviation[BLUE_INDEX] = Find_Standard_Deviation(blue_array, mean[BLUE_INDEX]);
    
    mean = Mean_Normalizer(mean[RED_INDEX], mean[GREEN_INDEX], mean[BLUE_INDEX]);
    
    if(Compare_Standard_Deviation(target[RED_INDEX], standard_deviation[RED_INDEX], mean[RED_INDEX]) && 
        Compare_Standard_Deviation(target[GREEN_INDEX], standard_deviation[GREEN_INDEX], mean[GREEN_INDEX]) &&
        Compare_Standard_Deviation(target[BLUE_INDEX], standard_deviation[BLUE_INDEX], mean[BLUE_INDEX])) {
      return true;
    }
    return false;
  }

  private double Find_Mean(double[] color_array) {
    double mean = 0;
    for (int i = 0; i < color_array.length; i++) {
      mean = mean + color_array[i];
    }
    mean = mean / color_array.length;
    return mean;
  }

  private double Find_Standard_Deviation(double[] color_array, double mean) {
    double stdr_dev = 0;
    double [] temp_array = new double[color_array.length];
    for (int i = 0; i < temp_array.length; i++) {
      temp_array[i] = (color_array[i] - mean)*(color_array[i] - mean);
    }
    for(int i = 0; i < temp_array.length; i++) {
      stdr_dev += temp_array[i];
    }
    stdr_dev = Math.sqrt(stdr_dev / (temp_array.length - 1));
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
  
  private double initialReading(int index) {
    for (int i = 0; i < 100; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, 0);
      // amplifies and sums the sample data
      std += lightData[index];
    }
    // take average of the standard
    return std /= 100.0;
  }  
}
