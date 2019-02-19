package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class CanCalibrator {

  private double std;
  private float[] lightData;
  private SampleProvider lightColor;
  private float filterSum;
  
  //RGB indeces
  private final int RED_INDEX = 0;
  private final int GREEN_INDEX = 1;
  private final int BLUE_INDEX = 2;
  
  private final int SCAN_TIME = 100;
  private final int SCAN_LENGTH = 100;

  public CanCalibrator(SampleProvider lightColor, float[] lightData) {
    this.lightData = lightData;
    this.lightColor = lightColor;
    this.std = 0;
  }


  private boolean Calibrate(double[] target, boolean stop) {
    double[] red_array = new double[SCAN_LENGTH];
    double[] green_array = new double[SCAN_LENGTH];
    double[] blue_array = new double[SCAN_LENGTH];
    double[] mean = new double[3];
    double[] standard_deviation = new double[3]; 
    /*
     * while(!stop) {
     * 
     * }
     */
    for (int i = 0; i < SCAN_TIME; i++) {
      red_array[i] = initialReading(RED_INDEX);
      green_array[i] = initialReading(GREEN_INDEX);
      blue_array[i] = initialReading(BLUE_INDEX);
    }

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
  
  /**
   * initial readings taken (100 times) and the average is used to distinguish between the wooden
   * board and the black line. Each reading is amplified to enhance the sensitivity of the sensor
   * 
   * @return
   */
  private double initialReading(int color_id) {
    for (int i = 0; i < 100; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, color_id);
      // amplifies and sums the sample data
      std += lightData[color_id] * 100.0;
    }
    // take average of the standard
    return std /= 100.0;
  }

  /**
   * This is a private method which functions as a mean or average filter. The filter ensures the
   * correctness of the readings, filtering out the noise in the signal from the color sensor. The
   * filter takes 5 readings and sums the amplified value of each reading.
   * 
   * @return returns the average of the amplified reading
   */
  private double meanFilter() {
    filterSum = 0;
    // take 5 readings
    for (int i = 0; i < 5; i++) {

      // acquire sample data and read into array with no offset
      lightColor.fetchSample(lightData, 0);

      // amplify signal for increased sensitivity
      filterSum += lightData[0] * 100;

    }

    // return an amplified average
    return filterSum / 5.0;

  }
}
