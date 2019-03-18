package ca.mcgill.ecse211.project;

import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.project.BLUE_COLOR;
import static ca.mcgill.ecse211.project.project.YELLOW_COLOR;
import static ca.mcgill.ecse211.project.project.GREEN_COLOR;
import static ca.mcgill.ecse211.project.project.RED_COLOR;

/**
 * This class reads the color value. Process it through mean and stdev calculation. 
 * Then the value is compared with the threshold to determine which color it is or
 * it is the target color.
 * @author Antoine Wang
 * @author Erica de Petrillo
 * @author Raymond Yang
 * 
 */
public class CanCalibrator {

  private SampleProvider lightColor;
  private float[] lightData;
  private double std = 0;
  public static double[] mean = new double[3];
  public static double[] standard_deviation = new double[3];

  // RGB indexes
  private static final int RED_INDEX = 4;
  private static final int GREEN_INDEX = 2;
  private static final int BLUE_INDEX = 1;
  private static final int YELLOW_INDEX = 3;


  /**
   * 
   * This is the constructor of the can Calibrator class. To create an instance of the can Calibrator class, the sample
   * provider and the array for storing the reading of the sensor is taken in.
   * 
   * @param lightColor Sampleprovider of the light sensor
   * @param lightData array to store the data
   */
  public CanCalibrator(SampleProvider lightColor, float[] lightData) {
    this.lightColor = lightColor;
    this.lightData = lightData;
  }

  /**
   * This is the classifier.
   * The method Calibrate takes no input. It calls the initialReading() method to retrieve the value of the color reading
   * Then the value is processed by finding the mean value of R,G and B readings.
   * Also the standard deviation is later calculated and then the filtered RGB reading is normalized with in the range of 0-1;
   * At the end, the isColor() method is called. The calculated value is compared with the theoretical threshold to return the integer
   * value representing the color index
   * @return int corresponding color index
   */
  public int Calibrate() {

    // initialize array
    double[] red_array = new double[50];
    double[] green_array = new double[50];
    double[] blue_array = new double[50];

    // take initial reading
    // record the computed mean for 50 times for rgb
    for (int i = 0; i < 50; i++) {
      // each array contains 100 values
      red_array[i] = initialReading(RED_INDEX);
      green_array[i] = initialReading(GREEN_INDEX);
      blue_array[i] = initialReading(BLUE_INDEX);
    }

    // compute to find mean of the 50 means
    // mean has only 3 values; r,g,b
    mean[RED_INDEX] = Find_Mean(red_array);
    mean[GREEN_INDEX] = Find_Mean(green_array);
    mean[BLUE_INDEX] = Find_Mean(blue_array);

    // find the standard deviation of each rgb array which contains 100 means
    standard_deviation[RED_INDEX] = Find_Standard_Deviation(red_array, mean[RED_INDEX]);
    standard_deviation[GREEN_INDEX] = Find_Standard_Deviation(green_array, mean[GREEN_INDEX]);
    standard_deviation[BLUE_INDEX] = Find_Standard_Deviation(blue_array, mean[BLUE_INDEX]);

    // normalize the mean to a range of 0 - 1
    mean = Mean_Normalizer(mean[RED_INDEX], mean[GREEN_INDEX], mean[BLUE_INDEX]);

    // determine which color
    return isColor();
  }

  /**
   * 
   * This is the overloaded classifier method. This method takes in an array of rgb value which is the 
   * pre-defined theoretical threshold. Then the processed RGB data is compared. A boolean will be returned if the color matched
   * 
   * @param target Theoretical threshold color RGB value
   * @return boolean whether the read color corresponding to the target threshold
   */
  public boolean Calibrate(double[] target) {

    // initialize array
    double[] red_array = new double[50];
    double[] green_array = new double[50];
    double[] blue_array = new double[50];

    // take initial reading
    // record the computed mean for 100 times for rgb
    for (int i = 0; i < 50; i++) {
      // each array contains 100 values
      red_array[i] = initialReading(RED_INDEX);
      green_array[i] = initialReading(GREEN_INDEX);
      blue_array[i] = initialReading(BLUE_INDEX);
    }

    // compute to find mean of the 50 means
    // mean has only 3 values; r,g,b
    mean[RED_INDEX] = Find_Mean(red_array);
    mean[GREEN_INDEX] = Find_Mean(green_array);
    mean[BLUE_INDEX] = Find_Mean(blue_array);

    // find the standard deviation of each rgb array which contains 100 means
    standard_deviation[RED_INDEX] = Find_Standard_Deviation(red_array, mean[RED_INDEX]);
    standard_deviation[GREEN_INDEX] = Find_Standard_Deviation(green_array, mean[GREEN_INDEX]);
    standard_deviation[BLUE_INDEX] = Find_Standard_Deviation(blue_array, mean[BLUE_INDEX]);

    // normalize the mean to a range of 0 - 1
    mean = Mean_Normalizer(mean[RED_INDEX], mean[GREEN_INDEX], mean[BLUE_INDEX]);

    // compare the read color value to the target threshold, also compare the stdev to eliminate false readings
    if (Compare_Standard_Deviation(target[RED_INDEX], standard_deviation[RED_INDEX],
        mean[RED_INDEX])
        && Compare_Standard_Deviation(target[GREEN_INDEX], standard_deviation[GREEN_INDEX],
            mean[GREEN_INDEX])
        && Compare_Standard_Deviation(target[BLUE_INDEX], standard_deviation[BLUE_INDEX],
            mean[BLUE_INDEX])) {
      return false;
    }
    return false;
  }

  /**
   * 
   * Compute the mean
   * Sum up all values and then divide by the sample size to remove outlier readings
   * 
   * @param color_array samples from the color sensor
   * @return double the mean of the reading
   */
  private double Find_Mean(double[] color_array) {
    double mean = 0;
    for (int i = 0; i < color_array.length; i++) {
      mean = mean + color_array[i];
    }
    mean = mean / color_array.length;
    return mean;
  }

  /**
   * 
   * Compute the standard deviation
   * Takes in the mean value and the samples
   * Using the sample standard deviation formula to compute the standard deviation
   * @param color_array sample data point
   * @param mean average value
   * @return double standard deviation value of the data set (for a single color reading, like R or G)
   */
  private double Find_Standard_Deviation(double[] color_array, double mean) {
    double stdr_dev = 0;
    double[] temp_array = new double[color_array.length];
    for (int i = 0; i < temp_array.length; i++) {
      temp_array[i] = (color_array[i] - mean) * (color_array[i] - mean);
    }
    for (int i = 0; i < temp_array.length; i++) {
      stdr_dev += temp_array[i];
    }
    stdr_dev = Math.sqrt(stdr_dev / (temp_array.length - 1));
    return stdr_dev;
  }

  /**
   * A method to normalize the values to a range from 0 to 1
   * Divide the RGB value with the euclidian distance.
   * 
   * @param red
   * @param green
   * @param blue
   * @return
   */
  private double[] Mean_Normalizer(double red, double green, double blue) {
    double[] rgb = new double[3];
    double euclidean = (Math.sqrt(red * red + green * green + blue * blue));
    rgb[0] = red / euclidean;
    rgb[1] = green / euclidean;
    rgb[2] = blue / euclidean;
    return rgb;
  }

  /**
   * Find out whether the target value is within 2 standard deviations of the measured readings
   * If yes, then the color we read can be decided as the target color
   * Or the read color is defined as a different color
   * @param target threshold for a certain color
   * @param strd_dev stdev of a certain color's data set
   * @param mean mean of a certain color's data set
   * @return boolean whether the color is considered "different"
   */
  private boolean Compare_Standard_Deviation(double target, double strd_dev, double mean) {
    if (Math.abs(mean - target) < Math.abs(mean - 2 * strd_dev)) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Determines whether measured color is target color by determining whether the values lies in
   * 2 stdev
   * @return int color judged (R,G,B or Y)
   */
  private int isColor() {
    if (Compare_Standard_Deviation(YELLOW_COLOR[RED_INDEX], standard_deviation[RED_INDEX],
        mean[RED_INDEX])
    	&& Compare_Standard_Deviation(YELLOW_COLOR[GREEN_INDEX], standard_deviation[GREEN_INDEX],
            mean[GREEN_INDEX])
        && Compare_Standard_Deviation(YELLOW_COLOR[BLUE_INDEX], standard_deviation[BLUE_INDEX],
            mean[BLUE_INDEX])) {
       return YELLOW_INDEX;
    } else if(Compare_Standard_Deviation(GREEN_COLOR[RED_INDEX], standard_deviation[RED_INDEX],
        mean[RED_INDEX])
        && Compare_Standard_Deviation(GREEN_COLOR[GREEN_INDEX], standard_deviation[GREEN_INDEX],
            mean[GREEN_INDEX])
        && Compare_Standard_Deviation(GREEN_COLOR[BLUE_INDEX], standard_deviation[BLUE_INDEX],
            mean[BLUE_INDEX])){
      return GREEN_INDEX;
    } else if(Compare_Standard_Deviation(BLUE_COLOR[RED_INDEX], standard_deviation[RED_INDEX],
        mean[RED_INDEX])
        && Compare_Standard_Deviation(BLUE_COLOR[GREEN_INDEX], standard_deviation[GREEN_INDEX],
            mean[GREEN_INDEX])
        && Compare_Standard_Deviation(BLUE_COLOR[BLUE_INDEX], standard_deviation[BLUE_INDEX],
            mean[BLUE_INDEX])) {
      return BLUE_INDEX;
    } else if(Compare_Standard_Deviation(RED_COLOR[RED_INDEX], standard_deviation[RED_INDEX],
            mean[RED_INDEX])
            && Compare_Standard_Deviation(RED_COLOR[GREEN_INDEX], standard_deviation[GREEN_INDEX],
                mean[GREEN_INDEX])
            && Compare_Standard_Deviation(RED_COLOR[BLUE_INDEX], standard_deviation[BLUE_INDEX],
                mean[BLUE_INDEX])) {
          return RED_INDEX;
    } else {
      // other color
      return -1;
    }
  }

  /**
   * 
   * Take initial readings (10 times), compute the mean of the 10 readings and return as one filtered datapoint
   * 
   * @param index Index of the color array (r=0, g=1, b=2)
   * @return mean
   */
  private double initialReading(int index) {
    for (int i = 0; i < 10; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, 0);
      // place reading into corresponding place
      std += lightData[index];
    }
    // take average of the standard
    return std /= 100.0;
  }
}
