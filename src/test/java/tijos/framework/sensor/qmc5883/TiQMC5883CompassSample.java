package tijos.framework.sensor.qmc5883;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;

public class TiQMC5883CompassSample {

	public static void main(String[] args) {
		try {
			/*
			 * 定义使用的TiI2CMaster port
			 */
			int i2cPort0 = 0;

			/*
			 * 资源分配， 将i2cPort0分配给TiI2CMaster对象i2c0
			 */
			TiI2CMaster i2c0 = TiI2CMaster.open(i2cPort0);

			TiQMC5883 compass = new TiQMC5883(i2c0);

			compass.initialize();
			
		    compass.setRange(TiQMC5883.RANGE.GA_2);
		    compass.setMeasurementMode(TiQMC5883.MODE.CONTINOUS); 
		    compass.setDataRate(TiQMC5883.DATARATE.HZ_50);
		    compass.setSamples(TiQMC5883.SAMPLES.SAMPLES_8);
		    
			while (true) {
				try {
					 Vector norm = compass.readNormalize();

					  // Calculate heading
					  double heading = Math.atan2(norm.YAxis, norm.XAxis);

					  // Set declination angle on your location and fix heading
					  // You can find your declination on: http://magnetic-declination.com/
					  // (+) Positive or (-) for negative
					  // For Bytom / Poland declination angle is 4'26E (positive)
					  // Formula: (deg + (min / 60.0)) / (180 / PI);
					  double declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / Math.PI);
					  heading += declinationAngle;

					  // Correct for heading < 0deg and heading > 360deg
					  if (heading < 0){
					    heading += 2 * Math.PI;
					  }

					  if (heading > 2 * Math.PI){
					    heading -= 2 * Math.PI;
					  }

					  // Convert to degrees
					  double headingDegrees = heading * 180/Math.PI; 

					  // Output
					  System.out.println(" Heading = " + heading);
					  System.out.println(" Degress = " + headingDegrees);
				}
				catch (IOException ie) {
					ie.printStackTrace();
				}

				
			}
				
		}
		catch (IOException ie) {
			ie.printStackTrace();
		}

	}

}
