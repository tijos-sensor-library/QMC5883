package tijos.framework.sensor.qmc5883;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;

import tijos.util.Delay;

/**
 * Hello world!
 *
 */

public class TiQMC5883Sample
{
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

		    int num = 100;
			while (num-- > 0) {
				try {

					Vector raw = compass.readRaw();
					Vector norm = compass.readNormalize();

					System.out.print(" Xraw = ");
					System.out.print(raw.XAxis);
					System.out.print(" Yraw = ");
					System.out.print(raw.YAxis);
					System.out.print(" Zraw = ");
					System.out.print(raw.ZAxis);
					System.out.print(" Xnorm = ");
					System.out.print(norm.XAxis);
					System.out.print(" Ynorm = ");
					System.out.print(norm.YAxis);
					System.out.print(" ZNorm = ");
					System.out.print(norm.ZAxis);
					System.out.println();

				} catch (Exception ex) {

					ex.printStackTrace();
				}

			}
		} catch (IOException ie) {
			ie.printStackTrace();
		}
	}
}
