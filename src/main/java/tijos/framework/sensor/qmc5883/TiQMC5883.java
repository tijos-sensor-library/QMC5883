package tijos.framework.sensor.qmc5883;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.util.BigBitConverter;

/**
 * QMC5883 compass module driver for TiJOS based on 
 * https://github.com/DFRobot/DFRobot_QMC5883
 */

class TiQMC5883Register {

	public static final int QMC5883_REG_OUT_X_M = 0x01;
	public static final int QMC5883_REG_OUT_X_L = 0x00;
	public static final int QMC5883_REG_OUT_Z_M = 0x05;
	public static final int QMC5883_REG_OUT_Z_L = 0x04;
	public static final int QMC5883_REG_OUT_Y_M = 0x03;
	public static final int QMC5883_REG_OUT_Y_L = 0x02;
	public static final int QMC5883_REG_STATUS = 0x06;
	public static final int QMC5883_REG_CONFIG_1 = 0x09;
	public static final int QMC5883_REG_CONFIG_2 = 0x0A;
	public static final int QMC5883_REG_IDENT_B = 0x0B;
	public static final int QMC5883_REG_IDENT_C = 0x20;
	public static final int QMC5883_REG_IDENT_D = 0x21;

}

class Vector {
	float XAxis;
	float YAxis;
	float ZAxis;
};

public class TiQMC5883 {
	public static final int QMC5883_ADDRESS = 0x0D;

	public enum SAMPLES {
		SAMPLES_1, SAMPLES_2, SAMPLES_4, SAMPLES_8
	};

	public enum DATARATE {
		HZ_10, HZ_50, HZ_100, HZ_200
	}

	public enum RANGE {
		GA_2, GA_8
	};

	public enum MODE {
		SINGLE, CONTINOUS
	};

	/**
	 * TiI2CMaster object
	 */
	private TiI2CMaster i2cmObj;

	private int i2cSlaveAddr = QMC5883_ADDRESS;

	private byte[] data = new byte[6];

	float mgPerDigit;
	Vector v = new Vector();
	int xOffset, yOffset;

	float minX, maxX;
	float minY, maxY;
	float minZ, maxZ;
	boolean firstRun = false;

	/**
	 * Initialize with I2C master and default I2C slave address 0x1E
	 * 
	 * @param i2c
	 */
	public TiQMC5883(TiI2CMaster i2c) {
		this(i2c, QMC5883_ADDRESS);
	}

	/**
	 * Initialize with I2C master and slave address
	 * 
	 * @param i2c
	 * @param address
	 */
	public TiQMC5883(TiI2CMaster i2c, int address) {
		this.i2cmObj = i2c;
		this.i2cSlaveAddr = address;
	}

	/**
	 * Initialize the sensor with default settings
	 * @throws IOException
	 */
	public void initialize() throws IOException {

		data[0] = 0x01;
		this.i2cmObj.write(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_IDENT_B, data, 0, 1);

		data[0] = 0x40;
		this.i2cmObj.write(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_IDENT_C, data, 0, 1);

		data[0] = 0X01;
		this.i2cmObj.write(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_IDENT_D, data, 0, 1);

		data[0] = 0X1D;
		this.i2cmObj.write(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);

		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_IDENT_B, data, 0, 1);
		if (data[0] != 0x01)
			throw new IOException("Invalid identity");

		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_IDENT_C, data, 0, 1);
		if (data[0] != 0x40)
			throw new IOException("Invalid identity");

		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_IDENT_D, data, 0, 1);
		if (data[0] != 0x01)
			throw new IOException("Invalid identity");

		setRange(RANGE.GA_8);
		setMeasurementMode(MODE.CONTINOUS);
		setDataRate(DATARATE.HZ_50);
		setSamples(SAMPLES.SAMPLES_8);
		mgPerDigit = 4.35f;
		this.firstRun = false;

	}

	/**
	 * read raw data 
	 * @return raw data
	 * @throws IOException
	 */
	public Vector readRaw() throws IOException {
		int range = 10;
		float Xsum = 0.0f;
		float Ysum = 0.0f;
		float Zsum = 0.0f;

		while (range-- > 0) {
			v.XAxis = readRegister16(TiQMC5883Register.QMC5883_REG_OUT_X_M);
			v.YAxis = readRegister16(TiQMC5883Register.QMC5883_REG_OUT_Y_M);
			v.ZAxis = readRegister16(TiQMC5883Register.QMC5883_REG_OUT_Z_M);
			calibrate();
			Xsum += v.XAxis;
			Ysum += v.YAxis;
			Zsum += v.ZAxis;
		}
		v.XAxis = Xsum / range;
		v.YAxis = Ysum / range;
		v.ZAxis = Zsum / range;
		
		if (firstRun) {
			initMinMax();
			firstRun = false;
		}
		
		return v;
	}
	
	/**
	 * read Normalize data
	 * @return Normalized data
	 * @throws IOException
	 */
	public Vector readNormalize()  throws IOException {
		
		 int range = 10;
		  float Xsum = 0.0f;
		  float Ysum = 0.0f;
		  float Zsum = 0.0f;
		  
		  while (range-- > 0){
		      v.XAxis = ((float)readRegister16(TiQMC5883Register.QMC5883_REG_OUT_X_M)) * mgPerDigit;
		      v.YAxis = ((float)readRegister16(TiQMC5883Register.QMC5883_REG_OUT_Y_M)) * mgPerDigit;
		      v.ZAxis = (float)readRegister16(TiQMC5883Register.QMC5883_REG_OUT_Z_M) * mgPerDigit;
		      Xsum += v.XAxis;
		      Ysum += v.YAxis;
		      Zsum += v.ZAxis;
		    }
		    v.XAxis = Xsum/range;
		    v.YAxis = Ysum/range;
		    v.ZAxis = Zsum/range;
		    if(firstRun){
		      initMinMax();
		      firstRun = false;
		    }
		    
		    calibrate();
		    v.XAxis= map(v.XAxis,minX,maxX,-360,360);
		    v.YAxis= map(v.YAxis,minY,maxY,-360,360);
		    v.ZAxis= map(v.ZAxis,minZ,maxZ,-360,360);
		    
		    return v;
	}

	public void setRange(RANGE range) throws IOException {
		switch (range) {
		case GA_2:
			mgPerDigit = 1.22f;
			break;

		case GA_8:
			mgPerDigit = 4.35f;
			break;

		default:
			break;
		}

		int value = (int) range.ordinal() << 4;

		data[0] = (byte) value;

		this.i2cmObj.write(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_2, data, 0, 1);
	}

	public RANGE getRange() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_2, data, 0, 1);

		return RANGE.values()[data[0] >> 4];
	}

	public void setMeasurementMode(MODE mode) throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);
		int value = data[0];
		value &= 0xfc;
		value |= mode.ordinal();

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);

	}

	public MODE getMeasurementMode() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);
		int value = data[0];
		value &= 0b00000011;

		return MODE.values()[value];

	}

	public void setDataRate(DATARATE dataRate) throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);
		int value = data[0];
		value &= 0xf3;
		value |= (dataRate.ordinal() << 2);

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);

	}

	public DATARATE getDataRate() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);
		int value = data[0];
		value &= 0b00001100;
		value >>= 2;

		return DATARATE.values()[value];
	}

	public void setSamples(SAMPLES samples) throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);
		int value = data[0];
		value &= 0x3f;
		value |= (samples.ordinal() << 6);

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);
	}

	public SAMPLES getSamples() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiQMC5883Register.QMC5883_REG_CONFIG_1, data, 0, 1);
		int value = data[0];
		value &= 0x3f;
		value >>= 6;

		return SAMPLES.values()[value];

	}

	// Read word from register
	private int readRegister16(int reg) throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, reg, data, 0, 2);
		return BigBitConverter.ToInt16(data, 0);
	}

	// The map function implementation from the Arduino¡¯s
	private float map(float x, float in_min, float in_max, int out_min, int out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
	

	private void calibrate() {
		if (v.XAxis < minX)
			minX = v.XAxis;
		if (v.XAxis > maxX)
			maxX = v.XAxis;
		if (v.YAxis < minY)
			minY = v.YAxis;
		if (v.YAxis > maxY)
			maxY = v.YAxis;
		if (v.ZAxis < minZ)
			minZ = v.ZAxis;
		if (v.ZAxis > maxZ)
			maxZ = v.ZAxis;
	}

	private void initMinMax() {
		minX = v.XAxis;
		maxX = v.XAxis;
		minY = v.YAxis;
		maxY = v.YAxis;
		minZ = v.ZAxis;
		maxZ = v.ZAxis;
	}
}
