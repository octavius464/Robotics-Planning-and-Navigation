import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;
import lejos.hardware.Sound;

public class Robot {

	static double WHEEL_RADIUS = 0.015;
	int localizedPos;
	EV3ColorSensor colorSensor;
	RegulatedMotor mA;
	RegulatedMotor mC;
	
	public Robot(){
		colorSensor = new EV3ColorSensor(SensorPort.S4);
		mA = new EV3LargeRegulatedMotor(MotorPort.A);
		mC = new EV3LargeRegulatedMotor(MotorPort.C);
		mA.synchronizeWith(new RegulatedMotor[] { mC });
	}
	
	/**
	 * Get the color measurement from the color sensor.
	 * @return an int corresponding to blue or white;
	 */
	public int getColorMeasurement(){
		int measurement = colorSensor.getColorID();
		if(measurement == 7){ //blue:7 , white:2
			measurement = 1;
		}else{
			measurement = 0;
		}
		return measurement;
	}
	
	/**
	 * Move 1.7cm in each iteration for the localization. 
	 * @param direction 0 for moving forward, 1 for moving backward
	 */
	public void moveToLocalize(int direction){
		double meterspersecond = 2.0*Math.PI*WHEEL_RADIUS/2.0;
		double duration = (0.017 /meterspersecond) * 1000.0; //move forward 1.7cm/1 cell of the array each iteration

		mA.startSynchronization();
		mA.setSpeed(180);
		mC.setSpeed(180);
		if(direction == 0){
			mA.forward();
			mC.forward();
		}
		if(direction == 1){
			mA.backward();
			mC.backward();
		}
		mA.endSynchronization();

		Delay.msDelay((long)duration);
		mA.startSynchronization();
		mA.stop();
		mC.stop();
		mA.endSynchronization();
	}
	
}
