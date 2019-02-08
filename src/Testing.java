import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import java.util.ArrayList;
import java.util.HashMap;

public class Testing {

	public static void main(String args[]){
		/*
		Robot1 robot = new Robot1();
		ArrayList<Integer> measurements = new ArrayList<>();
		float sum=0;
		float distance = 0.05f; 
		float increment = 0.05f;
		for(int i=0;i<10;++i){
			robot.moveForward(distance);		
			Delay.msDelay(10000);	
		}	
		
		Delay.msDelay(100000);	
		
		while(true){
			System.out.println(robot.getColorMeasurement());	
		}
		*/
		HashMap<int[],int[]> testMap = new HashMap();
		int[] a = {1,2};
		int[] b = {2,3};
		int[] c = {4,5};
		int[] d = {1,5};
		int[] e = {3,2};
		int[] aScore = {1,2,3};
		int[] bScore = {3,2,1};
		testMap.put(a, aScore);
		testMap.put(b, bScore);
		System.out.println(testMap.containsKey(a));
		System.out.println(testMap.containsKey(d));
		System.out.println(testMap.containsKey(e));
	}
}


class Robot1 {

	static double WHEEL_RADIUS = 0.015;
	int localizedPos;
	EV3ColorSensor colorSensor;
	RegulatedMotor mA;
	RegulatedMotor mC;
	
	public Robot1(){
		colorSensor = new EV3ColorSensor(SensorPort.S4);
		mA = new EV3LargeRegulatedMotor(MotorPort.A);
		mC = new EV3LargeRegulatedMotor(MotorPort.C);
		mA.synchronizeWith(new RegulatedMotor[] { mC });
	}
	
	/**
	 * Get the color measurement from the color sensor. If it is blue, then return 1, else return 0.
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
		double duration = (0.0085 /meterspersecond) * 1000.0; //move forward 1.7cm/cell of the array each iteration

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
	
	/**
	 * Move 1.7cm in each iteration for the localization. 
	 * @param direction 0 for moving forward, 1 for moving backward
	 */
	public void moveForward(float distance){
		double meterspersecond = 2.0*Math.PI*WHEEL_RADIUS/2.0;
		double duration = (distance /meterspersecond) * 1000.0; //move forward 1.7cm/cell of the array each iteration

		mA.startSynchronization();
		mA.setSpeed(180);
		mC.setSpeed(180);
		mA.forward();
		mC.forward();
		
		mA.endSynchronization();

		Delay.msDelay((long)duration);
		mA.startSynchronization();
		mA.stop();
		mC.stop();
		mA.endSynchronization();
	}
	
}
