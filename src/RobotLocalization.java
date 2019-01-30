import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;
import lejos.hardware.Sound;

public class RobotLocalization {

	static float carOrientation;
	static double wheel_radius = 0.015;
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);
	
	public static void main(String args[]){
		/*
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S1);
		SampleProvider touchMode = touchSensor.getTouchMode();
		*/
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4); // red:0, white:2 black :7
		
		
		gyroSensor.reset();
		SampleProvider angleMode = gyroSensor.getAngleMode();
		float[] angleSample = new float[angleMode.sampleSize()];
		angleMode.fetchSample(angleSample, 0); // Gyro sensing
		carOrientation = angleSample[angleMode.sampleSize() - 1];
		RegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.A);
		RegulatedMotor mC = new EV3LargeRegulatedMotor(MotorPort.C);
		mB.synchronizeWith(new RegulatedMotor[] { mC });
		while(true){
			/*
			float[] touchSample = new float[touchMode.sampleSize()];
			touchMode.fetchSample(touchSample, 0);
			float measuredTouch = touchSample[touchMode.sampleSize() - 1];
			if(measuredTouch == 1){
				Sound.beep();
			}
			colorSensor.getColorID();
			*/
			System.out.println(colorSensor.getColorID());
			/*
			for(int i=0; i<4; i++){
				double meterspersecond = 2.0*Math.PI*wheel_radius/2.0;
				double duration = (0.5 /meterspersecond) * 1000.0;
				
		
				mB.startSynchronization();
				mB.setSpeed(180);
				mC.setSpeed(180);
				mB.forward();
				mC.forward();
				mB.endSynchronization();
	
				Delay.msDelay((long)duration);
				mB.startSynchronization();
				mB.stop();
				mC.stop();
				mB.endSynchronization();
				
				gyroSensor.reset();
				while (carOrientation <= 90.0) { 
					System.out.println(carOrientation);
					mB.setSpeed(100);
					mB.backward();
					mC.setSpeed(100);
					mC.forward();				
					angleMode.fetchSample(angleSample, 0); // Gyro sensing
					carOrientation = angleSample[angleMode.sampleSize() - 1];
				}
				carOrientation=0;
				mB.startSynchronization();
				mB.stop();
				mC.stop();
				mB.endSynchronization();
				
				Delay.msDelay(5000);
			}
			*/
			/*
			for(int i = 0 ; i<10 ; i++){
				move20mmforward();
				Delay.msDelay(3000);	
			}*/
			/*
			for(int i = 0 ; i<10 ; i++){
				move20mmbackward();
				Delay.msDelay(30000);	
			}
			for(int i = 0 ; i<4 ; i++){
				repeatedRotation();
				Delay.msDelay(30000);	
			}
			
			repeatedRotation2();
			
			*/
		}
		
		
		
		
	}
	static public void move20mmforward(){
		double meterspersecond = 2.0*Math.PI*wheel_radius/4.0;
		double duration = (2 /meterspersecond) * 1000.0;
		System.out.println((long)duration);
		Motor.A.setSpeed(90); // degrees per second
		Motor.A.forward();
		Motor.C.setSpeed(90);
		Motor.C.forward();
		Delay.msDelay((long)duration);
		Motor.C.stop();
		Motor.A.stop();
		
	}
	static public void move20mmbackward(){
		double meterspersecond = 2.0*Math.PI*wheel_radius/4.0;
		double duration = (0.02 /meterspersecond) * 1000.0;
		Motor.A.setSpeed(90); // degrees per second
		Motor.A.backward();
		Motor.C.setSpeed(90);
		Motor.C.backward();
		Delay.msDelay((long)duration);
		Motor.A.stop();
		Motor.C.stop(); 
	}
	
	static public void repeatedRotation(){ //measure error in x,y, angle
		SampleProvider angleMode = gyroSensor.getAngleMode();
		float[] angleSample = new float[angleMode.sampleSize()];
		
		double duration =  0.5 /(2.0*Math.PI*wheel_radius/4.0) * 1000.0; //duration to move 50cm
		Motor.A.setSpeed(90); //  degrees per second
		Motor.A.forward();
		Motor.C.setSpeed(90);
		Motor.C.forward();
		Delay.msDelay((long)duration);
		Motor.A.stop();
		Motor.C.stop();
		
		gyroSensor.reset();
		while (carOrientation != -90.0) { 
			Motor.A.setSpeed(100);
			Motor.A.forward();
			Motor.C.setSpeed(100);
			Motor.C.backward();
			
			angleMode.fetchSample(angleSample, 0); // Gyro sensing
			carOrientation = angleSample[angleMode.sampleSize() - 1];
		}
		Motor.A.stop();
		Motor.C.stop();
		
	}
	static public void repeatedRotation2(){ //measure error in x,y, angle
		SampleProvider angleMode = gyroSensor.getAngleMode();
		float[] angleSample = new float[angleMode.sampleSize()];
		
		double duration =  0.01 /(2.0*Math.PI*wheel_radius/4.0) * 1000.0; //duration to move 50cm
		Motor.A.setSpeed(90); //  degrees per second
		Motor.A.forward();
		Motor.C.setSpeed(90);
		Motor.C.forward();
		Delay.msDelay((long)duration);
		Motor.C.setSpeed(0);
		Motor.A.setSpeed(0);
		
		gyroSensor.reset();
		while (carOrientation != -90.0) { 
			Motor.A.setSpeed(100);
			Motor.A.forward();
			Motor.C.setSpeed(100);
			Motor.C.backward();			
			angleMode.fetchSample(angleSample, 0); // Gyro sensing
			carOrientation = angleSample[angleMode.sampleSize() - 1];
		}
		Motor.A.stop();
		Motor.C.stop();
		
		duration =  0.05 /(2.0*Math.PI*wheel_radius/4.0) * 1000.0; //duration to move 50cm
		Motor.A.setSpeed(90); //  degrees per second
		Motor.A.forward();
		Motor.C.setSpeed(90);
		Motor.C.forward();
		Delay.msDelay((long)duration);
		Motor.A.stop();
		Motor.C.stop();
		
		gyroSensor.reset();
		while (carOrientation != 135.0) { 
			Motor.A.setSpeed(100);
			Motor.A.backward();
			Motor.C.setSpeed(100);
			Motor.C.forward();		
			angleMode.fetchSample(angleSample, 0); // Gyro sensing
			carOrientation = angleSample[angleMode.sampleSize() - 1];
		}
		Motor.A.stop();
		Motor.C.stop();
		
		duration =  0.05 /(2.0*Math.PI*wheel_radius/4.0) * 1000.0; //duration to move 50cm
		Motor.A.setSpeed(90); //  degrees per second
		Motor.A.forward();
		Motor.C.setSpeed(90);
		Motor.C.forward();
		Delay.msDelay((long)duration);	
		Motor.A.stop();
		Motor.C.stop();
		
	}
	
}
