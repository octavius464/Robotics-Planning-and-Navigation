import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;

public class Sensor {
	public static void main(String args[]){
		/*EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
		SensorMode reflectedLightMode = colorSensor.getRedMode();
		float[] reflectedLightSample = new float[reflectedLightMode.sampleSize()];
		
	    while(true){
			reflectedLightMode.fetchSample(reflectedLightSample, 0);	
		    System.out.println(reflectedLightSample[reflectedLightMode.sampleSize()-1]);
		    try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}	
	    } */
		
		EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
		ultrasonicSensor.enable();
		
		SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];
	
		while(true){
			distanceMode.fetchSample(distanceSample, 0);
			System.out.println("distance: " + distanceSample[distanceMode.sampleSize()-1]);
			try{
				Thread.sleep(500);
			} catch (InterruptedException e){
				e.printStackTrace();
			}
			
			/*while ( < float(0.03) &&  > float(0.02)){
				Motor.A.setSpeed(50);
				Motor.A.forward();
				Motor.C.setSpeed(10);
				Motor.C.forward();
				
			}
			
			if( < 0.01){
				
			}*/
		}
		// 
			
		
		
		
	}
	
}
