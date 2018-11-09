import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;

public class Sensor {
	public static void main(String args[]){
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
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
	    }
		
		
	}
	
}
