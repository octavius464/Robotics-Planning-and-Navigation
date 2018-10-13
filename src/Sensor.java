import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;

public class Sensor {
	public static void main(String args[]){
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
		while(true){			
			System.out.println(colorSensor.getColorID());
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		
		}
		
		
	}
	
}
