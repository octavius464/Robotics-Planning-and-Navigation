import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;

public class RoboticsCar {
	public static void main(String args[]){
		Motor.A.setSpeed(720);// 2 RPM
		Motor.C.setSpeed(720);
		Motor.A.forward();
		Motor.C.forward();
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Motor.A.stop();
		Motor.C.stop();
		Motor.A.rotateTo( 360);
		Motor.A.rotate(-720,true);
	}
}
