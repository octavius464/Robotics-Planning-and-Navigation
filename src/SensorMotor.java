import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
//1. Rotating very slowly   2. need to change direction of rotation intelligently  
//3. one direction, then next iteration opposite direction but in larger angle so that
//it is trying to reach the black line to a greater extent
public class SensorMotor {
	public static void main(String args[]){
		int RotateAngle = 0;
		int increment = 5;
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
		while(true){			
			int color = colorSensor.getColorID(); //white=6,black=7
			System.out.println(colorSensor.getColorID());
			if(color == 7){
				Motor.A.setSpeed(50);// 2 RPM
				Motor.C.setSpeed(50);
				Motor.A.forward();
				Motor.C.forward();
			}
			else if(color != 7){				
				while(color != 7){
					color = colorSensor.getColorID();
					RotateAngle += increment;
					RotateAngle = RotateAngle%360;
					Motor.A.rotateTo(RotateAngle);
				//	Motor.C.rotateTo(-RotateAngle);
					if(color == 7){
						break;
					}
				}
				RotateAngle = 0;
				
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}	
		}
		
		}

}
