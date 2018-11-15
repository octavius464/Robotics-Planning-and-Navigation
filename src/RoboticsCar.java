import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

//Motor functions: forward(), stop(), rotateTo(),rotate(), setSpeed()
public class RoboticsCar {
	public static void main(String args[]){
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
		SensorMode reflectedLightMode = colorSensor.getRedMode();
		float[] reflectedLightSample = new float[reflectedLightMode.sampleSize()];
		
		float white =(float) 0.66;
		float black = (float )0.08;
		float midPointValue = (white-black)/2 + black;
		
		// naive line following, without P or PID control
	    /*while(true){
			reflectedLightMode.fetchSample(reflectedLightSample, 0);	
			float measuredValue = reflectedLightSample[reflectedLightMode.sampleSize()-1];
			System.out.println(measuredValue);
		    
			if (measuredValue < midPointValue){
				Motor.A.setSpeed(100);   // 720 : 2 RPM
				Motor.A.forward();
				Motor.C.setSpeed(50);
				Motor.C.forward();
			} else{
				Motor.A.setSpeed(50);
				Motor.A.forward();
				Motor.C.setSpeed(100);
				Motor.C.forward();
			}    
	    }*/
	    
	    //Proportional Control
	    /*float Kp = 300;
	    while(true){
			reflectedLightMode.fetchSample(reflectedLightSample, 0);	
			float measuredValue = reflectedLightSample[reflectedLightMode.sampleSize()-1];
		    
			float correction = Kp *(midPointValue-measuredValue);
			// Turn each motor according to the correction
			Motor.A.setSpeed(100+correction); 
			Motor.A.forward();
			Motor.C.setSpeed(100-correction);
			Motor.C.forward();
		    // If this doesn't work, can try rotate,			
	    }*/
		
		//PID Control
	    float Kp = 360;
	    float Ki = 0;
	    float Kd = 0;
	    float lastError = 0;
	    float integral = 0;
	    while(true){
			reflectedLightMode.fetchSample(reflectedLightSample, 0);	
			float measuredValue = reflectedLightSample[reflectedLightMode.sampleSize()-1];
		    float error = midPointValue -measuredValue;
		    
		    integral +=error;
		    if(error > -0.02 &&error < 0.02){
		    	
		    	integral = 0;
		    }
		    float derivative = error - lastError;
		    lastError = error;
		    
			float correction = Kp *error + Ki*integral + Kd*derivative;
			// Turn each motor according to the correction
			Motor.A.setSpeed(150+correction); 
			Motor.A.forward();
			Motor.C.setSpeed(150-correction);
			Motor.C.forward();
			try{
				Thread.sleep(15);
			} catch (InterruptedException e){
				e.printStackTrace();
			}
		    // If this doesn't work, can try rotate,			
	    }
	    
	}
}
