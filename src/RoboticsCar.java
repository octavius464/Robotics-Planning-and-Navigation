import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;

//Motor functions: forward(), stop(), rotateTo(),rotate(), setSpeed()
public class RoboticsCar {
	public static void main(String args[]){
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);  // Set up Color Sensor
		SensorMode reflectedLightMode = colorSensor.getRedMode();
		float[] reflectedLightSample = new float[reflectedLightMode.sampleSize()];
		
		EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);  // Set up Ultrasonic Sensor
		ultrasonicSensor.enable();
		SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];
		
		float white =(float)0.66;
		float black = (float)0.08;
		float midPointValue = (white-black)/2 + black;
	    boolean hasObstacle = false;
	    float minObstDis = 100 ; // This is the distance when the car starts avoiding obstacle (Need to find out what value)
		
		//PI Control
	    float Kp = 360;
	    float Ki = (float) 0.001;
	    float Kd = 0;
	    float lastError = 0;
	    float integral = 0;
	    while(true){
	    	if (!hasObstacle){               // Algorithm for line following
				reflectedLightMode.fetchSample(reflectedLightSample, 0);	
				float measuredValue = reflectedLightSample[reflectedLightMode.sampleSize()-1];
			    float error = midPointValue - measuredValue;	    
			    integral +=error;
			    if(error > -0.02 &&error < 0.02){ // Wind down integral term 	    	
			    	integral = 0;
			    }
			    float derivative = error - lastError;
			    lastError = error;		    
				float correction = Kp *error + Ki*integral + Kd*derivative;
				
				// Differential drive according to correction
				Motor.A.setSpeed(150+correction); 
				Motor.A.forward();
				Motor.C.setSpeed(150-correction);
				Motor.C.forward();
	    	}
	    	
	    	distanceMode.fetchSample(distanceSample, 0);   // Obstacle detection
	    	float obstacleDistance = distanceSample[distanceMode.sampleSize()-1];
	    	if( obstacleDistance <= minObstDis){
	    		hasObstacle = true;
	    	}
	    	
	    	if(hasObstacle){             // Algorithm for obstacle avoidance
	    		
	    		
	    		
	    		
	    		
	    	}
			
			try{
				Thread.sleep(15);
			} catch (InterruptedException e){
				e.printStackTrace();
			}			
	    }
	    
	}
}
