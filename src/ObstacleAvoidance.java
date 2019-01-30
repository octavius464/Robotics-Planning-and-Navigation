import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;

//For testing the obstacle avoidance part
public class ObstacleAvoidance {
	public static void main(String args[]){
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);  // Set up Color Sensor
		SensorMode reflectedLightMode = colorSensor.getRedMode();
		float[] reflectedLightSample = new float[reflectedLightMode.sampleSize()];
		
		EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);  // Set up Ultrasonic Sensor
		ultrasonicSensor.enable();
		SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];
		
		EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S1);
		gyroSensor.reset();
		SampleProvider angleMode = gyroSensor.getAngleMode();
		float[] angleSample = new float[angleMode.sampleSize()];
		
		float white =(float)0.66;
		float black = (float)0.08;
		float midPointValue = (white-black)/2 + black;
	    boolean hasObstacle = false;
	    float minObstDis = (float) 0.130 ; // This is the distance when the car starts avoiding obstacle (Need to find out what value)
	    float defaultObstDis = (float) 0.400; // The measurement when there is no obstacle in front
	    
	    while(true){
	    	if (!hasObstacle){              			
				Motor.A.setSpeed(150); 
				Motor.A.forward();
				Motor.C.setSpeed(150);
				Motor.C.forward();
	    	}
	    	
	    	distanceMode.fetchSample(distanceSample, 0);   // Obstacle detection
	    	float obstacleDistance = distanceSample[distanceMode.sampleSize()-1];
	    	if( obstacleDistance <= minObstDis){
	    		hasObstacle = true;
	    	}
	    	
	    	// Algorithm for obstacle avoidance
	    	if(hasObstacle){             
	    		Motor.A.stop();
				Motor.C.stop();
				gyroSensor.reset();
				
				//rotate the car to 45 degrees
		    	while(true){ 
		    		angleMode.fetchSample(angleSample, 0);   // Gyro sensing
			    	float rotatedOrientation = angleSample[angleMode.sampleSize()-1];
			    	System.out.println(rotatedOrientation);
			    	if( rotatedOrientation < -60.1){ // for rotating to 45 degrees
			    		Motor.A.stop();
			    		Motor.C.stop();
						break;
			    	}else{
			    		Motor.A.setSpeed(100);
			    		Motor.A.forward();
			    		Motor.C.setSpeed(100);
			    		Motor.C.backward();	
			    	}
		    	}
		    	
		    	//rotate ultrasonic sensor to face the obstacle
		    	int angleToRotateForUS = -40;
		    	Motor.B.rotateTo(-40);
		    	while(true){ 
		    		distanceMode.fetchSample(distanceSample, 0);   // Obstacle detection
			    	float disToObstacle = distanceSample[distanceMode.sampleSize()-1];
			    	if(disToObstacle >minObstDis-0.03 && disToObstacle < minObstDis+0.03){
			    		break;
			    		
			    	}else{
			    		angleToRotateForUS=angleToRotateForUS - 5;
				    	Motor.B.rotateTo(angleToRotateForUS);
			    	}
		    	}
		    	System.out.println("Done looking");
		    	// Circle around the obstacle
		    	float Kp_obstacle = 410; 

		    	while(true){
		    		distanceMode.fetchSample(distanceSample, 0);   // Ultrasonic sensing
			    	float disToObstacle = distanceSample[distanceMode.sampleSize()-1];
			    	float error_to_obstacle = minObstDis - disToObstacle;
			    	if(error_to_obstacle>0 && error_to_obstacle > 0.05){
			    		error_to_obstacle = (float)0.05;
			    	}
			    	if(error_to_obstacle<0 && error_to_obstacle < -0.05){
			    		error_to_obstacle = (float)-0.05;
			    	}
			    	float rotate_correction = Kp_obstacle * error_to_obstacle;
			    	//if(50 - rotate_correction > 0){
				    Motor.A.setSpeed(100 + rotate_correction); 
					Motor.A.forward();
			    	/*}else{
			    		Motor.A.setSpeed(0); 
						Motor.A.forward();
			    	}*/
			    	//if(50 + rotate_correction > 0){
					Motor.C.setSpeed(100 - rotate_correction);
					Motor.C.forward();
			    	/*}else{
			    		Motor.C.setSpeed(0);
						Motor.C.forward();
			    	}*/
					
					angleMode.fetchSample(angleSample, 0);   // Gyro sensing
			    	float rotatedOrientation = angleSample[angleMode.sampleSize()-1];
			    	if(rotatedOrientation > 359){
			    		break;
			    	}	    	
			    	/*try{
						Thread.sleep(15);
					} catch (InterruptedException e){
						e.printStackTrace();
					}	*/
		    	}
		    	System.out.println("Done circling");
		    	
		    	
		    	
		    	
		    	System.out.println("stopped");
		    	try{
					Thread.sleep(1500000);
				} catch (InterruptedException e){
					e.printStackTrace();
				}		    		
	    	} // end obstacle avoidance 
	    
	    
			
			try{
				Thread.sleep(15);
			} catch (InterruptedException e){
				e.printStackTrace();
			}			
	}
	    
	}
}

