import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class RoboticsCar {
	static boolean hasObstacle = false;
	static float carOrientation;

	public static void main(String args[]) {

		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4); // Set
																		// up
																		// Color
																		// Sensor
		SensorMode reflectedLightMode = colorSensor.getRedMode();
		float[] reflectedLightSample = new float[reflectedLightMode.sampleSize()];

		EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2); // Set
																						// up
																						// Ultrasonic
																						// Sensor
		ultrasonicSensor.enable();
		SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];

		EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S1);
		gyroSensor.reset();
		SampleProvider angleMode = gyroSensor.getAngleMode();
		float[] angleSample = new float[angleMode.sampleSize()];
		angleMode.fetchSample(angleSample, 0); // Gyro sensing
		carOrientation = angleSample[angleMode.sampleSize() - 1];

		Motor.B.setSpeed(450); //300
		MotorBThread motorBThread = new MotorBThread();
		motorBThread.start();

		boolean canStart = false;
		while (true) {
			int colorID = colorSensor.getColorID();
			if (colorID == 0) {
				canStart = false;
			} else {
				canStart = true;
				break;
			}
		}

		float white = (float) 0.66;
		float black = (float) 0.08;
		float midPointValue = (white - black) / 2 + black;

		float minObstDis = (float) 0.125; // 0.120 ; // This is the distance
											// when the car starts avoiding
											// obstacle (Need to find out what
											// value)
		float defaultObstDis = (float) 0.400; // The measurement when there is
												// no obstacle in front

		// PI Control
		float Kp = 320; // 345 330 BETTER
		float Ki = (float) 0.0;
		float Kd = (float) 40; // 0.01
		float lastError = 0;
		float integral = 0;
		int samplingRate = 12;
		main: while (canStart) {
			if (!hasObstacle) { // Algorithm for line following
				reflectedLightMode.fetchSample(reflectedLightSample, 0);
				float measuredValue = reflectedLightSample[reflectedLightMode.sampleSize() - 1];
				float error = midPointValue - measuredValue;
				integral = 2 / 3 * integral + error;
				if (error > -0.02 && error < 0.02) { // Wind down integral term
					integral = 0;
				}
				float derivative = (error - lastError) / samplingRate;
				lastError = error;
				float correction = Kp * error + Ki * integral + Kd * derivative;

				// Differential drive according to correction
				Motor.A.setSpeed(155 + correction);
				Motor.A.forward();
				Motor.C.setSpeed(155 - correction);
				Motor.C.forward();

				if (measuredValue > 0.55) {
					while (true) {

						if (colorSensor.getColorID() == 0) {
							Motor.A.stop();
							Motor.C.stop();
							continue;
						} else {
							break;
						}
					}
				}

			} // end of line following

			distanceMode.fetchSample(distanceSample, 0); // Obstacle detection
			float obstacleDistance = distanceSample[distanceMode.sampleSize() - 1];
			if (obstacleDistance <= minObstDis) {
				hasObstacle = true;
			}

			if (hasObstacle) {
				Motor.A.stop();
				Motor.C.stop();
				gyroSensor.reset();

				// rotate the car to 45 degrees
				while (true) {
					angleMode.fetchSample(angleSample, 0); // Gyro sensing
					carOrientation = angleSample[angleMode.sampleSize() - 1];

					if (carOrientation < -90.1) { // testing 4: for rotating to //-70.1
													// 60 degrees, -60.1+
													// offsetAngleToRotate
						// is it quick enough to get the value, and find out
						// which direction
						Motor.A.stop();
						Motor.C.stop();
						break;
					} else {
						Motor.A.setSpeed(100);
						Motor.A.forward();
						Motor.C.setSpeed(100);
						Motor.C.backward();
					}
				}

				// Circle around the obstacle
				float Kp_obstacle = 430; // 430 good one
				obstAvoidloop: while (true) {
					distanceMode.fetchSample(distanceSample, 0); // Ultrasonic
																	// sensing
					float disToObstacle = distanceSample[distanceMode.sampleSize() - 1];
					float error_to_obstacle = minObstDis - disToObstacle;
					if (error_to_obstacle > 0 && error_to_obstacle > 0.05) {
						error_to_obstacle = (float) 0.05;
					}
					if (error_to_obstacle < 0 && error_to_obstacle < -0.05) {
						error_to_obstacle = (float) -0.05;
					}
					float rotate_correction = Kp_obstacle * error_to_obstacle;
					Motor.A.setSpeed(100 + rotate_correction);
					Motor.A.forward();
					Motor.C.setSpeed(100 - rotate_correction);
					Motor.C.forward();

					reflectedLightMode.fetchSample(reflectedLightSample, 0);
					float measuredValue = reflectedLightSample[reflectedLightMode.sampleSize() - 1];
					if (measuredValue < white - 0.05) {
						hasObstacle = false;
						// move forward to find white again
						while (measuredValue < white - 0.05) {
							Motor.A.setSpeed(250); //200 good
							Motor.A.forward();
							Motor.C.setSpeed(150);
							Motor.C.forward();
							reflectedLightMode.fetchSample(reflectedLightSample, 0);
							measuredValue = reflectedLightSample[reflectedLightMode.sampleSize() - 1];
						}
						Delay.msDelay(500);
						Motor.A.stop();
						Motor.C.stop();
						// rotate and find the black line
						findblacklineloop: while (true) {
							if (measuredValue < white - 0.05) {
								break findblacklineloop;
							} else {
								Motor.A.setSpeed(100); // 100
								Motor.A.forward();
								Motor.C.setSpeed(100);
								Motor.C.backward();
								reflectedLightMode.fetchSample(reflectedLightSample, 0);
								measuredValue = reflectedLightSample[reflectedLightMode.sampleSize() - 1];
							}
						}

						// rotate until it finds the white
						while (measuredValue < white - 0.01) {
							Motor.A.setSpeed(100);
							Motor.A.forward();
							Motor.C.setSpeed(100);
							Motor.C.backward();
							reflectedLightMode.fetchSample(reflectedLightSample, 0);
							measuredValue = reflectedLightSample[reflectedLightMode.sampleSize() - 1];
						}
						// rotate back to find the black line
						while (measuredValue > white - 0.05) {
							Motor.A.setSpeed(50);
							Motor.A.backward();
							Motor.C.setSpeed(50);
							Motor.C.forward();
							reflectedLightMode.fetchSample(reflectedLightSample, 0);
							measuredValue = reflectedLightSample[reflectedLightMode.sampleSize() - 1];
						}

						break obstAvoidloop;
					}

					/*
					 * try{ Thread.sleep(15); } catch (InterruptedException e){
					 * e.printStackTrace(); }
					 */
				}

			} // end obstacle avoidance

			try {
				Thread.sleep(samplingRate);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

	}

	static class MotorBThread extends Thread {
		public void run() {
			try {
				float motorBrotation = 30;
				float motorBdirection = 1;
				while (true) { // testing
					while (!isInterrupted() && !hasObstacle) {
						if (motorBrotation >= 30 || motorBrotation <= -30) {
							motorBdirection *= -1;
						}
						// motorBrotation = motorBrotation*motorBdirection;
						motorBrotation = motorBrotation + motorBdirection * 20; //20
						Motor.B.rotateTo((int) motorBrotation);
					}
					// testing 4:offsetAngleToRotate = Motor.B.getLimitAngle();
					// testing 2: for getting Motor B back to aligned position
					// to car after avoiding obstacle
					Motor.B.rotateTo(-80); // -70 too much //65 seems good latest //70
					while (hasObstacle) {
						// rotate ultrasonic sensor while rotating around
						// obstacle?
					}
					Thread.sleep(3000);
					Motor.B.rotateTo((int) carOrientation);
				}
			} catch (InterruptedException e) {
				System.out.println(e);
			} catch (Exception e) {
				System.out.println(e.toString());
			}
		}
	}

}