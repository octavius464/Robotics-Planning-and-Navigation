import lejos.utility.Delay;


public class RobotController {
	
	public static void main(String args[]){
		
		Robot robot = new Robot();
		Localization robotLocalization = new Localization(37); //each array distance is 1.7cm, so total length is 37*1.7=62.9cm
		int localizedPos = -1;
		
		//Localization
		localizationLoop: while(true){
			int direction = 0; // 0 corresponds to moving forward, 1 corresponds to moving backward
			//move forward initially
			
			for(int i=0;i<18;++i){ //moving 1.7cm for 18 times corresponds to moving half the distance of the pattern
				//Sense and update probabilities
				int colorMeasurement = robot.getColorMeasurement();				
				robotLocalization.updateAfterSensing2(colorMeasurement);
						
				//Move robot and update probabilities
				robot.moveToLocalize(direction);			
				robotLocalization.updateAfterMoving(direction);
				
				if(robotLocalization.isDoneLocalizing()){
					localizedPos = robotLocalization.getLocalizedPosition();
					break localizationLoop;
				}
				
				direction = ((direction == 0) ? 1 : 0);
						
				Delay.msDelay(100);		//TODO: see if needed	
			}		
		}
		
		
	}
}


