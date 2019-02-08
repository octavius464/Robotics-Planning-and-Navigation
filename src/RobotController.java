import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import java.util.ArrayList;
import java.util.HashMap;


public class RobotController {
	
	public static void main(String args[]){
		
		Robot robot = new Robot();
		Localization robotLocalization = new Localization(74); //multiples of 37
		//each array distance is 1.7cm, so total length is 37*1.7=62.9cm
		
		int localizedPos = -1;
		int direction = 0; // 0:moving forward, 1:moving backward, move forward initially
		
		//Localization
		localizationLoop: while(true){		
			for(int i=0;i<25;++i){ //moving for around 20cm forward and backward 
				//Sense and update probabilities
				int colorMeasurement = robot.getColorMeasurement();		//blue return 1, white return 0		
				robotLocalization.updateAfterSensing2(colorMeasurement);
						
				//Move robot and update probabilities
				robot.moveToLocalize(direction);			
				robotLocalization.updateAfterMoving(direction);
				System.out.println(robotLocalization.getLocalizedPosition());
				if(robotLocalization.isDoneLocalizing()){
					localizedPos = robotLocalization.getLocalizedPosition();
					System.out.println("done localizing: "+ localizedPos);
					break localizationLoop;
				}
						
				Delay.msDelay(100);		//TODO: see if needed	
			}
			
			direction = ((direction == 0) ? 1 : 0);
		}
		
		Delay.msDelay(1000000);	
		
		//Path Planning
		
		//Navigate to goal
		
		/* TODO test robot
		//Enter into cave, touch wall, make a beep sound, detect color, navigate out back to goal position
		robot.rotateToEntrance();
		robot.moveToWallAndBeep();
		int colorAtWall = robot.getColorMeasurement();
		robot.moveBackToGoal();
		*/
		
	} //end of main
	
}



class Robot {

	static double WHEEL_RADIUS = 0.015;
	int localizedPos;
	EV3ColorSensor colorSensor;
	
	EV3GyroSensor gyroSensor;
	SampleProvider angleMode;
	float[] angleSample;
	float carOrientation;
	
	EV3TouchSensor touchSensor;
	SampleProvider touchMode;
	float[] touchSample;
	float measuredTouch;
	
	RegulatedMotor mA;
	RegulatedMotor mC;
	
	
	
	public Robot(){
		colorSensor = new EV3ColorSensor(SensorPort.S4);
		gyroSensor = new EV3GyroSensor(SensorPort.S2);
		touchSensor = new EV3TouchSensor(SensorPort.S1);
		
		gyroSensor.reset();
		angleMode = gyroSensor.getAngleMode();
		angleSample = new float[angleMode.sampleSize()];
		angleMode.fetchSample(angleSample, 0); // Orientation sensing
		carOrientation = angleSample[angleMode.sampleSize() - 1];
		
		touchMode = touchSensor.getTouchMode();
		touchSample = new float[touchMode.sampleSize()];
		touchMode.fetchSample(touchSample, 0);
		measuredTouch = touchSample[touchMode.sampleSize() - 1];
		
		mA = new EV3LargeRegulatedMotor(MotorPort.A);
		mC = new EV3LargeRegulatedMotor(MotorPort.C);
		mA.synchronizeWith(new RegulatedMotor[] { mC });		
	}
	
	/**
	 * Get the color measurement from the color sensor. If it is blue, then return 1, else return 0.
	 * @return an int corresponding to blue or white;
	 */
	public int getColorMeasurement(){
		int measurement = colorSensor.getColorID();
		if(measurement == 7){ //blue:7 , white:2
			measurement = 1;
		}else{
			measurement = 0;
		}
		return measurement;
	}
	
	/**
	 * Move 1.7cm in each iteration for the localization. 
	 * @param direction 0 for moving forward, 1 for moving backward
	 */
	public void moveToLocalize(int direction){
		double meterspersecond = 2.0*Math.PI*WHEEL_RADIUS/2.0;
		double duration = (0.0085 /meterspersecond) * 1000.0; //move forward 1.7cm/cell of the array each iteration

		mA.startSynchronization();
		mA.setSpeed(180);
		mC.setSpeed(180);
		if(direction == 0){
			mA.forward();
			mC.forward();
		}
		if(direction == 1){
			mA.backward();
			mC.backward();
		}
		mA.endSynchronization();

		Delay.msDelay((long)duration);
		mA.startSynchronization();
		mA.stop();
		mC.stop();
		mA.endSynchronization();
	}
	
	public void rotateToEntrance(){
		angleMode.fetchSample(angleSample, 0);
		carOrientation = angleSample[angleMode.sampleSize() - 1];
		mA.startSynchronization();
		mA.setSpeed(180);
		mC.setSpeed(180);
		while(!(carOrientation > -0.1) || !(carOrientation < 0.1)){
			mA.forward();
			mC.backward();
		}
		mA.endSynchronization();
		mA.startSynchronization();
		mA.stop();
		mC.stop();
		mA.endSynchronization();
	}
	
	public void moveToWallAndBeep(){
		touchMode.fetchSample(touchSample, 0);
		measuredTouch = touchSample[touchMode.sampleSize() - 1];
		
		mA.startSynchronization();
		mA.setSpeed(180);
		mC.setSpeed(180);
		while(measuredTouch != 1){
			mA.forward();
			mC.forward();
		}
		Sound.beep();
		mA.endSynchronization();
		
		mA.startSynchronization();
		mA.stop();
		mC.stop();
		mA.endSynchronization();
	}
	
	public void moveBackToGoal(){ //TODO measure distance of going back to goal position
		double meterspersecond = 2.0*Math.PI*WHEEL_RADIUS/2.0;
		double duration = (0.0085 /meterspersecond) * 1000.0; //move forward 1.7cm/cell of the array each iteration

		mA.startSynchronization();
		mA.setSpeed(180);
		mC.setSpeed(180);
		mA.backward();
		mC.backward();	
		mA.endSynchronization();

		Delay.msDelay((long)duration);
		mA.startSynchronization();
		mA.stop();
		mC.stop();
		mA.endSynchronization();
	}
	
}



class Localization {
	
	float[] probabilityDistribution;
	int size;
	int[] grid;
	float SensorWorkProb = (float) 0.9; //TODO
	float PROB_MOVE_FORWARD = (float) 0.95; //TODO
	float PROB_MOVE_BACKWARD = (float) 0.95; //TODO
	float LOCALIZATION_THRESHOLD = (float) 0.5;
	
	public Localization(int size){
		this.size = size;
		probabilityDistribution = new float[this.size];
		grid = new int[this.size];
		initializeProb();
		initializeGrid();
	}
	
	/**
	 * To initialize the grid (sensor model) to store whether each index corresponds to having a blue
	 * or white color. This refers to the line pattern.
	 */
	public void initializeGrid(){ 
		// Pattern: 3.4cmWhite, 5.1cmBlue, 1.7W, 3.4B, 3.4W, 5.1B, 1.7W, 3.4B, 3.4W, 5.1B, 3.4W, 5.1B, 1.7W, 3.4B, 3.4W, 5.1B, 1.7W, 3.4B
		int scalingFactor = size/37;
		int[] pattern = {0,2,5,6,8,10,13,14,16,18,21,23,26,27,29,31,34,35,37};
		for(int i=0;i<pattern.length;++i){
			pattern[i] = pattern[i]*scalingFactor;
		}
		
		int color = 0; //0:white , 1:blue
		for(int n=0;n<pattern.length-1;++n){	
			for(int i=pattern[n];i<pattern[n+1];++i){
				grid[i] = color;
			}
			color = ((color == 0) ? 1 : 0);
		}    
	}
	
	/**
	 * To initialize the probability distribution by having each index having the same probability
	 * in the beginning.
	 */
	public void initializeProb(){
		float initialProb =(float) 1.0/size;
		for(int i=0; i<size; ++i){
			probabilityDistribution[i] = initialProb;
		}		
	}
	
	public void updateAfterSensing(float[] observationLikelihood){
		float normalizationFactor = 0;
		for(int i=0; i<size; ++i){
			probabilityDistribution[i] = observationLikelihood[i] * probabilityDistribution[i];
			normalizationFactor = normalizationFactor + probabilityDistribution[i];			
		}
		for(int i=0; i<size; ++i){
			probabilityDistribution[i] = probabilityDistribution[i]/normalizationFactor;		
		}
	}
	
	/**
	 * Update the probability distribution after making an observation using robot sensors.
	 * @param observation
	 */
	public void updateAfterSensing2(int observation){
		float probSum = 0;
		for(int i=0; i<size; ++i){
			if(observation == grid[i]){
				probabilityDistribution[i] = probabilityDistribution[i] * SensorWorkProb;
			}else{
				probabilityDistribution[i] = probabilityDistribution[i] * (1-SensorWorkProb);
			}
			probSum = probSum + probabilityDistribution[i];
		}
		for(int i=0; i<size; ++i){
			probabilityDistribution[i] = probabilityDistribution[i]/probSum;
		}
	}
	
	/**
	 * Update probability distribution after an action which is moving in this case.
	 * @param direction
	 */
	public void updateAfterMoving(int direction){ 
		if(direction == 0){ //moving forward
			for(int i=1; i<size; ++i){		
				probabilityDistribution[i] =(float) (probabilityDistribution[i-1]*PROB_MOVE_FORWARD + probabilityDistribution[i]*(1-PROB_MOVE_FORWARD));
			}
		}	
		if(direction == 1){ //moving backward
			for(int i=0; i<(size-1); ++i){		
				probabilityDistribution[i] =(float) (probabilityDistribution[i+1]*PROB_MOVE_BACKWARD + probabilityDistribution[i]*(1-PROB_MOVE_BACKWARD));
			}
		}	
	}
	
	/**
	 * Check whether the robot is done localizing by checking whether any value of the array
	 * is above the localization threshold.
	 * @return
	 */
	public boolean isDoneLocalizing(){
		for(int i=0; i<size; ++i){
			if(probabilityDistribution[i] > LOCALIZATION_THRESHOLD){
				return true;
			}
		}
		return false;
	}
	
	/**
	 * @return the index of the array lineMap with the highest probability. This corresponds to 
	 * the localized position of the robot.  
	 */
	public int getLocalizedPosition(){
		float max_prob = -1;
		int mostLikelyPosition = 0;
		for(int i=0; i<size; ++i){
			if(probabilityDistribution[i] > max_prob){
				max_prob = probabilityDistribution[i];
				mostLikelyPosition = i;
			}
		}
		return mostLikelyPosition;
	}
	
	/**
	 * for testing. Tested, initialized grid properly.
	 */
	public void printGrid(){
		for(int i=0;i<size;++i){
			System.out.print(grid[i]);
		}
	}
	
}


class AStarPlanner{
	
	int[] currentNode = new int[2];
	ArrayList<int[]> path;
	int[] startingNode;
	int[] goalNode;
	int[][] map;
	ArrayList<int[]> openList;
	ArrayList<int[]> closedList;
	HashMap<int[],int[]> mapNodeToParent = new HashMap<>();
	HashMap<int[], ArrayList<int[]>> mapNodeToNeighbours = new HashMap<>();
	HashMap<int[], float[]> mapNodeToGnHnFn = new HashMap<>();
	
	public AStarPlanner(int[] startingNode, int[] goalNode, int[][] map2DGrid){
		this.startingNode = startingNode;
		this.goalNode = goalNode;
		map = map2DGrid;
		openList = new ArrayList<>();
		closedList = new ArrayList<>();
		generateNeighbours();
		mapNodeToGnHnFn.put(startingNode, new float[]{0f,calculateHn(startingNode),calculateHn(startingNode)});
		plan();
	}
	
	private void generateNeighbours(){ //TODO test code
		int maxX = map[0].length; 
		int maxY = map.length;
		for(int y=0; y < maxY; ++y){
			for(int x=0; x < maxX; ++x){
				ArrayList<int[]> neighbours = new ArrayList<>();
				if(y+1 < maxY){
					if(map[x][y+1] != 1){
						neighbours.add(new int[]{x,y+1});
					}
				}
				if(y-1 >= 0){
					if(map[x][y-1] != 1){
						neighbours.add(new int[]{x,y-1});
					}
				}
				if(x+1 < maxX){
					if(map[x+1][y] != 1){
						neighbours.add(new int[]{x+1,y});
					}
				}
				if(x-1 >= 0){
					if(map[x-1][y] != 1){
						neighbours.add(new int[]{x-1,y});
					}
				}
				mapNodeToNeighbours.put(new int[]{x,y}, neighbours);
			}
		}
	}
	
	public ArrayList<int[]> getPath(){
		return path;
	}
	
	private void plan(){
		currentNode = startingNode;
		closedList.add(startingNode);
		do{
			for(int[] neighbour : mapNodeToNeighbours.get(currentNode)){
				float fn = calculateFn(currentNode,neighbour);
				if(openList.contains(neighbour)){
					if(mapNodeToGnHnFn.get(neighbour)[2] > fn){
						setNodeGnHnFn(neighbour,calculateGnHnFn(currentNode, neighbour));
						setNodeParent(neighbour,currentNode);
					}
				}else{		
					openList.add(neighbour);  //TODO only add if not in closed list as well?
					setNodeGnHnFn(neighbour,calculateGnHnFn(currentNode, neighbour));
					setNodeParent(neighbour,currentNode);
				}
				int[] nodeWithLowestFn = getNodeWithLowestFn();
				currentNode = nodeWithLowestFn;
				openList.remove(nodeWithLowestFn);
				closedList.add(nodeWithLowestFn);			
			}	
		}while(!openList.isEmpty() || !closedList.contains(goalNode));
		
		if(closedList.contains(goalNode)){
			path.add(goalNode);
			path = generatePath(path, goalNode);
		}		
	}
	
	private float calculateFn(int[] parent, int[] child){
		float parentGn = mapNodeToGnHnFn.get(parent)[0];
		float childGn = parentGn + 1;
		float childHn = calculateHn(child);
		float childFn = childGn + childHn;
		return childFn;
	}
	
	private float[] calculateGnHnFn(int[] parent, int[] child){
		float parentGn = mapNodeToGnHnFn.get(parent)[0];
		float childGn = parentGn + 1;
		float childHn = calculateHn(child);
		float childFn = childGn + childHn;
		float[] newScores = {childGn,childHn,childFn};
		return newScores;
	}
	
	private void setNodeGnHnFn(int[] node, float[] scores){
		mapNodeToGnHnFn.put(node, scores);
	}
	
	private float calculateHn(int[] node){ //TODO
		float heuristic = 0;
		return heuristic;
	}
	
	private float getNodeFn(int[] node){
		return mapNodeToGnHnFn.get(node)[2];
	}
	
	
    private void setNodeParent(int[] newNode, int[] parentNode){
    	mapNodeToParent.put(newNode, parentNode);
    }
    
    private int[] getNodeWithLowestFn(){
    	float lowestScore = 10000f;
    	int[] nodeWithLowestScore = openList.get(0);
    	for(int[] eachNode : openList){
    		float eachNodeFn = mapNodeToGnHnFn.get(eachNode)[2];
    		if(eachNodeFn < lowestScore){
    			lowestScore = eachNodeFn;
    			nodeWithLowestScore = eachNode;
    		}
    	}
    	return nodeWithLowestScore;
    }
    
    /**
     * Return a path corresponding to the sequence of nodes that the robot should travel
     * to reach to goal from starting point, e.g. [startingPoint,[1,2],[2,2],[2,3],goalPoint]
     */
    private ArrayList<int[]> generatePath(ArrayList<int[]> path,int[] nextNode){
    	int[] parentNode = mapNodeToParent.get(nextNode);
    	if(parentNode.equals(startingNode)){
    		path.add(0,parentNode);
    		return path;
    	}else{		
    		path.add(0,parentNode);
    		return generatePath(path,parentNode);
    	}
    }
    
 }














