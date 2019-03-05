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
import java.util.Iterator;


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
				int colorObservation = robot.getColorObservation();		//blue return 1, white return 0		
				robotLocalization.updateAfterSensing2(colorObservation);
						
				//Move robot and update probabilities
				robot.moveToLocalize(direction);		//0:move forward, 1:move backward	
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
		 
		int[] startingNode = robot.mapLocToNode(localizedPos);
		
		//Path Planning
		int initialMapIndex = 2;
		
		MapGrid mapGrid = new MapGrid(initialMapIndex, new int[]{12,3} ,new int[]{1,8});
		System.out.println("Done initializing map");
		AStarPlanner planner = new AStarPlanner(mapGrid.getStartingNode(), mapGrid.getGoalNode(), mapGrid.getMap());
		System.out.println("Path planned");
		ArrayList<int[]> path = planner.getPath();
		ArrayList<int[]> wayPoints = planner.convertPathToWaypoints(path); 
		System.out.println("Start navigating...");
		//Navigate to goal
		robot.navigateToGoal(wayPoints,mapGrid.getCellSize());
		System.out.println("Done navigating...");
	
		
		
		//Enter into cave, touch wall, make a beep sound, detect color, navigate out back to goal position
		
	    robot.rotateToGoal(new int[]{path.get(path.size()-1)[0]-path.get(path.size()-2)[0], path.get(path.size()-1)[1]-path.get(path.size()-2)[1]} );		
		robot.moveToWallAndBeep();
		int colorAtWall = robot.getColorID();
		int mapIndex = robot.getMapIndex(colorAtWall);
		robot.moveBackToGoal();
		

		
		//Navigate to starting point
		
		MapGrid mapGridToStart = new MapGrid(mapIndex, new int[]{1,8} ,new int[]{14,3});
		System.out.println("Done initializing map");
		AStarPlanner plannerToStart = new AStarPlanner(mapGridToStart.getStartingNode(), mapGridToStart.getGoalNode(), mapGridToStart.getMap());
		System.out.println("Path planned");
		ArrayList<int[]> pathToStart = plannerToStart.getPath();
		ArrayList<int[]> wayPointsToStart = plannerToStart.convertPathToWaypoints(pathToStart); 
		System.out.println("Start navigating...");
		robot.navigateToStart(wayPointsToStart,mapGridToStart.getCellSize());
		System.out.println("Done navigating...");
		
		
	} //end of main
	
}



class Robot {

	static double WHEEL_RADIUS = 0.015;
	int localizedPos;
	float angleCorrection = 0.93f;
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
	public int getColorObservation(){
		int measurement = colorSensor.getColorID();
		if(measurement == 7){ //blue:7 , white:2
			measurement = 1;
		}else{
			measurement = 0;
		}
		return measurement;
	}
	
	public int getColorID(){ 
		return colorSensor.getColorID();
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
	
	public int[] mapLocToNode(int input){
	  if( input >= 0.0 && input < 14.8){
		   	 return new int[]{13, 2};
		    }else if (input >= 14.8 && input < 29.6) {
		   	 return new int[]{12, 3};
		    }else if (input >= 29.6 && input < 44.4) {
		   	 return new int[]{11, 4};
		    }else if (input >= 44.4 && input < 59.2) {
		   	 return new int[]{10, 5};
		    }else {
		   	 return new int[]{9, 6};
		    }
	}
	
	public void rotateToEntrance(){
		angleMode.fetchSample(angleSample, 0);
		carOrientation = angleSample[angleMode.sampleSize() - 1];
		//mA.startSynchronization();
		mA.setSpeed(180);
		mC.setSpeed(180);
		while(!(carOrientation > -46.9) || !(carOrientation < -45.1)){
			mA.forward();
			mC.backward();
			angleMode.fetchSample(angleSample, 0);
			carOrientation = angleSample[angleMode.sampleSize() - 1];
			System.out.println(carOrientation);
		}
		//mA.endSynchronization();
		mA.startSynchronization();
		mA.stop();
		mC.stop();
		mA.endSynchronization();
	}
	
	public void moveToWallAndBeep(){
		touchMode.fetchSample(touchSample, 0);
		measuredTouch = touchSample[touchMode.sampleSize() - 1];
		
		//mA.startSynchronization();
		mA.setSpeed(180);
		mC.setSpeed(180);
		while(measuredTouch != 1){
			mA.forward();
			mC.forward();
			touchMode.fetchSample(touchSample, 0);
			measuredTouch = touchSample[touchMode.sampleSize() - 1];
		}
		Sound.beep();
		//mA.endSynchronization();
		
		mA.startSynchronization();
		mA.stop();
		mC.stop();
		mA.endSynchronization();
	}
	
	public void moveBackToGoal(){ //TODO measure distance of going back to goal position
		double meterspersecond = 2.0*Math.PI*WHEEL_RADIUS/2.0;
		double duration = (0.23 /meterspersecond) * 1000.0; //move forward 1.7cm/cell of the array each iteration

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
	
	public float getCarOrientation(){
		angleMode.fetchSample(angleSample, 0);
		carOrientation = angleSample[angleMode.sampleSize() - 1];
		return carOrientation;
	}
	
	public void navigate(ArrayList<int[]> path, float cellSize, int[] previousVector){
		mA.setSpeed(180);
		mC.setSpeed(180);
		for(int i = 1; i < path.size(); ++i){
			gyroSensor.reset();
			carOrientation = 0; 
			System.out.println(carOrientation); 
			int[] newVector = new int[]{path.get(i)[0]-path.get(i-1)[0],path.get(i)[1]-path.get(i-1)[1]};
			float crossProduct = previousVector[0]*newVector[1]-previousVector[1]*newVector[0];
			float dotProduct = previousVector[0]*newVector[0]+previousVector[1]*newVector[1];
			float angleToRotate = (float) ((float) Math.atan2(crossProduct,dotProduct) * (180/Math.PI));
			angleToRotate = (float) (angleToRotate*angleCorrection);
			double distanceToTravel = Math.sqrt( Math.pow(path.get(i)[0]-path.get(i-1)[0],2) + Math.pow(path.get(i)[1]-path.get(i-1)[1],2) ) * cellSize; //in meters
		    
			previousVector = newVector;
			double meterspersecond = 2.0*Math.PI*WHEEL_RADIUS/2.0;
			double duration = (distanceToTravel /meterspersecond) * 1000.0;
			if(angleToRotate>0){
				while(carOrientation < angleToRotate){ 
					mA.backward();
					mC.forward();
					angleMode.fetchSample(angleSample, 0);
					carOrientation = angleSample[angleMode.sampleSize() - 1];
					System.out.println(carOrientation);
				}
			}else{
				while(carOrientation > angleToRotate){
					mA.forward();
					mC.backward();
					angleMode.fetchSample(angleSample, 0);
					carOrientation = angleSample[angleMode.sampleSize() - 1];
					System.out.println(carOrientation);
				}
			}
			
			mA.startSynchronization();
			mA.stop();
			mC.stop();
			mA.endSynchronization();
		
			mA.startSynchronization();
			mA.setSpeed(180);
			mC.setSpeed(180);		
			mA.forward();
			mC.forward();			
			mA.endSynchronization();
			Delay.msDelay((long)duration);
			mA.startSynchronization();
			mA.stop();
			mC.stop();
			
			mA.endSynchronization(); 
			
		}
	}
	
	public void navigateToGoal(ArrayList<int[]> path, float cellSize){
		System.out.println(carOrientation);
		int[] previousVector = new int[]{20-21,5-4};
		navigate(path, cellSize, previousVector);
		
	}
	
	public void navigateToStart(ArrayList<int[]> path, float cellSize){
		System.out.println(carOrientation);
		int[] previousVector = new int[]{0,9-8};
		navigate(path, cellSize, previousVector);
	}
	
	public int getMapIndex(int colorAtWall){ // red:0 green:7 black:7 
		if(colorAtWall == 7){
			return 3;
		}else{
			return 4;
		}
	}
	
	public void testrotate(){
		mA.setSpeed(180);
		mC.setSpeed(180); 
		while(carOrientation > 45){
			mA.forward();
			mC.backward();
			angleMode.fetchSample(angleSample, 0);
			carOrientation = angleSample[angleMode.sampleSize() - 1];
			System.out.println(carOrientation);
		}
	}
	public void rotateToGoal(int[] previousVector){ //TODO test for robot
		gyroSensor.reset();
		carOrientation = 0; 
		int[] newVector = new int[]{1-1,9-8};
		float crossProduct = previousVector[0]*newVector[1]-previousVector[1]*newVector[0];
		float dotProduct = previousVector[0]*newVector[0]+previousVector[1]*newVector[1];
		float angleToRotate = (float) ((float) Math.atan2(crossProduct,dotProduct) * (180/Math.PI));
		//angleToRotate = (float) (angleToRotate*angleCorrection);
		angleMode.fetchSample(angleSample, 0);
		carOrientation = angleSample[angleMode.sampleSize() - 1];
		mA.setSpeed(180);
		mC.setSpeed(180);
		if(angleToRotate>0){
			while(carOrientation < angleToRotate){ 
				mA.backward();
				mC.forward();
				angleMode.fetchSample(angleSample, 0);
				carOrientation = angleSample[angleMode.sampleSize() - 1];
				System.out.println(carOrientation);
			}
		}else{
			while(carOrientation > angleToRotate){
				mA.forward();
				mC.backward();
				angleMode.fetchSample(angleSample, 0);
				carOrientation = angleSample[angleMode.sampleSize() - 1];
				System.out.println(carOrientation);
			}
		}
		
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
	float LOCALIZATION_THRESHOLD = (float) 0.6; //recommended:0.5
	
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
		path = new ArrayList<int[]>(); 
		openList = new ArrayList<>();
		closedList = new ArrayList<>();
		addObstaclesToClosedList();
		generateNeighbours(); //populate mapNodeToNeighbours
		mapNodeToGnHnFn.put(startingNode, new float[]{0f,calculateHn(startingNode),calculateHn(startingNode)});
		plan();
	}
	
	private void addObstaclesToClosedList(){
		int maxX = map[0].length; 
		int maxY = map.length;
		for(int y=0; y < maxY; ++y){
			for(int x=0; x < maxX; ++x){
				if(map[y][x] == 1){
					closedList.add(new int[]{x,y});
				}
			}
		}
	}
	
	/**
	 * Apart from moving foward,backward,right and left, the robot can also move diagonally.
	 */
	private void generateNeighbours(){
		int maxX = map[0].length; 
		int maxY = map.length;
		for(int y=0; y < maxY; ++y){
			for(int x=0; x < maxX; ++x){
				ArrayList<int[]> neighbours = new ArrayList<>();
				if(y+1 < maxY){
					if(map[y+1][x] != 1){
						neighbours.add(new int[]{x,y+1});
					}
				}
				if(y-1 >= 0){
					if(map[y-1][x] != 1){
						neighbours.add(new int[]{x,y-1});
					}
				}
				if(x+1 < maxX){
					if(map[y][x+1] != 1){
						neighbours.add(new int[]{x+1,y});
					}
				}
				if(x-1 >= 0){
					if(map[y][x-1] != 1){
						neighbours.add(new int[]{x-1,y});
					}
				}
				if(x-1 >= 0 && y-1>=0){
					if(map[y-1][x-1] != 1){
						neighbours.add(new int[]{x-1,y-1});
					}
				}
				if(x-1 >= 0 && y+1< maxY){
					if(map[y+1][x-1] != 1){
						neighbours.add(new int[]{x-1,y+1});
					}
				}
				if(x+1 < maxX && y-1>=0){
					if(map[y-1][x+1] != 1){
						neighbours.add(new int[]{x+1,y-1});
					}
				}
				if(x+1 < maxX && y+1< maxY){
					if(map[y+1][x+1] != 1){
						neighbours.add(new int[]{x+1,y+1});
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
			for(int[] neighbour : getNeighbours(currentNode)){
				if(!isInClosedList(neighbour)){
					float fn = calculateFn(currentNode,neighbour);
					if(isInOpenList(neighbour)){
						if(getNodeFn(neighbour) > fn){
							setNodeGnHnFn(neighbour,calculateGnHnFn(currentNode, neighbour));
							setNodeParent(neighbour,currentNode);
						}
					}else{		
						openList.add(neighbour);  
						setNodeGnHnFn(neighbour,calculateGnHnFn(currentNode, neighbour));
						setNodeParent(neighbour,currentNode);
					}
				}			
			}
			int[] nodeWithLowestFn = getNodeWithLowestFn();
			currentNode = nodeWithLowestFn;
			removeFromOpenList(nodeWithLowestFn);
			closedList.add(nodeWithLowestFn);
		}while(!openList.isEmpty() || !isInClosedList(goalNode));
		
		if(isInClosedList(goalNode)){
			path.add(goalNode);
			path = generatePath(path, goalNode);
		}		
	}
	
	private void removeFromOpenList(int[] node){
		Iterator<int[]> iter = openList.iterator();

		while (iter.hasNext()) {
		    int[] eachNode = iter.next();
		    if (eachNode[0] == node[0] && eachNode[1] == node[1])
		        iter.remove();
		}
	}
	
	private ArrayList<int[]> getNeighbours(int[] node){
		for(int[] eachNode: mapNodeToNeighbours.keySet()){
			if(eachNode[0] == node[0] && eachNode[1] == node[1]){
				return mapNodeToNeighbours.get(eachNode);
			}
		}
		return null;
	}
	
	private boolean isInOpenList(int[] node){
		for(int[] eachNode : openList){
			if(eachNode[0] == node[0] && eachNode[1] == node[1]){
				return true;
			}
		}
		return false;
	}
	
	private boolean isInClosedList(int[] node){
		for(int[] eachNode : closedList){
			if(eachNode[0] == node[0] && eachNode[1] == node[1]){
				return true;
			}
		}
		return false;
	}
	
	private float calculateFn(int[] parent, int[] child){
		float parentGn = getNodeGn(parent);
		float childGn = parentGn + 1;
		float childHn = calculateHn(child);
		float childFn = childGn + childHn;
		return childFn;
	}
	
	private float[] calculateGnHnFn(int[] parent, int[] child){
		float parentGn = getNodeGn(parent);
		float childGn = parentGn + 1;
		float childHn = calculateHn(child);
		float childFn = childGn + childHn;
		float[] newScores = {childGn,childHn,childFn};
		return newScores;
	}
	
	private void setNodeGnHnFn(int[] node, float[] scores){
		mapNodeToGnHnFn.put(node, scores);
	}
	
	/**
	 * @param node
	 * @return the Manhattan distance of the node to the goal node.
	 */
	private float calculateHn(int[] node){
		float heuristic = Math.abs(node[0]-goalNode[0]) + Math.abs(node[1]-goalNode[1]);
		return heuristic;
	}

    private void setNodeParent(int[] newNode, int[] parentNode){
    	mapNodeToParent.put(newNode, parentNode);
    }
    
    private int[] getNodeWithLowestFn(){
    	float lowestScore = 10000f;
    	int[] nodeWithLowestScore = openList.get(0);
    	for(int[] eachNode : openList){
    		float eachNodeFn = getNodeFn(eachNode);
    		if(eachNodeFn < lowestScore){
    			lowestScore = eachNodeFn;
    			nodeWithLowestScore = eachNode;
    		}
    	}
    	return nodeWithLowestScore;
    }
    
    private float getNodeFn(int[] node){
    	for(int[] eachNode : mapNodeToGnHnFn.keySet()){
    		if(eachNode[0] == node[0] && eachNode[1] == node[1]){
    			return mapNodeToGnHnFn.get(eachNode)[2];
    		}
    	}
    	return 0f;
    }
    
    private float getNodeGn(int[] node){
    	for(int[] eachNode : mapNodeToGnHnFn.keySet()){
    		if(eachNode[0] == node[0] && eachNode[1] == node[1]){
    			return mapNodeToGnHnFn.get(eachNode)[0];
    		}
    	}
    	return 0f;
    }
    
    
    /**
     * Return a path corresponding to the sequence of nodes that the robot should travel
     * to reach to goal from starting point, e.g. [startingPoint,[1,2],[2,2],[2,3],goalPoint]
     */
    private ArrayList<int[]> generatePath(ArrayList<int[]> path,int[] nextNode){
    	int[] parentNode = getNodeParent(nextNode);
    	if(parentNode[0] == startingNode[0] && parentNode[1] == startingNode[1]){
    		path.add(0,parentNode);
    		return path;
    	}else{		
    		path.add(0,parentNode);
    		return generatePath(path,parentNode);
    	}
    }
    
    private int[] getNodeParent(int[] node){
    	for(int[] eachNode : mapNodeToParent.keySet()){
    		if(eachNode[0] == node[0] && eachNode[1]==node[1]){
    			return mapNodeToParent.get(eachNode);
    		}
    	}
    	return null;
    };
    
    public ArrayList<int[]> convertPathToWaypoints(ArrayList<int[]> path){ 
    	int previousIndex = 0;
    	ArrayList<int[]> newpath = new ArrayList<int[]>();
    	for(int i=1 ; i < path.size() ; ++i){
    		if(path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==0 && previousIndex==1){
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    			continue;
    		}
    		else if(path.get(i)[0]-path.get(i-1)[0]==0 && path.get(i)[1]-path.get(i-1)[1]==1 && previousIndex==2){
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    			continue;
    		}
    		else if(path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==1 && previousIndex==3){
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    			continue;
    		}
    		else if(path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==-1 && previousIndex==4){
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    			continue;
    		}
    		else if(path.get(i)[0]-path.get(i-1)[0]==-1 && path.get(i)[1]-path.get(i-1)[1]==1 && previousIndex==5){
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    			continue;
    		}
    		else if(path.get(i)[0]-path.get(i-1)[0]==-1 && path.get(i)[1]-path.get(i-1)[1]==-1 && previousIndex==6){
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    			continue;
    		}
    		else if(path.get(i)[0]-path.get(i-1)[0]==0 && path.get(i)[1]-path.get(i-1)[1]==-1 && previousIndex==7){
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    			continue;
    		}
    		else if(path.get(i)[0]-path.get(i-1)[0]==-1 && path.get(i)[1]-path.get(i-1)[1]==0 && previousIndex==8){
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    			continue;
    		}
    		else{
    			if( path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==0){
        			previousIndex = 1;
        		}
        		if(path.get(i)[0]-path.get(i-1)[0]==0 && path.get(i)[1]-path.get(i-1)[1]==1){
        			previousIndex = 2;
        		}
        		if(path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==1){
        			previousIndex = 3;
        		}
        		if(path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==-1){
        			previousIndex = 4;
        		}
        		if(path.get(i)[0]-path.get(i-1)[0]==-1 && path.get(i)[1]-path.get(i-1)[1]==1){
        			previousIndex = 5;
        		}
        		if(path.get(i)[0]-path.get(i-1)[0]==-1 && path.get(i)[1]-path.get(i-1)[1]==-1){
        			previousIndex = 6;
        		}
        		if(path.get(i)[0]-path.get(i-1)[0]==0 && path.get(i)[1]-path.get(i-1)[1]==-1){
        			previousIndex = 7;
        		}
        		if(path.get(i)[0]-path.get(i-1)[0]==-1 && path.get(i)[1]-path.get(i-1)[1]==0){
        			previousIndex = 8;
        		}
    			newpath.add(path.get(i-1));
    			if(i == path.size()-1){
    				newpath.add(path.get(i));
    			}
    		} 		
    	}
    	return newpath;	
    }
 }


class MapGrid{
	
	int[][] map = new int[16][16];
	float cellSize;
	int xMAX = 16;
	int yMAX = 16;
	int[] goalNode;
	int[] startingNode;
	int index;
	
	public MapGrid(int index, int[] startingNode, int[] goalNode){
		this.index = index;
		setMap();
		this.goalNode = goalNode;
		this.startingNode = startingNode;
		cellSize = 1.215f/map.length; //in meters
	}
	
	private void setMap(){
		for(int y=0; y<yMAX; ++y){
			for(int x=0; x<xMAX; ++x){
				map[y][x] = 0;
			}
		}
		setObstacles();
	}
	
	public void setObstacles(){
        //top left corner obstacle
    	for(int y=0; y<4; ++y){
	        for(int x=0;x<4-y ;++x){
	        	map[15-y][x] = 1;
	        }
    	}
        //goal
        for(int y=yMAX-5; y>yMAX-8;--y){
        	for(int x=0; x<3; x++){
        		map[y][x] = 1;
        	}
        }        
        //bottom right corner obstacle
        for(int y = 0; y < 4 ; y++){
	        for(int x = 12+y; x < xMAX; x++){
	            map[y][x] = 1;
	        }
        } 
        //line 
        for(int i=2; i<7 ; ++i){
        	map[i][15-i] = 1;
        }
        
        //red area
        if(index == 1 || index == 2){
	        for(int i=0;i<8;++i){
	        	for(int x=15-i; x<xMAX; x++){
	        		map[0+i][x] = 1;
	        	}
	        }
	        for(int i=0;i<8;++i){
	        	for(int x=15-i; x<xMAX; x++){
	        		map[15-i][x] = 1;
	        	}
	        }
        }
        if(index == 3 || index == 4){
        	for(int i=0;i<8;++i){
	        	for(int y=i; y>=0; --y){
	        		map[y][0+i] = 1;
	        	}
	        }    
        	for(int i=0;i<8;++i){
	        	for(int y=i; y>=0; --y){
	        		map[y][15-i] = 1;
	        	}
	        }       	
        }
        
        //mid obstacle
        if(index==1){
        	for(int n=-1;n<2; ++n){
		        for(int i=4; i<10; ++i){ //i=9 for the actual obstacle, extended to make robot move past close to obstacle
		        	map[i+n][i] = 1;
		        }
        	}
        }       
        if(index==2){
        	for(int n=-1;n<2; ++n){
		        for(int i=6; i<11; ++i){ //i=9 for the actual obstacle, extended to make robot move past close to obstacle
		        	map[i+n][i] = 1;
		        }
        	}
        }      
        if(index==3){
        	for(int n=-1;n<2; ++n){
		        for(int i=6; i<11; ++i){ //i=9 for the actual obstacle, extended to make robot move past close to obstacle
		        	map[i+n][i] = 1;
		        }
        	}
        }    
        if(index==4){
        	for(int n=-1;n<2; ++n){
		        for(int i=6; i<12; ++i){ //i=9 for the actual obstacle, extended to make robot move past close to obstacle
		        	map[i+n][i] = 1;
		        }
        	}
        }  
        //round obstacle
        if(index==1){
	        //one round obstacle on the left
	        map[2][2] = 1;
        }
        if(index==2){
	        //one round obstacle on the right
	        map[3][3] = 1;
        }
        if(index==3){
	        //one round obstacle on the left when returning
	        map[12][12] = 1;
        }
        if(index==4){
	        //one round obstacle on the right when returning
	        map[13][13] = 1;
        }
    }
	
	public float getCellSize(){
		return cellSize;
	}
	
	public int[][] getMap(){
		return map;
	}
	
	public int[] getGoalNode(){
		return goalNode;
	}
	
	public int[] getStartingNode(){
		return startingNode;
	}
	
	public void printMap(){
		for(int y=(yMAX-1); y>-1; --y){
			for(int x=0; x<xMAX; ++x){
				System.out.print(map[y][x]);;
			}
			System.out.println();
		}	
	}
	
	public void printMapWithPath(ArrayList<int[]> path){
		for(int[] point:path){
			map[point[1]][point[0]]=2;
		}
		for(int y=(yMAX-1); y>-1; --y){
			for(int x=0; x<xMAX; ++x){
				System.out.print(map[y][x]);;
			}
			System.out.println();
		}	
	}
}


