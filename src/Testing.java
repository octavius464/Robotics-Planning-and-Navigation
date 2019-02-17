import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

public class Testing {

	public static void main(String args[]){
		/*
		Robot1 robot = new Robot1();
		ArrayList<Integer> measurements = new ArrayList<>();
		float sum=0;
		float distance = 0.05f; 
		float increment = 0.05f;
		for(int i=0;i<10;++i){
			robot.moveForward(distance);		
			Delay.msDelay(10000);	
		}	
		
		Delay.msDelay(100000);	
		
		while(true){
			System.out.println(robot.getColorMeasurement());	
		}
		*/
		

		
		MapGrid mapGrid = new MapGrid(new int[][] {{0,0,0,0},{0,1,1,0},{0,1,1,0},{0,0,0,0}},new int[]{0,0} ,new int[]{3,3});

		AStarPlanner1 planner = new AStarPlanner1(mapGrid.getStartingNode(), mapGrid.getGoalNode(), mapGrid.getMap());
		ArrayList<int[]> path = planner.getPath();
		for(int[] wayPoint : path){
			System.out.println(wayPoint[0] +" " + wayPoint[1]);
		}
		ArrayList<int[]> newpath = planner.convertPathToWaypoints(path);
		System.out.println("new Path:");
		for(int[] wayPoint : newpath){
			System.out.println(wayPoint[0] +" " + wayPoint[1]);
		}
		/*
		HashMap<int[],ArrayList<int[]>> map = new HashMap<>();
		int[] a = {0,0};
		ArrayList<int[]> aList = new ArrayList<>();
		aList.add(new int[]{0,0});
		aList.add(new int[]{2,3});
		int[] d = {0,0};
		
		map.put(a,aList);
		int[] b = {1,9};
		ArrayList<int[]> bList = new ArrayList<>();
		aList.add(new int[]{1,2});
		aList.add(new int[]{2,3});
		map.put(b,bList);
		for(int[] eachKey:map.keySet()){
			if(eachKey[0] == d[0] && eachKey[1] == d[1]){
				System.out.println(true);
			}
		}
		*/
	}
}


class Robot1 {

	static double WHEEL_RADIUS = 0.015;
	int localizedPos;
	EV3ColorSensor colorSensor;
	RegulatedMotor mA;
	RegulatedMotor mC;
	
	public Robot1(){
		colorSensor = new EV3ColorSensor(SensorPort.S4);
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
	
	/**
	 * Move 1.7cm in each iteration for the localization. 
	 * @param direction 0 for moving forward, 1 for moving backward
	 */
	public void moveForward(float distance){
		double meterspersecond = 2.0*Math.PI*WHEEL_RADIUS/2.0;
		double duration = (distance /meterspersecond) * 1000.0; //move forward 1.7cm/cell of the array each iteration

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

class AStarPlanner1{
	
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
	
	public AStarPlanner1(int[] startingNode, int[] goalNode, int[][] map2DGrid){
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
	
	private void generateNeighbours(){ //TODO test code
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
						openList.add(neighbour);  //TODO only add if not in closed list as well?
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
    }
    
    public ArrayList<int[]> convertPathToWaypoints(ArrayList<int[]> path){ //TODO
    	int previousIndex = 0;
    	ArrayList<int[]> newpath = new ArrayList<int[]>();
    	newpath.add(path.get(0));
    	for(int i=1 ; i < path.size() ; ++i){
    		if(previousIndex==0 && path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==0){
    			previousIndex = 1;
    		}
    		if(previousIndex==0 && path.get(i)[0]-path.get(i-1)[0]==0 && path.get(i)[1]-path.get(i-1)[1]==1){
    			previousIndex = 2;
    		}
    		if(previousIndex==0 && path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==1){
    			previousIndex = 3;
    		}
    		
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
    			newpath.add(path.get(i-1));
    		} 		
    	}
    	return newpath;	
    }
    
    
 }


class MapGrid1{
	
	int[][] map;
	float cellSize;
	int[] goalNode;
	int[] startingNode;
	
	public MapGrid1(int[][] map,int[] startingNode, int[] goalNode){
		this.map = map;
		this.goalNode = goalNode;
		this.startingNode = startingNode;
		cellSize = 1.215f/map.length; //in meters
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
}
