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
		

		
		//MapGrid1 mapGrid = new MapGrid1(new int[][] {{0,0,0,0},{0,1,1,0},{0,1,1,0},{0,0,0,0}},new int[]{0,0} ,new int[]{3,3});
		MapGrid1 mapGrid = new MapGrid1(1, new int[]{11,4} ,new int[]{1,8});
		mapGrid.printMap();
		
		AStarPlanner1 planner = new AStarPlanner1(mapGrid.getStartingNode(), mapGrid.getGoalNode(), mapGrid.getMap());
		ArrayList<int[]> path = planner.getPath();
		System.out.println("Path length: "+path.size());
	
		
		System.out.println("Path length: "+path.size());
		for(int[] wayPoint : path){
			System.out.print("[" +wayPoint[0] +","+ wayPoint[1] + "], ");
		}
		System.out.println();
		ArrayList<int[]> newpath = planner.convertPathToWaypoints(path);
		System.out.println();
		System.out.println("new Path:");
		System.out.println("Path length: "+newpath.size());
		for(int[] wayPoint : newpath){
			System.out.print("[" +wayPoint[0] +"," + wayPoint[1]+ "], ");
		}
		System.out.println();
		mapGrid.printMapWithPath(newpath);
		System.out.println();
		System.out.println();
		
		TestNavigation testnavigate = new TestNavigation();
		testnavigate.testNavigateToGoal(newpath, mapGrid.getCellSize());
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
		generateNeighbours1(); //populate mapNodeToNeighbours
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
	 * Robot moves only upward,downward,right and left.
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
				mapNodeToNeighbours.put(new int[]{x,y}, neighbours);
			}
		}
	}
	
	/**
	 * Apart from moving foward,backward,right and left, the robot can also move diagonally.
	 */
	private void generateNeighbours1(){ 
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
    		if(previousIndex==0 && path.get(i)[0]-path.get(i-1)[0]==1 && path.get(i)[1]-path.get(i-1)[1]==-1){
    			previousIndex = 4;
    		}
    		if(previousIndex==0 && path.get(i)[0]-path.get(i-1)[0]==-1 && path.get(i)[1]-path.get(i-1)[1]==1){
    			previousIndex = 5;
    		}
    		if(previousIndex==0 && path.get(i)[0]-path.get(i-1)[0]==-1 && path.get(i)[1]-path.get(i-1)[1]==-1){
    			previousIndex = 6;
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
    			newpath.add(path.get(i-1));
    		} 		
    	}
    	return newpath;	
    }
    
    
 }

class MapGrid1{
	
	int[][] map = new int[16][16];
	float cellSize;
	int xMAX = 16;
	int yMAX = 16;
	int[] goalNode;
	int[] startingNode;
	int index;
	
	public MapGrid1(int index, int[] startingNode, int[] goalNode){
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


/*
class MapGrid{
	
	int[][] map = new int[30][30];
	float cellSize;
	int xMAX = 30;
	int yMAX = 30;
	int[] goalNode;
	int[] startingNode;
	
	public MapGrid(int[] startingNode, int[] goalNode){
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
        for(int x = 0; x < 8; x++){
            map[yMAX-1][x] = 1;
        }
        for(int x = 0; x < 7; x++){
            map[yMAX-2][x] = 1;
        }
        for(int x = 0; x < 6; x++){
            map[yMAX-3][x] = 1;
        }
        for(int x = 0; x < 5; x++){
            map[yMAX-4][x] = 1;
        }
        for(int x = 0; x < 4; x++){
            map[yMAX-5][x] = 1;
        }
        for(int x = 0; x < 3; x++){
            map[yMAX-6][x] = 1;
        }
        for(int x = 0; x < 2; x++){
            map[yMAX-7][x] = 1;
        }
        for(int x = 0; x < 1; x++){
            map[yMAX-8][x] = 1;
        }
        //goal
        for(int y=yMAX-9; y>yMAX-14;--y){
        	for(int x=0; x<6; x++){
        		map[y][x] = 1;
        	}
        }
        
        //bottom right corner obstacle
        map[3][xMAX-1] = 1;
        for(int x = 22; x < xMAX; x++){
            map[0][x] = 1;
        }
        for(int x = 23; x < xMAX; x++){
            map[1][x] = 1;
        }
        for(int x = 24; x < xMAX; x++){
            map[2][x] = 1;
        }
        for(int x = 25; x < xMAX; x++){
            map[3][x] = 1;
        }
        for(int x = 26; x < xMAX; x++){
            map[4][x] = 1;
        }
        for(int x = 27; x < xMAX; x++){
            map[5][x] = 1;
        }
        for(int x = 28; x < xMAX; x++){
            map[6][x] = 1;
        }
        map[7][29]=1;
          
        //mid obstacle
        for(int i=7; i<20; ++i){ //i=9 for the actual obstacle
        	map[i][i] = 1;
        }
        for(int i=7; i<20; ++i){
        	map[i+1][i] = 1;
        }
        for(int i=7; i<20; ++i){
        	map[i-1][i] = 1;
        }
       
        //one round obstacle on the left
        map[3][3] = 1;
        map[4][4] = 1;
        map[3][4] = 1;
        map[4][3] = 1;
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
*/

class TestNavigation{
	public void testNavigateToGoal(ArrayList<int[]> path, float cellSize){
		int[] previousVector = new int[]{20-21,5-4};
		for(int i = 1; i < path.size(); ++i){
			/*
			int[] newVector = new int[]{path.get(i)[0]-path.get(i-1)[0],path.get(i)[1]-path.get(i-1)[1]};
			float dotProduct = previousVector[0]*newVector[0]+previousVector[1]*newVector[1];
			float lengthOfPreviousVector = (float) Math.sqrt(Math.pow(previousVector[0], 2) + Math.pow(previousVector[1], 2));
			float lengthToNewWayPoint = (float) Math.sqrt(Math.pow(newVector[0],2)+Math.pow(newVector[1],2) );
			float angleToRotate = (float) ((float) Math.acos(dotProduct/(lengthToNewWayPoint*lengthOfPreviousVector)) * (180/Math.PI)) ;
			double distanceToTravel = Math.sqrt( Math.pow(path.get(i)[0]-path.get(i-1)[0],2) + Math.pow(path.get(i)[1]-path.get(i-1)[1],2) ) * cellSize; //in meters
		    
			previousVector = newVector;
			*/
			
			int[] newVector = new int[]{path.get(i)[0]-path.get(i-1)[0],path.get(i)[1]-path.get(i-1)[1]};
			float crossProduct = previousVector[0]*newVector[1]-previousVector[1]*newVector[0];
			float dotProduct = previousVector[0]*newVector[0]+previousVector[1]*newVector[1];
			float angleToRotate = (float) ((float) Math.atan2(crossProduct,dotProduct) * (180/Math.PI));
			float lengthOfPreviousVector = (float) Math.sqrt(Math.pow(previousVector[0], 2) + Math.pow(previousVector[1], 2));
			float lengthToNewWayPoint = (float) Math.sqrt(Math.pow(newVector[0],2)+Math.pow(newVector[1],2) );
			//float angleToRotate = (float) ((float) Math.asin(crossProduct/(lengthToNewWayPoint*lengthOfPreviousVector)) * (180/Math.PI)) ;
			double distanceToTravel = Math.sqrt( Math.pow(path.get(i)[0]-path.get(i-1)[0],2) + Math.pow(path.get(i)[1]-path.get(i-1)[1],2) ) * cellSize; //in meters
		    
			previousVector = newVector;
			
			
			System.out.println("Rotating Angle: "+angleToRotate + ", Distance To Travel: " +distanceToTravel );
		
			
		}
	}
}
