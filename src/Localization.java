/**
 * This class contains the probability distribution on the map pattern for localization, and the methods 
 * for updating the probability distribution.
 * @author Sarosh
 *
 */
public class Localization {
	
	float[] probabilityDistribution;
	int size = 37;
	int[] grid;
	float SensorWorkProb = (float) 0.99; //TODO
	float LOCALIZATION_THRESHOLD = (float) 0.7;
	
	public Localization(int size){
		this.size = size;
		probabilityDistribution = new float[this.size];
		grid = new int[this.size];
		initializeProb();
		initializeGrid();
	}
	
	/**
	 * To initialize the grid (sensor model) to store whether each index corresponds to having a blue
	 * or white color. This refers to the line pattern of the challenge.
	 */
	public void initializeGrid(){ //2.70, 5.40, 8.10 for rounded down % of 1.7cm,3.4cm,5.1cm respectively, total length:62.9
		// Pattern: 3.4cmWhite, 5.1cmBlue, 1.7W, 3.4B, 3.4W, 5.1B, 1.7W, 3.4B, 3.4W, 5.1B, 3.4W, 5.1B, 1.7W, 3.4B, 3.4W, 5.1B, 1.7W, 3.4B
		for(int i=0;i<2;++i){
			grid[i]=0;
		}
		for(int i=2;i<5;++i){
			grid[i]=1;
		}
		for(int i=5;i<6;++i){
			grid[i]=0;
		}
		for(int i=6;i<8;++i){
			grid[i]=1;
		}
		for(int i=8;i<10;++i){
			grid[i]=0;
		}
		for(int i=10;i<13;++i){
			grid[i]=1;
		}
		for(int i=13;i<14;++i){
			grid[i]=0;
		}
		for(int i=14;i<16;++i){
			grid[i]=1;
		}
		for(int i=16;i<18;++i){
			grid[i]=0;
		}
		for(int i=18;i<21;++i){
			grid[i]=1;
		}
		for(int i=21;i<23;++i){
			grid[i]=0;
		}
		for(int i=23;i<26;++i){
			grid[i]=1;
		}
		for(int i=26;i<27;++i){
			grid[i]=0;
		}
		for(int i=27;i<29;++i){
			grid[i]=1;
		}
		for(int i=29;i<31;++i){
			grid[i]=0;
		}
		for(int i=31;i<34;++i){
			grid[i]=1;
		}
		for(int i=34;i<35;++i){
			grid[i]=0;
		}
		for(int i=35;i<37;++i){
			grid[i]=1;
		}
		
		/**
		float[] patternInPercent = {5.40f, 8.10f, 2.70f, 5.40f , 5.40f, 8.10f, 2.70f, 5.40f, 5.40f, 8.10f, 5.40f, 8.10f, 2.70f, 5.40f, 5.40f, 8.10f, 2.70f, 5.40f };
		int start = 0;
		int binaryValue = 0; // first portion is in white color
		
		for(int i=0;i<patternInPercent.length;++i){
			int lengthToBeAdded = (int)patternInPercent[i]*size;		
			for(int j = start; j< (start + lengthToBeAdded) ; ++j){
				grid[j] = binaryValue;
			}
			binaryValue = (binaryValue == 0) ? 1 : 0;
			start = start + lengthToBeAdded;
		}
		**/
			
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
	public void updateAfterMoving(int direction){ //TODO
		if(direction == 0){ //moving forward
			for(int i=0; i<size; ++i){		
				probabilityDistribution[i] =(float) (probabilityDistribution[i+1]*0.99 + probabilityDistribution[i]*0.1);
			}
		}	
		if(direction == 1){ //moving backward
			for(int i=0; i<size; ++i){		
				probabilityDistribution[i] =(float) (probabilityDistribution[i-1]*0.99 + probabilityDistribution[i]*0.1);
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
