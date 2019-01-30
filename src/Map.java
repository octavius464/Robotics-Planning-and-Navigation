
public class Map {	
	
	float[][] map;
	int size;
	
	public Map(int size){
		map = new float[size][size];
	}
	
	public void updateAfterSensing(float[][] observationLikelihood){
		float normalizationFactor = 0;
		for(int i=0; i<size; ++i){
			for(int j=0; j<size; ++j){
				map[i][j] = observationLikelihood[i][j] * map[i][j];
				normalizationFactor = normalizationFactor + map[i][j];
			}
		}
		for(int i=0; i<size; ++i){
			for(int j=0; j<size; ++j){
				map[i][j] = map[i][j]/normalizationFactor;
			}
		}
	}
	
	public void updateAfterMoving(int direction){
		if(direction == 0){ //moving upward
			for(int i=0; i<size; ++i){
				for(int j=0; j<size; ++j){
					map[i][j] =(float) (map[i][j-1]*0.85 + map[i][j]*0.05 + map[i-1][j]*0.05 + map[i+1][j]*0.05);
				}
			}
		}		
	}
	
	
}
