#include"HungarianAlgorithm.h"

HungarianAlgorithm::HungarianAlgorithm(std::vector<std::vector<float>> &matrix){
    _costMatrix = matrix;
    _rows    = _costMatrix.size();
    _cols    = _costMatrix[0].size();

}

void HungarianAlgorithm::first_step(std::vector<std::vector<float>> &matrix){
   
   for(auto &row :matrix){
       auto minimum = *std::min_element(row.begin(),row.end());
      
       if(minimum > 0)
        {
            for (auto &values: row){
                values -= minimum;
            }
        }
   }

   for (int j=0; j<_cols; ++j) {
        float minval = std::numeric_limits<float>::max();
        for (int i=0; i< _rows; ++i) {
            minval = std::min(minval, matrix[i][j]);
        }

         if (minval > 0) {
            for (int i=0; i<_rows; ++i) {
                matrix[i][j] -= minval;
            }
         }
   }

}

void HungarianAlgorithm::second_step(std::vector<std::vector<int>> &mask , std::vector<std::vector<float>> &matrix,
                                      std::vector<int> & rowcover, std::vector<int>& colcover)
               {
                  for(int i{0}; i< _rows; ++i)
                      for (int j{0}; j< _cols ; ++j)
                          if(matrix[i][j] == 0)
                              if(rowcover[i] == 0 & colcover[j] == 0){
                                  mask[i][j]++;
                                  rowcover[i] +=1;
                                  colcover[j] +=1;
                              }
        
                                      }


inline void HungarianAlgorithm::clear_vectors(std::vector<int> &vec){
    for(auto & value: vec) 
       value =0;
}


 void HungarianAlgorithm::optimal_assignment(std::vector<std::vector<int>> &mask){

       for(int r = 0 ; r < _rows; r++){
           for(int c =0 ; c < _cols; c++){
               if(mask[r][c]){
                  std::pair<int,int> ObjectTrack(r,c);
                   _result.push_back(ObjectTrack);
               }
           }
       }
         
}

std::vector<std::pair<int,int>> HungarianAlgorithm::init(){
    first_step(_costMatrix);
    
    std::vector<std::vector<int>> mask (_costMatrix.size(),std::vector<int>(_costMatrix.size(),0));
    std::vector<int> rowcover(_rows, 0);
    std::vector<int>colcover(_cols,0); 

    second_step(mask,_costMatrix, rowcover,colcover);
  
    clear_vectors(rowcover);
    clear_vectors(colcover);

    optimal_assignment(mask);
    return _result;

}
