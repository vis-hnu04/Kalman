#ifndef HUNGARIANALGORITHM_H_
#define HUNGARIANALGORITHM_H_ HUNGARIANALGORITHM_H_
#include<algorithm>
#include <vector>



class HungarianAlgorithm
{

public:
 HungarianAlgorithm(std::vector<std::vector<float>> & matrix );
 std::vector<std::pair<int,int>> init();

 void first_step(std::vector<std::vector<float>> &matrix);
 void second_step(std::vector<std::vector<int>> &mask , std::vector<std::vector<float>> &matrix,
                                      std::vector<int> & rowcover, std::vector<int>& colcover);

 inline void clear_vectors(std::vector<int> &vec);

 void optimal_assignment(std::vector<std::vector<int>> &mask);

 private:
 std::vector<std::vector<float>> _costMatrix;
 std::size_t _rows ;
 std::size_t _cols;
 std::vector<std::pair<int,int> > _result;
 

};

#endif