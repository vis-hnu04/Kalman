#ifndef HUNGARIANALGORITHM_H_
#define HUNGARIANALGORITHM_H_ HUNGARIANALGORITHM_H_
#include<algorithm>
#include <vector>



class HungarianAlgorithm
{

public:
 HungarianAlgorithm(std::vector<std::vector<float>> & matrix );
 
/* single function to solve the assignment problem
  @param[out] vector<pair<int,int>> associations of observations with targets */

 std::vector<std::pair<int,int>> solve();

/*  For each row of the matrix, find the smallest element and subtract it from every 
 * element in its row.  
 * For each col of the matrix, find the smallest element and subtract it from every 
 * element in its col. Go to Step 2.
 * @ Param [in] cost matrix  */ 

 void first_step(std::vector<std::vector<float>> &matrix);

 /* Find a zero (Z) in the resulting matrix.  In this step, 
 * we introduce the mask matrix M, which in the same dimensions as cost matrix . If M(i,j)=1 then C(i,j) is a 
 * starred zero,  If M(i,j)=2 then C(i,j) is a primed zero.  We also define two vectors 
 * RowCover and ColCover that are used to "cover" the rows and columns of the cost matrix.
 * If the number of ones in ColCover = number of columns of the cost matrix, then we have found the 
 * optimal solution.
 */ 

 void second_step(std::vector<std::vector<int>> &mask , std::vector<std::vector<float>> &matrix,
                                      std::vector<int> & rowcover, std::vector<int>& colcover);

 /* To clear the row and column vectors. 
 */
 inline void clear_vectors(std::vector<int> &vec);

/* Used to find the optimal assignment from the mask matrix.
   associations are written to the _result. 
   @param[in] vector<vector<int>> mask  */

 void optimal_assignment(std::vector<std::vector<int>> &mask);

 private:
 std::vector<std::vector<float>> _costMatrix;
 std::size_t _rows ;
 std::size_t _cols;
 std::vector<std::pair<int,int> > _result;
 bool _found;
 

};

#endif