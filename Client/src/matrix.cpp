#include"matrix.h"


Matrix::Matrix(): _m_rowSize{0}, _m_colSize{0}{};

Matrix::Matrix(unsigned rowSize, unsigned colSize, double initial){
    _m_rowSize = rowSize;
    _m_colSize = colSize;
    _m_matrix.resize(rowSize);
    for (unsigned i = 0; i < _m_matrix.size(); i++)
    {
        _m_matrix[i].resize(colSize, initial);
    }
}

Matrix::Matrix(std::vector<std::vector<float>>& vec){
     _m_rowSize = vec.size();
     _m_colSize = vec[0].size();
     _m_matrix.resize(_m_rowSize);
     for(int i {0}; i< _m_rowSize; i++){
         std::copy(vec[i].begin(),vec[i].end(), std::back_inserter(_m_matrix[i]));
     }
     
}

Matrix Matrix::operator+(Matrix &B){
    Matrix sum(_m_rowSize, _m_colSize, 0.0);
    unsigned i,j;
    for (i = 0; i < _m_rowSize; i++)
    {
        for (j = 0; j < _m_colSize; j++)
        {
            sum(i,j) = this->_m_matrix[i][j] + B(i,j);
        }
    }
    return sum;
}


Matrix Matrix::operator-(Matrix & B){
    Matrix difference(_m_rowSize, _m_colSize, 0.0);
    unsigned i,j;
    for (i = 0; i < _m_rowSize; i++)
    {
        for (j = 0; j < _m_colSize; j++)
        {
            difference(i,j) = this->_m_matrix[i][j] - B(i,j);
        }
    }
    
    return difference;
}



Matrix Matrix::operator*(Matrix & B){
    Matrix product(_m_rowSize,B.getCols(),0.0);
    if(_m_colSize == B.getRows())
    {
        unsigned i,j,k;
        double temp = 0.0;
        for (i = 0; i < _m_rowSize; i++)
        {
            for (j = 0; j < B.getCols(); j++)
            {
                temp = 0.0;
                for (k = 0; k < _m_colSize; k++)
                {
                    temp += _m_matrix[i][k] * B(k,j);
                }
                product(i,j) = temp;
                //cout << product(i,j) << " ";
            }
            //cout << endl;
        }
        return product;
    }
    else
    {
        std::cout<<"size incompatimble"<<std::endl  ;
    }
}

double& Matrix::operator()(const unsigned &rowNo, const unsigned & colNo)
{
    return this->_m_matrix[rowNo][colNo];
}


Matrix Matrix::operator/(double &s){

 Matrix result(_m_rowSize,_m_colSize,0);
 for(unsigned i{0};i<_m_rowSize;i++){
     for(unsigned j{0};j <_m_colSize;j++){
         result(i,j) = _m_matrix[i][j]/s; 
     }
 }
   

}


unsigned Matrix::getRows() const
{
    return this->_m_rowSize;
}

// returns col #
unsigned Matrix::getCols() const
{
    return this->_m_colSize;
}

// Take any given matrices transpose and returns another matrix
Matrix Matrix::transpose()
{
    Matrix transposed(_m_colSize,_m_rowSize,0.0);
    for (unsigned i = 0; i < _m_colSize; i++)
    {
        for (unsigned j = 0; j < _m_rowSize; j++) {
            transposed(i,j) = this->_m_matrix[j][i];
        }
    }
    return transposed;
}

void Matrix::print() const
{
    std::cout << "Matrix: " << std::endl;
    for (unsigned i = 0; i < _m_rowSize; i++) {
        for (unsigned j = 0; j < _m_colSize; j++) {
            std::cout << "[" << _m_matrix[i][j] << "] ";
        }
        std::cout << std::endl;
    }
}

 Matrix& Matrix::createIdentityMatrix()

{
    // if it's not a square matrix, we cant make an identity matrix.
    assert(getCols() == getRows());
    
       
    for (int i=0;i<_m_rowSize; i++)
    {
        _m_matrix[i][i] = 1;
    }
    
    return *this;
}

Matrix& Matrix::createDiagonalMatrix(double  value){
    assert(getCols() == getRows());
    
       
    for (int i=0;i<_m_rowSize; i++)
    {
        _m_matrix[i][i] = value;
    }
    
    return *this;
}




Matrix Matrix::appendColumn(Matrix &b){
    assert(b.getRows()== getRows());
    int cur_cols = getCols();
    
    Matrix augmented = *this;
    for (int i=0;i<getRows();i++)
    {
        augmented._m_matrix[i].resize(cur_cols + b.getCols());
        for (int j=cur_cols;j<cur_cols + b.getCols();j++)
        {
            augmented(i,j) = b(i,j-cur_cols);
        }
    }
    augmented._m_colSize = getCols()+b.getCols();
    
    return augmented;
}


Matrix Matrix::gaussianJordan()
{
    Matrix b(getRows(),getCols(),0);                                 //assume |x y | is the matrix we are calculating the inverse.
    Matrix result(getRows(),getCols(),0);                            //       |w z |
                                                                  
    b.createIdentityMatrix();
    Matrix augmented = appendColumn(b);                             //  step 1 .appends the identity matrix to the given matrix.      |x y | 1 0| 
                                                                    // after this step the augmented matrix will be -->               |w z | 0 1|
   for (int i = 0;i < getRows();i++){
    
      if(augmented(i,i) == 0){
          std::cout<<"mathematical error." <<std::endl;
      }
    
    for(int j{0};j<getCols();j++){
       
        if(i != j){
            
            float ratio = augmented(j,i)/augmented(i,i);
       
            for(int k{0}; k< augmented.getCols(); k++){
             
                augmented(j,k) = augmented(j,k) - ratio*augmented(i,k);         
            
            } // k
          } // if(i!=j)
        }// for j
      }// i
    

    // Makes the the given matrix identity.
     for(int i=0;i<getRows();i++)
		 {                                                                            
			  for(int j=getCols();j< augmented.getCols(); j++)
			  {
			   	augmented(i,j) = augmented(i,j)/augmented(i,i);
			  }
		 } 
                     // |1 0 | a b |            |a b | is the inverse of the given 
                     // |0 1 | c d |   , matrix |c d |       matrix.
   
   //writing back the results from the appended portion to the result object.
   for(int i = 0;i<getRows();i++)
		 {
			  for(int j=getCols();j< augmented.getCols();j++)
			  {
			   	result(i,j-getCols()) = augmented(i,j); 
			  }
			  
		 }
      
      return result;
}

/** non member function for calculating the determinant. */
double calculateDeterminant(std::vector<std::vector<double>> mat){
   
   
        // it's a recursive function that can handle matrix of any dimension
        float det = 0; // the determinant value will be stored here
        if (mat.size() == 1)  
        {
            return mat[0][0]; // no calculation needed
        }
        else if (mat.size() == 2)             // matrix size 2 is the base condition.
        {
            //in this case we calculate the determinant of a 2-dimensional matrix in a 
            //default procedure
            det = (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]);
            return det;
        }
        else
        {
            //in this case we calculate the determinant of a squared matrix that have 
            for (int p = 0; p < mat[0].size(); p++)
            {
                //this loop iterate on each elements of the first row in the matrix.
                //at each element we cancel the row and column it exist in
                //and form a matrix from the rest of the elements in the matrix
                vector<vector<double>> TempMatrix; // to hold the shaped matrix;
                for (int i = 1; i < mat.size(); i++)
                {
                    // iteration will start from row one cancelling the first row values
                    vector<double> TempRow;
                    for (int j = 0; j < mat[i].size(); j++)
                    {
                        // iteration will pass all cells of the i row excluding the j 
                        //value that match p column
                        if (j != p)
                        {
                           TempRow.push_back(mat[i][j]);//add current cell to TempRow 
                        }
                    }
                    if (TempRow.size() > 0)
                        TempMatrix.push_back(TempRow);
                        }
                det = det + mat[0][p] * pow(-1, p) * calculateDeterminant(TempMatrix);
                
            }
            return det;
        }
    }


double Matrix::det()
   {     
        double determinant = calculateDeterminant(this->_m_matrix);
        return determinant;
         
    }

Matrix Matrix::inverse(){

     Matrix inv = gaussianJordan();
     return inv;
}

