#include"matrix.h"

Matrix::Matrix(unsigned rowSize, unsigned colSize, double initial){
    m_rowSize = rowSize;
    m_colSize = colSize;
    m_matrix.resize(rowSize);
    for (unsigned i = 0; i < m_matrix.size(); i++)
    {
        m_matrix[i].resize(colSize, initial);
    }
}

Matrix::Matrix(std::vector<std::vector<double>>& vec){
     m_rowSize = vec.size();
     m_colSize = vec[0].size();
     m_matrix.resize(m_rowSize);
     for(int i {0}; i< m_rowSize; i++){
         std::copy(vec[i].begin(),vec[i].end(), std::back_inserter(m_matrix[i]));
     }
     
}

Matrix Matrix::operator+(Matrix &B){
    Matrix sum(m_colSize, m_rowSize, 0.0);
    unsigned i,j;
    for (i = 0; i < m_rowSize; i++)
    {
        for (j = 0; j < m_colSize; j++)
        {
            sum(i,j) = this->m_matrix[i][j] + B(i,j);
        }
    }
    return sum;
}


Matrix Matrix::operator-(Matrix & B){
    Matrix diff(m_colSize, m_rowSize, 0.0);
    unsigned i,j;
    for (i = 0; i < m_rowSize; i++)
    {
        for (j = 0; j < m_colSize; j++)
        {
            diff(i,j) = this->m_matrix[i][j] - B(i,j);
        }
    }
    
    return diff;
}



Matrix Matrix::operator*(Matrix & B){
    Matrix multip(m_rowSize,B.getCols(),0.0);
    if(m_colSize == B.getRows())
    {
        unsigned i,j,k;
        double temp = 0.0;
        for (i = 0; i < m_rowSize; i++)
        {
            for (j = 0; j < B.getCols(); j++)
            {
                temp = 0.0;
                for (k = 0; k < m_colSize; k++)
                {
                    temp += m_matrix[i][k] * B(k,j);
                }
                multip(i,j) = temp;
                //cout << multip(i,j) << " ";
            }
            //cout << endl;
        }
        return multip;
    }
    else
    {
        std::cout<<"size incompatimble"<<std::endl  ;
    }
}

double& Matrix::operator()(const unsigned &rowNo, const unsigned & colNo)
{
    return this->m_matrix[rowNo][colNo];
}


Matrix Matrix::operator/(double &s){

 Matrix result(m_rowSize,m_colSize,0);
 for(unsigned i{0};i<m_rowSize;i++){
     for(unsigned j{0};j <m_colSize;j++){
         result(i,j) = m_matrix[i][j]/s; 
     }
 }
   

}


unsigned Matrix::getRows() const
{
    return this->m_rowSize;
}

// returns col #
unsigned Matrix::getCols() const
{
    return this->m_colSize;
}

// Take any given matrices transpose and returns another matrix
Matrix Matrix::transpose()
{
    Matrix Transpose(m_colSize,m_rowSize,0.0);
    for (unsigned i = 0; i < m_colSize; i++)
    {
        for (unsigned j = 0; j < m_rowSize; j++) {
            Transpose(i,j) = this->m_matrix[j][i];
        }
    }
    return Transpose;
}

void Matrix::print() const
{
    std::cout << "Matrix: " << std::endl;
    for (unsigned i = 0; i < m_rowSize; i++) {
        for (unsigned j = 0; j < m_colSize; j++) {
            std::cout << "[" << m_matrix[i][j] << "] ";
        }
        std::cout << std::endl;
    }
}

 Matrix& Matrix::identityMatrix()

{
    // if it's not a square matrix, we cant make an identity matrix.
    assert(getCols() == getRows());
    
       
    for (int i=0;i<m_rowSize; i++)
    {
        m_matrix[i][i] = 1;
    }
    
    return *this;
}

Matrix& Matrix::diagonalMatrix(double & value){
    assert(getCols() == getRows());
    
       
    for (int i=0;i<m_rowSize; i++)
    {
        m_matrix[i][i] = value;
    }
    
    return *this;
}




double calculateDeterminant(std::vector<std::vector<double>> mat){
   
   
        //this function is written in c++ to calculate the determinant of matrix
        // it's a recursive function that can handle matrix of any dimension
        float det = 0; // the determinant value will be stored here
        if (mat.size() == 1)  
        {
            return mat[0][0]; // no calculation needed
        }
        else if (mat.size() == 2) //base condition
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

Matrix Matrix::appendColumn(Matrix &b){
    assert(b.getRows()== getRows());
    int cur_cols = getCols();
    Matrix augmented = *this;
    for (int i=0;i<getRows();i++)
    {
        augmented.m_matrix[i].resize(cur_cols + b.getCols());
        for (int j=cur_cols;j<cur_cols + b.getCols();j++)
        {
            augmented(i,j) = b(i,j-cur_cols);
        }
    }
    augmented.m_colSize = getCols()+b.getCols();
    
    return augmented;
}


Matrix Matrix::gaussianJordan()
{
    Matrix b(getRows(),getCols(),0);
    Matrix result(getRows(),getCols(),0);

    b.identityMatrix();
    Matrix augmented = appendColumn(b);

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
    
     for(int i=0;i<getRows();i++)
		 {
			  for(int j=getCols();j< augmented.getCols(); j++)
			  {
			   	augmented(i,j) = augmented(i,j)/augmented(i,i);
			  }
		 } 
  
   
   for(int i = 0;i<getRows();i++)
		 {
			  for(int j=getCols();j< augmented.getCols();j++)
			  {
			   	result(i,j-getCols()) = augmented(i,j); 
			  }
			  
		 }
      
      return result;
}


double Matrix::det()
   {     
        double determinant = calculateDeterminant(this->m_matrix);
        return determinant;
         
    }

Matrix Matrix::inverse(){

     Matrix inv = gaussianJordan();
     return inv;
}

