#ifndef MATRIX_H_
#define MATRIX_H_ MATRIX_H_

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <cassert>  

using std::vector;

template<typename matrixType>
class Matrix {

    public:
    Matrix();

    /**
     * Initialization of matrix with the given rows and columns with the given value. 
    */
    Matrix(unsigned, unsigned, matrixType);

   /**
    * Initialization of matrix from a 2d vector.
   */
    Matrix(std::vector<std::vector<matrixType> >& a);
    

    // returns number of rows #
    unsigned getRows() const;
    
    // returns number of cols #
    unsigned getCols() const;

    // adds two matrices and returns the resulting matrix object #
    Matrix operator+(Matrix &);
    
   //subtract two matrices and returns the resulting Matrix object.
    Matrix operator-(Matrix &);
    

  //  Multiplies two matrices and returns the resulting Matrix object.
    Matrix operator*(Matrix &);
    

  /**    Division of the matrix with the scalar value and returns the resulting Matrix object.*/
    Matrix operator/(double &);

   // data accessing operator . accessed by a(row,col).
    double& operator()(const unsigned &, const unsigned &);

  /** Returns a Matrix object that is the transpose of the current Matrix. */
    Matrix transpose();
    /** Makes this matrix an identity matrix. Throws an error if 
        this is not a square matrix. */
    Matrix& createIdentityMatrix();

    /** Makes this matrix a diagonal matrix. */
    Matrix& createDiagonalMatrix(const matrixType );
   
   /**Returns the matrix object which is the inverse of the given matrix.*/
    Matrix inverse();
    
    /**GaussianJordan elimination method to find the inverse of the given matrix. 
     * Converts the given matrix into row echloen form.
     * Steps explained in the .cpp file
      */
    Matrix gaussianJordan();


     /** Adds a column to the current Matrix object.        
           | a b |          |e|     |a b e |
           | c d | appends  |f|   = |c d f |
                   */
    Matrix appendColumn(Matrix &);
    
    /**  returns the determinant of the given matrix.   */
    double det();
    
    /** print the elements in the matrix.*/
    void print() const;
    
    
   
    private:
    unsigned _m_rowSize;
    unsigned _m_colSize;
    vector<vector<matrixType> > _m_matrix;

};

template<typename matrixType>
double calculateDeterminant(std::vector<std::vector<matrixType>> mat);




#include"matrixcopy.hpp"
#endif

