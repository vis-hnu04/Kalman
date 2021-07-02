#ifndef MATRIX_H_
#define MATRIX_H_ MATRIX_H_



#include <stdio.h>
#include <fstream> // for file access
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include<memory>
#include <cassert>  

using std::vector;
class Matrix {

    public:
    Matrix(unsigned, unsigned, double);
    Matrix(std::vector<std::vector<double> >& a);
    
    unsigned getRows() const;
    unsigned getCols() const;

    
    Matrix operator+(Matrix &);
    Matrix operator-(Matrix &);
    Matrix operator*(Matrix &);
    Matrix operator/(double &);
    Matrix transpose();
    Matrix& createIdentityMatrix();
    Matrix& createDiagonalMatrix(double &);
   
    Matrix inverse();
    Matrix gaussianJordan();
    
    Matrix appendColumn(Matrix &);
    double det();

    double& operator()(const unsigned &, const unsigned &);
    void print() const;
    
    
   
    private:
    unsigned m_rowSize;
    unsigned m_colSize;
    vector<vector<double> > m_matrix;

};

#endif