/*      File: Polyhedron.cpp
*       This file is part of the program eigen-cdd
*       Program description : Small Eigen Wrapper of cdd.
*       Copyright (C) 2017 -  vsamy (LIRMM). All Right reserved.
*
*       This software is free software: you can redistribute it and/or modify
*       it under the terms of the CeCILL-C license as published by
*       the CEA CNRS INRIA, either version 1
*       of the License, or (at your option) any later version.
*       This software is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*       CeCILL-C License for more details.
*
*       You should have received a copy of the CeCILL-C License
*       along with this software. If not, it can be found on the official website
*       of the CeCILL licenses family (http://www.cecill.info/index.en.html).
*/

#include "Polyhedron.h"
#include <iostream>

namespace Eigen {

std::atomic_int Polyhedron::counter(0);
std::mutex Polyhedron::mtx;

Polyhedron::Polyhedron()
    : matPtr_(nullptr)
    , polytope_(nullptr)
{
    if (counter == 0)
        ddf_set_global_constants();
    counter++;
}

Polyhedron::~Polyhedron()
{
    counter--;

    if (matPtr_ != nullptr)
        ddf_FreeMatrix(matPtr_);
    if (polytope_ != nullptr)
        ddf_FreePolyhedra(polytope_);

    if (counter == 0)
        ddf_free_global_constants();
}

void Polyhedron::vrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    std::unique_lock<std::mutex> lock(mtx);
    if (!hvrep(A, b, false))
        throw std::runtime_error("Bad conversion from hrep to vrep.");
}

void Polyhedron::hrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    std::unique_lock<std::mutex> lock(mtx);
    if (!hvrep(A, b, true))
        throw std::runtime_error("Bad conversion from vrep to hrep.");
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::vrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    ddf_MatrixPtr mat = ddf_CopyGenerators(polytope_);
    return ddfMatrix2EigenMatrix(mat, true);
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::hrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    ddf_MatrixPtr mat = ddf_CopyInequalities(polytope_);
    return ddfMatrix2EigenMatrix(mat, false);
}

void Polyhedron::printVrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    ddf_MatrixPtr mat = ddf_CopyGenerators(polytope_);
    ddf_WriteMatrix(stdout, mat);
}

void Polyhedron::printHrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    ddf_MatrixPtr mat = ddf_CopyInequalities(polytope_);
    ddf_WriteMatrix(stdout, mat);
}

/**
 * Private functions
 */

bool Polyhedron::hvrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool isFromGenerators)
{
    Eigen::MatrixXd cMat = concatenateMatrix(A, b, isFromGenerators);
    return doubleDescription(cMat, isFromGenerators);
}

void Polyhedron::initializeMatrixPtr(Eigen::Index rows, Eigen::Index cols, bool isFromGenerators)
{
    if (matPtr_ != nullptr)
        ddf_FreeMatrix(matPtr_);
    
    matPtr_ = ddf_CreateMatrix(rows, cols);
    matPtr_->representation = (isFromGenerators ? ddf_Generator : ddf_Inequality);
}

bool Polyhedron::doubleDescription(const Eigen::MatrixXd& matrix, bool isFromGenerators)
{
    initializeMatrixPtr(matrix.rows(), matrix.cols(), isFromGenerators);

    //std::cout << "matrix size: " << matPtr_->rowsize << "x" << matPtr_->colsize << std::endl;

    for (auto row = 0; row < matrix.rows(); ++row){
      for (auto col = 0; col < matrix.cols(); ++col){
        //double d = 0;
#ifdef ddf_GMPRATIONAL
        mpq_set_d(&matPtr_->matrix[row][col][0], matrix(row, col));
        //d = mpq_get_d(&matPtr_->matrix[row][col][0]);
#else
        matPtr_->matrix[row][col][0] = matrix(row, col);
        //d = matPtr_->matrix[row][col][0];
#endif
        // std::cout << "[" << row << "/" << matrix.rows() << "," << col << "/" << matrix.cols()<< "] : ";
        // std::cout << matrix(row,col) << " -> ";
        // std::cout << d;
        // std::cout << std::endl;
      }
    }

    if (polytope_ != nullptr)
        ddf_FreePolyhedra(polytope_);

    polytope_ = ddf_DDMatrix2Poly(matPtr_, &err_);
    return (err_ == ddf_NoError) ? true : false;
}

Eigen::MatrixXd Polyhedron::concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool isFromGenerators)
{
    double sign = (isFromGenerators ? 1 : -1);
    Eigen::MatrixXd mat(A.rows(), A.cols() + 1);
    mat.col(0) = b;
    mat.block(0, 1, A.rows(), A.cols()).noalias() = sign * A;
    return mat;
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::ddfMatrix2EigenMatrix(const ddf_MatrixPtr mat, bool isOuputVRep) const
{
    double sign = (isOuputVRep ? 1 : -1);
    auto rows = mat->rowsize;
    auto cols = mat->colsize;
    Eigen::MatrixXd mOut(rows, cols - 1);
    Eigen::VectorXd vOut(rows);
    for (auto row = 0; row < rows; ++row) {
#ifdef ddf_GMPRATIONAL
        vOut(row) = mpq_get_d(&mat->matrix[row][0][0]);
        for (auto col = 1; col < cols; ++col){
          mOut(row, col-1) = sign*mpq_get_d(&mat->matrix[row][col][0]);
        }
#else
        vOut(row) = mat->matrix[row][0][0];
        for (auto col = 1; col < cols; ++col){
          mOut(row, col - 1) = sign * mat->matrix[row][col][0];
        }
#endif
    }

    return std::make_pair(mOut, vOut);
}

} // namespace Eigen
