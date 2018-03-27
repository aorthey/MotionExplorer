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

namespace Eigen {

std::atomic_int Polyhedron::counter(0);
std::mutex Polyhedron::mtx;

Polyhedron::Polyhedron()
    : matPtr_(nullptr)
    , polytope_(nullptr)
{
    if (counter == 0)
        dd_set_global_constants();
    counter++;
}

Polyhedron::~Polyhedron()
{
    counter--;

    if (matPtr_ != nullptr)
        dd_FreeMatrix(matPtr_);
    if (polytope_ != nullptr)
        dd_FreePolyhedra(polytope_);

    if (counter == 0)
        dd_free_global_constants();
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
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    return ddfMatrix2EigenMatrix(mat, true);
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::hrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    return ddfMatrix2EigenMatrix(mat, false);
}

void Polyhedron::printVrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    dd_WriteMatrix(stdout, mat);
}

void Polyhedron::printHrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    dd_WriteMatrix(stdout, mat);
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
        dd_FreeMatrix(matPtr_);
    
    matPtr_ = dd_CreateMatrix(rows, cols);
    matPtr_->representation = (isFromGenerators ? dd_Generator : dd_Inequality);
}

bool Polyhedron::doubleDescription(const Eigen::MatrixXd& matrix, bool isFromGenerators)
{
    initializeMatrixPtr(matrix.rows(), matrix.cols(), isFromGenerators);

    for (auto row = 0; row < matrix.rows(); ++row)
        for (auto col = 0; col < matrix.cols(); ++col)
            matPtr_->matrix[row][col][0] = matrix(row, col);

    if (polytope_ != nullptr)
        dd_FreePolyhedra(polytope_);

    polytope_ = dd_DDMatrix2Poly(matPtr_, &err_);
    return (err_ == dd_NoError) ? true : false;
}

Eigen::MatrixXd Polyhedron::concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool isFromGenerators)
{
    double sign = (isFromGenerators ? 1 : -1);
    Eigen::MatrixXd mat(A.rows(), A.cols() + 1);
    mat.col(0) = b;
    mat.block(0, 1, A.rows(), A.cols()).noalias() = sign * A;
    return mat;
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::ddfMatrix2EigenMatrix(const dd_MatrixPtr mat, bool isOuputVRep) const
{
    double sign = (isOuputVRep ? 1 : -1);
    auto rows = mat->rowsize;
    auto cols = mat->colsize;
    Eigen::MatrixXd mOut(rows, cols - 1);
    Eigen::VectorXd vOut(rows);
    for (auto row = 0; row < rows; ++row) {
        vOut(row) = mat->matrix[row][0][0];
        for (auto col = 1; col < cols; ++col)
            mOut(row, col - 1) = sign * mat->matrix[row][col][0];
    }

    return std::make_pair(mOut, vOut);
}

} // namespace Eigen
