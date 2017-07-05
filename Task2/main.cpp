// Определить шаблонный класс матриц NxN и реализовать для него функция вычисления определителя матрицы

#include <iostream>
using namespace std;

template <class Type>
class Matrix {
    Type **m_matrix;
    size_t m_size;

public:
    Matrix(size_t size);
    virtual ~Matrix();
    Type* operator[] (size_t index);
    Type determinant();
};

template <class Type>
Matrix<Type>::Matrix(size_t size) {
    m_size = size;
    m_matrix = new Type*[size];
    for (size_t i = 0; i < size; ++i) {
        m_matrix[i] = new Type[size];
    }
}

template <class Type>
Matrix<Type>::~Matrix() {
    for (size_t i = 0; i < m_size; ++i) {
        delete[] m_matrix[i];
    }
    delete[] m_matrix;
}

template <class Type>
Type *Matrix<Type>::operator[](size_t index) {
    return m_matrix[index];
}

template <class Type>
Type Matrix<Type>::determinant() {
    Type det = 0;
    for (size_t i = 0; i < m_size; ++i) {
        Type mul = 1;
        for (size_t j = 0; j < m_size; ++j) {
            mul *= m_matrix[ (i + j) % m_size ][ j ];
        }
        det += mul;
        mul = -1;
        for (size_t j = 0; j < m_size; ++j) {
            mul *= m_matrix[ (i + j) % m_size ][ m_size - j - 1 ];
        }
        det += mul;
    }
    return det;
}


int main() {
    size_t size;
    cin >> size;
    Matrix<double> matrix(size);
    for (size_t i = 0; i < size; ++i) {
        for (size_t j = 0; j < size; ++j) {
            cin >> matrix[i][j];
        }
    }
    cout << matrix.determinant();
    return 0;
}