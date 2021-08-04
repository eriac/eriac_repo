#include <iostream>        // standard input/output
#include <vector>          // standard vector
#include <cppad/cppad.hpp> // the CppAD package

int main(void)
{
    using CppAD::AD;   // use AD as abbreviation for CppAD::AD
    using std::vector; // use vector as abbreviation for std::vector

    vector<AD<double>> ax(2);
    CppAD::Independent(ax);
    vector<AD<double>> ay(2);
    ay[0] = CppAD::cos(ax[0]) - CppAD::sin(ax[1]);
    ay[1] = CppAD::sin(ax[0]) + CppAD::cos(ax[1]);
    CppAD::ADFun<double> f(ax, ay);

    vector<double> x(2);
    x[0] = 0;
    x[1] = 0;
 
    vector<double> jac;
    jac = f.Jacobian(x);
    vector<double> fw0;
    fw0 = f.Forward(0, x);
    vector<double> fw1;
    fw1 = f.Forward(1, x);

    std::cout << "xsize:" << x.size() << ", jsize:" << jac.size() << std::endl;

    std::cout<< "x: ";
    for (auto item : x)std::cout<< item << ", ";
    std::cout << std::endl;

    std::cout<< "fw0: ";
    for (auto item : fw0)std::cout<< item << ", ";
    std::cout << std::endl;

    std::cout<< "fw1: ";
    for (auto item : fw1)std::cout<< item << ", ";
    std::cout << std::endl;

    std::cout<< "j: ";
    for (auto item : jac)std::cout<< item << ", ";
    std::cout << std::endl;
}

// https://qiita.com/RyosukeH/items/b947efa9dbcefa396baa
// https://coin-or.github.io/CppAD/doc/example.htm