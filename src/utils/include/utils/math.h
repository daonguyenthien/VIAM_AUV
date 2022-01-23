#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <complex>
#include <iostream>
#include <string>

using namespace std;

inline void convert_string_to_complex(const string& str, complex<double>& num)
{
  string real, imag;
  for (int i = str.size() - 2; i >= 0; i--)
  {
    if (str.data()[i] != '+' && str.data()[i] != '-')
      continue;
    imag = str.substr(i, str.size() - i - 1);
    real = str.substr(0, i);
    break;
  }
  num = complex<double>(stod(real), stod(imag));
}

inline void convert_string_to_complex(const string& str, complex<float>& num)
{
  string real, imag;
  for (int i = str.size() - 2; i >= 0; i--)
  {
    if (str.data()[i] != '+' && str.data()[i] != '-')
      continue;
    imag = str.substr(i, str.size() - i - 1);
    real = str.substr(0, i);
    break;
  }
  num = complex<float>(stof(real), stof(imag));
}

#endif // UTILS_MATH_H
