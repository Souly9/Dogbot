// Utility class for stuff
#pragma once
#include <math.h>
#include <Aria.h>
#define PI 3.14159265359
// Utility vector class, mainly to make working with coordinates easier than with ArPoses 
struct CVector
{
    CVector()
    {
    }
    CVector(double val1, double val2)
        : x(val1), y(val2){}
    ~CVector() {}

    // Operator overloading
    CVector operator+(CVector vec)
    {
        CVector tmp;
        tmp.x += vec.x;
        tmp.y += vec.y;
        return tmp;
    }
    CVector operator-(CVector vec)
    {
        CVector tmp;
        tmp.x -= vec.x;
        tmp.y -= vec.y;
        return tmp;
    }
    CVector operator*(double num)
    {
        CVector tmp;
        tmp.x *= num;
        tmp.y *= num;
        return tmp;
    }
    bool operator==(double val) 
    {
        return x == val && y == val;
    }
    bool operator!=(double val) 
    {
        return x != val && y != val;
    }

    // Helper functions
    double length()
    {
        return sqrt(x * x + y * y);
    }

    double length(CVector x2)
    {
        return sqrt((x - x2.x) * (x - x2.x) + (y - x2.y) * (y - x2.y));
    }

    double dot(CVector vec2) 
    {
        return x * vec2.x + y * vec2.y;
    }

    CVector normalize() 
    {
        double length = this->length();
        CVector norm(x / length, y / length);
        return norm;
    }
    // Members are public to make access easier
    double x;
    double y;
};

enum HerrchenState
{
  herrchenLocalizing,
  herrchenFound,
  herrchenTracking,
  disabled
};
