/*
Todo:
1, coordinate system for distance calculation
2, how to get nearest boundary segments
3,assumptions reasonable?
*/


#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <vector>
#include "boundary_checking.h"

using namespace std;

const double pi = 3.14159265358979323846;

CollisionChecker::CollisionChecker(string file1, string file2)
{
    /*
    this function initializes the class with the boundary coordinates

    @param file1: path to the file containing coordinates of the inside boundary
    @param file2: path to the file containing coordinates of the outside boundary
    */
    inside = parseCSV(file1);
    outside = parseCSV(file2);
    searchIdx = 0;
}

vector<vector<double>> CollisionChecker::get_coordinates(double x, double y, double l, double w, double theta)
{
    /*
    this function finds coordinates of the corners of a vehicle given its state

    @param x: x coordinate of the vehicle
    @param y: y coordinate of the vehicle
    @param l: length of the vehicle
    @param w: width of the vehicle
    @param theta: angle the vehicle is heading towards 
    @return: A 2d array containing coordinates of all four corners
    */
    l = l / 2;
    w = w / 2;
    double rad = theta * pi / 180;
    double c = cos(rad);
    double s = sin(rad);
    vector<vector<double> > results(4, vector<double>(2));
    results[0][0] = x+c*l-s*w;
    results[0][1] = y+s*l+c*w;
    results[1][0] = x-c*l-s*w;
    results[1][1] = y-s*l+c*w;
    results[2][0] = x-c*l+s*w;
    results[2][1] = y-s*l-c*w;
    results[3][0] = x+c*l+s*w;
    results[3][1] = y+s*l-c*w;
    return results;
}

vector<vector<double> > CollisionChecker::parseCSV(string filename){
    std::ifstream data;
    data.open(filename);
    string line, val;
    vector<vector<double> > parsedCsv;
    int k = 0;
    while(getline(data, line)){
        if (k < 4){
            k++;
            continue;
        }
        vector<double> v;
        stringstream lineStream(line);
        size_t cnt = 0;
        while (getline(lineStream, val, ',')){
            v.push_back(stod(val));
            cnt += 1;
            if (cnt==2) break;
        }
        parsedCsv.push_back(v);
    }

    return parsedCsv;
}

double CollisionChecker::dist(vector<double> v1, vector<double> v2){
    return sqrt(pow(v1[0] - v2[0], 2) + pow(v1[1] - v2[1], 2));
}

size_t CollisionChecker::find_nearest(vector<double> ego, vector<vector<double> > lines){
    size_t index = -1;
    double dMin = INFINITY;
    double lastDist = INFINITY;
    size_t i = searchIdx;
    while (true) {
        double d = dist(ego, lines[i]);
        if (d < dMin) {
            dMin = d;
            index = i;
        }
        if (d > lastDist) {
            searchIdx = index;
            break;
        }
        lastDist = d;
        i = (i + 1) % lines.size();

    }
    return index;
}

bool CollisionChecker::check_boundary_collision(vector<double> egoVeh){
    vector<double> ego = {egoVeh[0], egoVeh[1]};
    size_t insideIdx = find_nearest(ego, inside);
    size_t outsideIdx = find_nearest(ego, outside);
    
    vector<vector<double> > v1, v2;
    v1 = {inside[insideIdx-1], inside[insideIdx]};
    v2 = {outside[outsideIdx-1], outside[outsideIdx]};
    bool t1 = check_segments(egoVeh, v1, v2);

    v1 = {inside[insideIdx], inside[insideIdx+1]};
    v2 = {outside[outsideIdx], outside[outsideIdx+1]};
    bool t2 = check_segments(egoVeh, v1, v2);

    return (t1 && t2);
}

bool CollisionChecker::check_segments(vector<double> egoVeh, vector<vector<double> > v1, vector<vector<double> > v2) {
    double m1, n1, a1, b1, c1;
    m1 = (v1[1][1] - v1[0][1]) / (v1[1][0] - v1[0][0]);
    n1 = v1[0][1] - m1 * v1[0][0];
    a1 = m1;
    b1 = -1;
    c1 = n1;

    double m2, n2, a2, b2, c2;
    m2 = (v2[1][1] - v2[0][1]) / (v2[1][0] - v2[0][0]);
    n2 = v2[0][1] - m2 * v2[0][0];
    a2 = m2;
    b2 = -1;
    c2 = n2;

    vector<vector<double> > pts = get_coordinates(egoVeh[0], egoVeh[1], egoVeh[2], egoVeh[3], egoVeh[4]);
    bool sign;
    
    for (size_t i=0; i<4; ++i){
        bool new_sign = (pts[i][0] * a1 + pts[i][1] * b1 + c1 > 0);
        if (i==0) sign = new_sign;
        if (new_sign!=sign) return false;
    }

    for (size_t i=0; i<4; ++i){
        bool new_sign = (pts[i][0] * a2 + pts[i][1] * b2 + c2 > 0);
        if (i==0) sign = new_sign;
        if (new_sign!=sign) return false;
    }

    return true;
}