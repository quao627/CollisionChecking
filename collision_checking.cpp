#include <cmath>
#include <iostream>
#include <vector>
#include <map>

using namespace std;
using std::vector;

const double pi = 3.14159265358979323846;

double** get_coordinates(double x, double y, double l, double w, double theta){
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
    double** results = new double*[4];
    for (size_t i=0; i<4; ++i){
        results[i] = new double[2];
    }
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

map<char, double> get_normal(double* p1, double* p2){
    /*
    this function obtains directional vector between two points

    @param p1: the first point
    @param p2: the second point

    @return: A map representing the normal vector
    */
    
    map<char, double> normal;
    double norm = sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
    normal['x'] = (p2[0] - p1[0]) / norm;
    normal['y'] = (p2[1] - p1[1]) / norm;
    return normal;
}


bool check_collision(vector<double> rec1, vector<double> rec2){
    /*
    this function check if two vehicles collide given their states

    @param rec1: state of the ego vehicle (i.e., x, y, l, w, theta)
    @param rec2: state of the opponent vehicle

    @return: A boolean indicating if a collision happens
    */

    double** pts1 = get_coordinates(rec1[0], rec1[1], rec1[2], rec1[3], rec1[4]);
    double** pts2 = get_coordinates(rec2[0], rec2[1], rec2[2], rec2[3], rec2[4]);
    map<char, double> normals[4];
    normals[0] = get_normal(pts1[0], pts1[1]);
    normals[1] = get_normal(pts1[1], pts1[2]);
    normals[2] = get_normal(pts2[0], pts2[1]);
    normals[3] = get_normal(pts2[1], pts2[2]);
    
    // check if there is overlap between projections on the normal vector
    for (size_t i=0; i<4; ++i){
        map<char, double> tmp = normals[i];
        double min1 = INFINITY;
        double max1 = -INFINITY;
        double min2 = INFINITY;
        double max2 = -INFINITY;

        // get all projections for corners of rec1
        for (size_t j=0; j<4; ++j){
            double projected = tmp['x'] * pts1[j][0] + tmp['y'] * pts1[j][1];
            if ((!min1) || (projected < min1)){
                min1 = projected;
            }
            if ((!max1) || (projected > max1)){
                max1 = projected;
            }
        }

        // get all projections for corners of rec2
        for (size_t j=0; j<4; ++j){
            double projected = tmp['x'] * pts2[j][0] + tmp['y'] * pts2[j][1];
            if ((!min2) || (projected < min2)){
                min2 = projected;
            }
            if ((!max2) || (projected > max2)){
                max2 = projected;
            }
        }

        if (max1 < min2 || max2 < min1) return false;
    }
    return true;
}

int main() {
    // an example
    static const double arr1[] = {0, 0, 1, 1, 45};
    static const double arr2[] = {-1.5, 0, 3, 1, 130};
    vector<double> rec1 (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]));
    vector<double> rec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]));
    std::cout << check_collision(rec1, rec2);

}