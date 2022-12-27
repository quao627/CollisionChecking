#include <vector>
#include <string>

using namespace std;
class CollisionChecker {
    private:
        vector<vector<double>> inside;
        vector<vector<double>> outside;
        size_t searchIdx;

    public:
        CollisionChecker(string file1, string file2);
        
        vector<vector<double>> get_coordinates(double x, double y, double l, double w, double theta);
        
        vector<vector<double>> parseCSV(string filename);
        
        size_t find_nearest(vector<double> ego, vector<vector<double> > lines);
        
        // call this function to check if the ego vehicle is colliding with the boundary
        bool check_boundary_collision(vector<double> egoVeh);
        
        bool check_segments(vector<double> egoVeh, vector<vector<double> > v1, vector<vector<double> > v2);
        
        double dist(vector<double> v1, vector<double> v2);
};