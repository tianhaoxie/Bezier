#include <Eigen/Core>
#include <vector>
using namespace Eigen;
using namespace std;
class bezier
{
private:
    Vector3d deCasteljau(const vector<Vector3d>& C,double t);
    vector<Vector3d> slicing(const vector<Vector3d> ori, int start, int end);
public:
    bezier();
    ~bezier();
    vector<vector<Vector3d>> gen_tp_surface(vector<vector<Vector3d>> &Cs, int resolution);
};
