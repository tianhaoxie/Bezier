#include <Eigen/Core>
#include <vector>
#include <fstream>
using namespace Eigen;
using namespace std;
class bezier
{
private:
    Vector3d deCasteljau(const vector<Vector3d>& C,double t);
    void find_combination(VectorXd& x,vector<double>& data,vector<vector<double>>&results, int start, int end, int index, int r);
    vector<Vector3d> slicing(const vector<Vector3d> ori, int start, int end);
public:
    bezier();
    ~bezier();
    vector<vector<Vector3d>> gen_tp_surface(vector<vector<Vector3d>> &Cs, int resolution);
    vector<vector<Vector3d>> gen_control();
    Vector3d blossoming(const vector<Vector3d> & Cs, double t1, double t2, double t3);
    MatrixXd sample_blossoming(const vector<vector<Vector3d>>& Cs, int sample_res_t , int sample_res_u,double t_min = 0, double t_max=1);
    void gen_mesh(MatrixXd& V,MatrixXi& F, const vector<vector<Vector3d>>& tp_surface);
    void convert2matrix(const vector<vector<Vector3d>>&pts, MatrixXd& V);
    void convert2matrix(const vector<Vector3d>&pts, MatrixXd& V);
    void convert2vector(vector<vector<Vector3d>>&pts,const MatrixXd& V);
    void read_control(const string path, vector<MatrixXd>& patches);
    void read_control(const string path, MatrixXd& patch);
    void read_target(const string path, MatrixXd& PC);
};
