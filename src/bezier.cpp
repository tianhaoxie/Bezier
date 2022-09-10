#include "bezier.h"
using namespace Eigen;
using namespace std;


bezier::bezier(){}
bezier::~bezier(){}


vector<Vector3d> bezier::slicing(const vector<Vector3d> ori, int start, int end){
    vector<Vector3d> sliced;
    for (int i=start; i<end;i++){
        sliced.push_back(ori[i]);
    }

    return sliced;
}

Vector3d bezier::deCasteljau(const vector<Vector3d>& C,double t){
    if (C.size() !=2){
            Vector3d a = deCasteljau(slicing(C,0,C.size()-1),t);
            Vector3d b = deCasteljau(slicing(C,1,C.size()),t);
            Vector3d r = (1-t)*a+t*b;
            return r;
    }
    else{
        Vector3d r = C[0]*(1-t)+C[1]*t;
        return r;
    }
}

vector<vector<Vector3d>> bezier::gen_tp_surface(vector<vector<Vector3d>> & Cs, int resolution){
    vector<Vector3d> temp(resolution);
    vector<vector<Vector3d>> pts(resolution);
    vector<Vector3d> pts_u;
    VectorXd u = VectorXd::LinSpaced(resolution,0.0,1.0);
    VectorXd v = VectorXd::LinSpaced(resolution,0.0,1.0);
    for (int idx_u=0;idx_u<u.size();idx_u++){
        for (int i=0;i<Cs.size();i++){
            pts_u.push_back(deCasteljau(Cs[i],u(idx_u)));
        }
        for (int idx_v=0;idx_v<v.size();idx_v++){
            temp[idx_v]=deCasteljau(pts_u,v(idx_v));
        }
        pts_u.clear();
        pts[idx_u] = temp;
    }

    return pts;

}

