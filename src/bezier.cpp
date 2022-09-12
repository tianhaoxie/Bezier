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



vector<vector<Vector3d>> bezier::gen_control(){
    VectorXd x = VectorXd::LinSpaced(4,-1.0,1.0);
    VectorXd y = VectorXd::LinSpaced(4,-1.0,1.0);
    vector<vector<Vector3d>> control_points;
    vector<Vector3d> c_y;
    Vector3d temp;
    for (int i=0; i<4; i++){
        for (int j=0;j<4; j++){
            temp(0)=x[i];
            temp(1)=y[j];
            if ( (i==1 || i==2) && (j==1 || j==2)){
                temp(2)=1;
            }
            else{
                temp(2)=0;
            }
            c_y.push_back(temp);
        }
        control_points.push_back(c_y);
        c_y.clear();
    }

    return control_points;
}

void bezier::gen_mesh(MatrixXd& V,MatrixXi& F, const vector<vector<Vector3d>>& tp_surface){
    V = MatrixXd::Zero(tp_surface.size()*tp_surface[0].size(),3);
    int faceNum = 2*(tp_surface.size()-1)*(tp_surface[0].size()-1);
    F = MatrixXi::Zero(faceNum,3);
    for (int i=0 ; i<tp_surface.size() ; i++){
        for (int j=0 ; j<tp_surface[0].size() ; j++){
            V(i*tp_surface.size()+j,0) = tp_surface[i][j](0);
            V(i*tp_surface.size()+j,1) = tp_surface[i][j](1);
            V(i*tp_surface.size()+j,2) = tp_surface[i][j](2);
        }
    }
    int f_num = 0;
    for (int i=0 ; i<tp_surface.size()-1;i++){
        for (int j=0 ; j<tp_surface[0].size()-1 ; j++){
            F(f_num,0) = tp_surface.size()*i+j;
            F(f_num,1) = tp_surface.size()*(i+1)+j+1;
            F(f_num,2) = tp_surface.size()*(i+1)+j;
            f_num++;
            F(f_num,0) = tp_surface.size()*i+j;
            F(f_num,1) = tp_surface.size()*i+j+1;
            F(f_num,2) = tp_surface.size()*(i+1)+j+1;
            f_num++;
        }
    }
}

void bezier::convert2matrix(const vector<vector<Vector3d>>&pts, MatrixXd& V){
    V = MatrixXd::Zero(pts.size()*pts[0].size(),3);
    for (int i=0 ; i<pts.size() ; i++){
        for (int j=0 ; j<pts[0].size() ; j++){
            V(i*pts.size()+j,0) = pts[i][j](0);
            V(i*pts.size()+j,1) = pts[i][j](1);
            V(i*pts.size()+j,2) = pts[i][j](2);
        }
    }
}

void bezier::convert2vector(vector<vector<Vector3d>>&pts,const MatrixXd& V){
    for (int i=0 ; i<pts.size() ; i++){
        for (int j=0 ; j<pts[0].size() ; j++){
            pts[i][j](0) = V(i*pts.size()+j,0);
            pts[i][j](1) = V(i*pts.size()+j,1);
            pts[i][j](2) = V(i*pts.size()+j,2);
        }
    }
}

