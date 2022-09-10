#include "src/bezier.h"
#include <Eigen/Core>
#include <iostream>
#include <igl/writeOBJ.h>

using namespace Eigen;
vector<vector<Vector3d>> gen_control(){
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
                temp(2)=-1;
            }
            c_y.push_back(temp);
        }
        control_points.push_back(c_y);
        c_y.clear();
    }

    return control_points;
}

void gen_mesh(MatrixXd& V,MatrixXi& F, const vector<vector<Vector3d>>& tp_surface){
    V = MatrixXd::Zero(tp_surface.size()*tp_surface[0].size(),3);
    int faceNum = 2*(tp_surface.size()-1)*(tp_surface[0].size()-1);
    F = MatrixXi::Zero(faceNum,3);
    Vector3d temp;
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
            F(f_num,1) = tp_surface.size()*(i+1)+j;
            F(f_num,2) = tp_surface.size()*(i+1)+j+1;
            f_num++;
            F(f_num,0) = tp_surface.size()*i+j;
            F(f_num,1) = tp_surface.size()*i+j+1;
            F(f_num,2) = tp_surface.size()*(i+1)+j+1;
            f_num++;
        }
    }
}
int main(int argc, char* argv[]){
    vector<vector<Vector3d>> control_points;
    vector<vector<Vector3d>> sample;
    control_points = gen_control();
    bezier*  b = new bezier();
    sample = b -> gen_tp_surface(control_points,20);
    MatrixXd V;
    MatrixXi F;
    gen_mesh(V,F,sample);
    igl::writeOBJ("test.obj",V,F);

}
