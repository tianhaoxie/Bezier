#include "bezier.h"
using namespace Eigen;
using namespace std;


bezier::bezier()= default;
bezier::~bezier()= default;


vector<Vector3d> bezier::slicing(const vector<Vector3d> ori, int start, int end){
    vector<Vector3d> sliced;
    for (int i=start; i<end;i++){
        sliced.push_back(ori[i]);
    }

    return sliced;
}

void bezier::find_combination(VectorXd& x, vector<double>& data,vector<vector<double>>& results, int start, int end, int index, int r){
    if (index == r){
        results.push_back(data);
        return;
    }
    for (int i = start; i<=end && end-i+1>=r-index; i++){
        data[index] = x(i);
        find_combination(x,data,results,i+1,end,index+1,r);
    }
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

Vector3d bezier::blossoming(const vector<Vector3d> & Cs, double t1, double t2, double t3){

    Vector3d r0_t3,r1_t3,r2_t3,r0_t2,r1_t2,r;
    r0_t3 = (1-t3) * Cs[0] + t3 * Cs[1];
    r1_t3 = (1-t3) * Cs[1] + t3 * Cs[2];
    r2_t3 = (1-t3) * Cs[2] + t3 * Cs[3];
    r0_t2 = (1-t2) * r0_t3 + t2 * r1_t3;
    r1_t2 = (1-t2) * r1_t3 + t2 * r2_t3;
    r = (1-t1) * r0_t2 + t1 * r1_t2;

    return r;
}

MatrixXd bezier::sample_blossoming(const vector<vector<Vector3d>>& Cs, int sample_res_t, int sample_res_u, double t_min, double t_max){
    VectorXd t = VectorXd::LinSpaced(sample_res_t,t_min,t_max);
    VectorXd u = VectorXd::LinSpaced(sample_res_u,0.0,1.0);
    vector<vector<double>> combinations;
    vector<double> data(3);
    vector<Vector3d> control_u(Cs.size());
    find_combination(t,data,combinations,0,t.size()-1,0,3);
    MatrixXd pts = MatrixXd::Zero(combinations.size()*sample_res_u,3);
    for (int i=0;i<sample_res_u;i++){
        for (int j=0;j<Cs.size();j++){
            control_u[j] = deCasteljau(Cs[j],u(i));
        }
        for (int j=0;j<combinations.size();j++){
            pts.row(i*combinations.size()+j) = blossoming(control_u,combinations[j][0],combinations[j][1],combinations[j][2]);
        }

    }
    return pts;
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

void bezier::read_control(const string path, vector<MatrixXd>& patches){

    ifstream fin(path);
    string line;
    string word;
    Vector3d tmp;
    vector<Vector3d> patch;
    MatrixXd V;
    int l_num = -1;
    while (getline(fin,line)){
        if (l_num ==-1){
            l_num++;
            continue;
        }
        
        stringstream str(line);
        for (int i=0;i<3;i++) {
            getline(str,word,',');
            tmp(i) = stod(word);
        }
        patch.push_back(tmp);
        if ((l_num+1) %16 == 0){
            convert2matrix(patch,V);
            patches.push_back(V);
            patch.clear();
        }
        l_num++;
    }
}

void bezier::read_control(const string path, MatrixXd& patch){
    ifstream fin(path);
    string line;
    string word;
    Vector3d tmp;
    vector<Vector3d> patch_tmp;
    while (getline(fin,line)){
    
        stringstream str(line);
        for (int i=0;i<3;i++) {
            getline(str,word,',');
            tmp(i) = stod(word);
        }
        patch_tmp.push_back(tmp);
    
    }
    convert2matrix(patch_tmp,patch);
}

void bezier::read_target(const string path, MatrixXd& PC){
    ifstream fin(path);
    string line;
    string word;
    Vector3d tmp;
    vector<Vector3d> patch_tmp;
    int l_num = 0;
    while (getline(fin,line)){
        if (l_num ==0){
            l_num++;
            continue;
        }
        stringstream str(line);
        for (int i=0;i<3;i++) {
            getline(str,word,',');
            tmp(i) = stod(word);
        }
        patch_tmp.push_back(tmp);
    
    }
    convert2matrix(patch_tmp,PC);
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

void bezier::convert2matrix(const vector<Vector3d>&pts, MatrixXd& V){
    V = MatrixXd::Zero(pts.size(),3);
    for (int i=0 ; i<pts.size() ; i++){
    
        V(i,0) = pts[i](0);
        V(i,1) = pts[i](1);
        V(i,2) = pts[i](2);
        
    }
}

void bezier::convert2vector(vector<vector<Vector3d>>& patch,const MatrixXd& V){
    
    if (patch.size() == 0){
        Vector3d pt;
        vector<Vector3d> pts; 
        for (int i=0 ; i<4 ; i++){
            for (int j=0 ; j<4 ; j++){
                pt = V.row(i*4+j);
                pts.push_back(pt);
            }
            patch.push_back(pts);
            pts.clear();
        }
    }

    else{
        for (int i=0 ; i<4 ; i++){
            for (int j=0 ; j<4 ; j++){
                patch[i][j](0) = V(i*patch.size()+j,0);
                patch[i][j](1) = V(i*patch.size()+j,1);
                patch[i][j](2) = V(i*patch.size()+j,2);
        }
    }
    }
    
}

