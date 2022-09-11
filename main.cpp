#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "src/bezier.h"

using namespace std;
using namespace Eigen;
int main(int argc, char *argv[])
{

  vector<vector<Vector3d>> control_points;
  vector<vector<Vector3d>> sample;
  bezier*  s = new bezier();

  control_points = s -> gen_control();
  sample = s -> gen_tp_surface(control_points,20);
  MatrixXd V;
  MatrixXi F;
  MatrixXd CP_matrix;
  s -> gen_mesh(V,F,sample);
  s ->convert2matrix(control_points,CP_matrix);
  // Init the viewer
  igl::opengl::glfw::Viewer viewer;

  // Attach a menu plugin
  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  viewer.plugins.push_back(&plugin);
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  plugin.widgets.push_back(&menu);
  RowVector3f last_mouse;
  long sel = -1;
  // Customize the menu
  int pointSize = 5; 
  double r = 1.0;
  double g,b = 0.0; 
  // Add content to the default menu window


  menu.callback_draw_viewer_menu = [&]()
  {
    // Draw parent menu content
    menu.draw_viewer_menu();

    // Add new group
    if (ImGui::CollapsingHeader("Control Points", ImGuiTreeNodeFlags_DefaultOpen))
    {
      // Expose variable directly ...
      ImGui::InputInt("Size", &pointSize, 0, 0);
      ImGui::InputDouble("r", &r ,0, 0, "%.2f");
      ImGui::InputDouble("g", &g ,0, 0, "%.2f");
      ImGui::InputDouble("b", &b ,0, 0, "%.2f");

      // Add a button
      if (ImGui::Button("SET", ImVec2(-1,0)))
      {
        viewer.data().point_size = pointSize;
        viewer.data().set_points(CP_matrix, Eigen::RowVector3d(r, g, b));
      }

    }
  };
  const auto & update = [&]()
  {
    s -> convert2vector(control_points,CP_matrix);
    sample = s -> gen_tp_surface(control_points,20);
    s -> gen_mesh(V,F,sample);
    viewer.data().set_mesh(V, F);
    viewer.data().set_points(CP_matrix, Eigen::RowVector3d(r, g, b));
    
  };


  viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer&, int, int)->bool
  {
    last_mouse = Eigen::RowVector3f(
      viewer.current_mouse_x,viewer.core().viewport(3)-viewer.current_mouse_y,0);
    
    
      // Move closest control point
    Eigen::MatrixXf CP;
    igl::project(
        CP_matrix.cast<float>(),
        viewer.core().view,
        viewer.core().proj, viewer.core().viewport, CP);
    Eigen::VectorXf D = (CP.rowwise()-last_mouse).rowwise().norm();
    sel = (D.minCoeff(&sel) < 30)?sel:-1;
      if(sel != -1)
      {
        last_mouse(2) = CP(sel,2);
        update();
        return true;
    }
    return false;
  };

  viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer &, int,int)->bool
  {
    if(sel!=-1)
    {
      Eigen::RowVector3f drag_mouse(
        viewer.current_mouse_x,
        viewer.core().viewport(3) - viewer.current_mouse_y,
        last_mouse(2));
      Eigen::RowVector3f drag_scene,last_scene;
      igl::unproject(
        drag_mouse,
        viewer.core().view,
        viewer.core().proj,
        viewer.core().viewport,
        drag_scene);
      igl::unproject(
        last_mouse,
        viewer.core().view,
        viewer.core().proj,
        viewer.core().viewport,
        last_scene);
      CP_matrix.row(sel) += (drag_scene-last_scene).cast<double>();
      last_mouse = drag_mouse;
      update();
      return true;
    }
    return false;
  };
  
  viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int, int)->bool
  {
    sel = -1;
    return false;
  };

  viewer.data().set_mesh(V, F);
  viewer.data().set_face_based(true);
  viewer.core().is_animating = true;
  //plot control points
  
  viewer.data().point_size = pointSize;
  viewer.data().set_points(CP_matrix, Eigen::RowVector3d(r, g, b));
  viewer.launch();
}
