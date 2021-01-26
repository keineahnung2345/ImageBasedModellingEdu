//
// Created by caoqi on 2018/8/31.
//


//3D:  1.36939, -1.17123, 7.04869
//obs: 0.180123 -0.156584


#include "sfm/bundle_adjustment.h"
#include "math/vector.h"
#include "math/matrix.h"
/*
 * This function computes the Jacobian entries for the given camera and
 * 3D point pair that leads to one observation.
 *
 * The camera block 'cam_x_ptr' and 'cam_y_ptr' is:
 * - ID 0: Derivative of focal length f
 * - ID 1-2: Derivative of distortion parameters k0, k1
 * - ID 3-5: Derivative of translation t0, t1, t2
 * - ID 6-8: Derivative of rotation w0, w1, w2
 *
 * The 3D point block 'point_x_ptr' and 'point_y_ptr' is:
 * - ID 0-2: Derivative in x, y, and z direction.
 *
 * The function that leads to the observation is given as follows:
 *
 *   u = f * D(x,y) * x  (image observation x coordinate)
 *   v = f * D(x,y) * y  (image observation y coordinate)
 *
 * with the following definitions:
 *
 *   xc = R0 * X + t0  (homogeneous projection)
 *   yc = R1 * X + t1  (homogeneous projection)
 *   zc = R2 * X + t2  (homogeneous projection)
 *   x = xc / zc  (central projection)
 *   y = yc / zc  (central projection)
 *   D(x, y) = 1 + k0 (x^2 + y^2) + k1 (x^2 + y^2)^2  (distortion)
 */

 /**
  * /description 给定一个相机参数和一个三维点坐标，求解雅各比矩阵，即公式中的df(theta)/dtheta
  * @param cam       相机参数
  * @param point     三维点坐标
  * @param cam_x_ptr 重投影坐标x 相对于相机参数的偏导数，相机有9个参数： [0] 焦距f; [1-2] 径向畸变系数k1, k2; [3-5] 平移向量 t1, t2, t3
  *                                                               [6-8] 旋转矩阵（角轴向量）
  * @param cam_y_ptr    重投影坐标y 相对于相机参数的偏导数，相机有9个参数
  * @param point_x_ptr  重投影坐标x 相对于三维点坐标的偏导数
  * @param point_y_ptr  重投影坐标y 相对于三维点坐标的偏导数
  */
void jacobian(sfm::ba::Camera const& cam,
              sfm::ba::Point3D const& point,
              double* cam_x_ptr, double* cam_y_ptr,
              double* point_x_ptr, double* point_y_ptr)
{
    const double f = cam.focal_length;
    const double *R = cam.rotation;
    const double *t = cam.translation;
    const double *X = point.pos;
    const double k0 = cam.distortion[0];
    const double k1 = cam.distortion[1];

    math::Matrix<double, 3, 3> R_mat(R);
    math::Vec3d t_vec(t);
    math::Vec3d X_vec(X);
    math::Vec3d Xc_vec = R_mat * X_vec + t_vec; //3d point in camera coordinate
    math::Vec2d X_norm(Xc_vec[0]/Xc_vec[2], Xc_vec[1]/Xc_vec[2]); //in normalized image plane
    const double r_sq = X_norm[0]*X_norm[0] + X_norm[1]*X_norm[1];
    const double D = 1 + k0 * r_sq + k1 * r_sq * r_sq;

    // 相机焦距的偏导数
    // du/df = D(x,y) * x
    cam_x_ptr[0] = D * X_norm[0];
    // dv/df = D(x,y) * y
    cam_y_ptr[0] = D * X_norm[1];

    // 相机径向畸变的偏导数
    // du/dk0 = f * x * r^2
    cam_x_ptr[1] = f * X_norm[0] * r_sq;
    // du/dk1 = f * x * r^4
    cam_x_ptr[2] = f * X_norm[0] * r_sq * r_sq;
    // dv/dk0 = f * y * r^2
    cam_y_ptr[1] = f * X_norm[1] * r_sq;
    // dv/dk1 = f * y * r^4
    cam_y_ptr[2] = f * X_norm[1] * r_sq * r_sq;

    // 相机将向畸变系数的偏导数 //duplicate!!
    // cam_x_ptr[1] = 0.0;
    // cam_x_ptr[2] = 0.0;
    // cam_y_ptr[1] = 0.0;
    // cam_y_ptr[2] = 0.0;

    // du/dD = f * x
    const double dudD = f * X_norm[0];
    // dv/dD = f * y
    const double dvdD = f * X_norm[1];
    // du/dx = fD
    const double dudx = f * D;
    // dv/dy = fD
    const double dvdy = f * D;
    // dD/dxc = (k0 + 2*k1*r^2)*2*x/zc
    const double dDdxc = (k0 + 2*k1*r_sq)*2*X_norm[0]/Xc_vec[2];
    // dD/dyc = (k0 + 2*k1*r^2)*2*y/zc
    const double dDdyc = (k0 + 2*k1*r_sq)*2*X_norm[1]/Xc_vec[2];
    // dD/dzc = -(k0 + 2*k1*r^2)*2*r^2/zc
    const double dDdzc = -(k0 + 2*k1*r_sq)*2*r_sq/Xc_vec[2];
    // dx/dxc = 1/zc
    const double dxdxc = 1/Xc_vec[2];
    // dx/dyc = 0
    const double dxdyc = 0;
    // dx/dzc = -x/zc
    const double dxdzc = -X_norm[0]/Xc_vec[2];
    // dy/dxc = 0
    const double dydxc = 0;
    // dy/dyc = 1/zc
    const double dydyc = 1/Xc_vec[2];
    // dy/dzc = -y/zc
    const double dydzc = -X_norm[1]/Xc_vec[2];

    // 相机平移向量的偏导数
    // du/dt0 = du/dxc = du/dD * dD/dxc + du/dx * dx/dxc
    cam_x_ptr[3] = dudD * dDdxc + dudx * dxdxc;
    // du/dt1 = du/dyc = du/dD * dD/dyc + du/dx * dx/dyc
    cam_x_ptr[4] = dudD * dDdyc + dudx * dxdyc;
    // du/dt2 = du/dzc = du/dD * dD/dzc + du/dx * dx/dzc
    cam_x_ptr[5] = dudD * dDdzc + dudx * dxdzc;
    // dv/dt0 = dv/dxc = dv/dD * dD/dxc + dv/dy * dy/dxc
    cam_y_ptr[3] = dvdD * dDdxc + dvdy * dydxc;
    // dv/dt1 = dv/dyc = dv/dD * dD/dyc + dv/dy * dy/dyc
    cam_y_ptr[4] = dvdD * dDdyc + dvdy * dydyc;
    // dv/dt2 = dv/dzc = dv/dD * dD/dzc + dv/dy * dy/dzc
    cam_y_ptr[5] = dvdD * dDdzc + dvdy * dydzc;

    // dxc/dw0 = 0
    const double dxcdw0 = 0;
    // dxc/dw1 = r2 * X
    const double dxcdw1 = R_mat.row(2).dot(X_vec);
    // dxc/dw2 = -r1 * X
    const double dxcdw2 = -R_mat.row(1).dot(X_vec);
    // dyc/dw0 = -r2 * X
    const double dycdw0 = -R_mat.row(2).dot(X_vec);
    // dyc/dw1 = 0
    const double dycdw1 = 0;
    // dyc/dw2 = r0 * X
    const double dycdw2 = R_mat.row(0).dot(X_vec);
    // dzc/dw0 = r1 * X
    const double dzcdw0 = R_mat.row(1).dot(X_vec);
    // dzc/dw1 = -r0 * X
    const double dzcdw1 = -R_mat.row(0).dot(X_vec);
    // dzc/dw2 = 0
    const double dzcdw2 = 0;

    // 相机旋转矩阵的偏导数
    // du/dw0 = du/dyc * dyc/dw0 + du/dzc * dzc/dw0
    cam_x_ptr[6] = cam_x_ptr[4] * dycdw0 + cam_x_ptr[5] * dzcdw0;
    // du/dw1 = du/dxc * dxc/dw1 + du/dzc * dzc/dw1
    cam_x_ptr[7] = cam_x_ptr[3] * dxcdw1 + cam_x_ptr[5] * dzcdw1;
    // du/dw2 = du/dxc * dxc/dw2 + du/dyc * dyc/dw2
    cam_x_ptr[8] = cam_x_ptr[3] * dxcdw2 + cam_x_ptr[4] * dycdw2;
    // dv/dw0 = dv/dyc * dyc/dw0 + dv/dzc * dzc/dw0
    cam_y_ptr[6] = cam_y_ptr[4] * dycdw0 + cam_y_ptr[5] * dzcdw0;
    // dv/dw1 = dv/dxc * dxc/dw1 + dv/dzc * dzc/dw1
    cam_y_ptr[7] = cam_y_ptr[3] * dxcdw1 + cam_y_ptr[5] * dzcdw1;
    // dv/dw2 = dv/dxc * dxc/dw2 + dv/dyc * dyc/dw2
    cam_y_ptr[8] = cam_y_ptr[3] * dxcdw2 + cam_y_ptr[4] * dycdw2;

    // 三维点的偏导数
    // du/dxw = du/dxc * dxc/dxw + du/dyc * dyc/dxw + du/dzc * dzc/dxw
    point_x_ptr[0] = cam_x_ptr[3] * R_mat(0,0) + cam_x_ptr[4] * R_mat(1,0) + cam_x_ptr[5] * R_mat(2,0);
    // du/dyw = du/dxc * dxc/dyw + du/dyc * dyc/dyw + du/dzc * dzc/dyw
    point_x_ptr[1] = cam_x_ptr[3] * R_mat(0,1) + cam_x_ptr[4] * R_mat(1,1) + cam_x_ptr[5] * R_mat(2,1);
    // du/dzw = du/dxc * dxc/dzw + du/dyc * dyc/dzw + du/dzc * dzc/dzw
    point_x_ptr[2] = cam_x_ptr[3] * R_mat(0,2) + cam_x_ptr[4] * R_mat(1,2) + cam_x_ptr[5] * R_mat(2,2);
    // dv/dxw = dv/dxc * dxc/dxw + dv/dyc * dyc/dxw + dv/dzc * dzc/dxw
    point_y_ptr[0] = cam_y_ptr[3] * R_mat(0,0) + cam_y_ptr[4] * R_mat(1,0) + cam_y_ptr[5] * R_mat(2,0);
    // dv/dyw = dv/dxc * dxc/dyw + dv/dyc * dyc/dyw + dv/dzc * dzc/dyw
    point_y_ptr[1] = cam_y_ptr[3] * R_mat(0,1) + cam_y_ptr[4] * R_mat(1,1) + cam_y_ptr[5] * R_mat(2,1);
    // dv/dzw = dv/dxc * dxc/dzw + dv/dyc * dyc/dzw + dv/dzc * dzc/dzw
    point_y_ptr[2] = cam_y_ptr[3] * R_mat(0,2) + cam_y_ptr[4] * R_mat(1,2) + cam_y_ptr[5] * R_mat(2,2);




}
int main(int argc, char*argv[])
{

    sfm::ba::Camera cam;
    cam.focal_length  =  0.919654;
    cam.distortion[0] = -0.108298;
    cam.distortion[1] =  0.103775;

    cam.rotation[0] = 0.999999;
    cam.rotation[1] = -0.000676196;
    cam.rotation[2] = -0.0013484;
    cam.rotation[3] = 0.000663243;
    cam.rotation[4] = 0.999949;
    cam.rotation[5] = -0.0104095;
    cam.rotation[6] = 0.00135482;
    cam.rotation[7] = 0.0104087;
    cam.rotation[8] = 0.999949;

    cam.translation[0]=0.00278292;
    cam.translation[1]=0.0587996;
    cam.translation[2]=-0.127624;

    sfm::ba::Point3D pt3D;
    pt3D.pos[0]= 1.36939;
    pt3D.pos[1]= -1.17123;
    pt3D.pos[2]= 7.04869;

    double cam_x_ptr[9]={0};
    double cam_y_ptr[9]={0};
    double point_x_ptr[3]={0};
    double point_y_ptr[3]={0};

    jacobian(cam, pt3D, cam_x_ptr, cam_y_ptr, point_x_ptr, point_y_ptr);


   std::cout<<"Result is :"<<std::endl;
    std::cout<<"cam_x_ptr: ";
    for(int i=0; i<9; i++){
        std::cout<<cam_x_ptr[i]<<" ";
    }
    std::cout<<std::endl;

    std::cout<<"cam_y_ptr: ";
    for(int i=0; i<9; i++){

        std::cout<<cam_y_ptr[i]<<" ";
    }
    std::cout<<std::endl;

    std::cout<<"point_x_ptr: ";
    std::cout<<point_x_ptr[0]<<" "<<point_x_ptr[1]<<" "<<point_x_ptr[2]<<std::endl;

    std::cout<<"point_y_ptr: ";
    std::cout<<point_y_ptr[0]<<" "<<point_y_ptr[1]<<" "<<point_y_ptr[2]<<std::endl;


    std::cout<<"\nResult should be :\n"
       <<"cam_x_ptr: 0.195942 0.0123983 0.000847141 0.131188 0.000847456 -0.0257388 0.0260453 0.95832 0.164303\n"
       <<"cam_y_ptr: -0.170272 -0.010774 -0.000736159 0.000847456 0.131426 0.0223669 -0.952795 -0.0244697 0.179883\n"
       <<"point_x_ptr: 0.131153 0.000490796 -0.0259232\n"
       <<"point_y_ptr: 0.000964926 0.131652 0.0209965\n";


    return 0;
}
