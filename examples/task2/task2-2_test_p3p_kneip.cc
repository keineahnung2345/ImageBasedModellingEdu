// Created by sway on 2018/8/25.
/*
 * POSIT (Pose from Orthography and Scaling with iterations), 比例正交
 * 投影迭代变换算法，适用条件是物体在Z轴方向的厚度远小于其在Z轴方向的平均深度。
 */
#include <complex>
#include <algorithm>
#include<set>
#include <iostream>

#include "math/matrix_tools.h"
#include "math/matrix.h"
#include "math/vector.h"
#include "sfm/correspondence.h"
#include "sfm/defines.h"
#include "util/system.h"

typedef math::Matrix<double, 3, 4> Pose;
typedef std::vector<Pose> PutativePoses;

/**
 *
 * @param factors
 * @param real_roots
 */
// 參考https://mathworld.wolfram.com/QuarticEquation.html
void solve_quartic_roots (math::Vec5d const& factors, math::Vec4d* real_roots)
{
    double const A = factors[0];
    double const B = factors[1];
    double const C = factors[2];
    double const D = factors[3];
    double const E = factors[4];

    double const A2 = A * A;
    double const B2 = B * B;
    double const A3 = A2 * A;
    double const B3 = B2 * B;
    double const A4 = A3 * A;
    double const B4 = B3 * B;

    double const alpha = -3.0 * B2 / (8.0 * A2) + C / A;
    double const beta = B3 / (8.0 * A3)- B * C / (2.0 * A2) + D / A;
    double const gamma = -3.0 * B4 / (256.0 * A4) + B2 * C / (16.0 * A3) - B * D / (4.0 * A2) + E / A;

    double const alpha2 = alpha * alpha;
    double const alpha3 = alpha2 * alpha;
    double const beta2 = beta * beta;

    std::complex<double> P(-alpha2 / 12.0 - gamma, 0.0);
    std::complex<double> Q(-alpha3 / 108.0 + alpha * gamma / 3.0 - beta2 / 8.0, 0.0);
    std::complex<double> R = -Q / 2.0 + std::sqrt(Q * Q / 4.0 + P * P * P / 27.0);

    std::complex<double> U = std::pow(R, 1.0 / 3.0);
    std::complex<double> y = (U.real() == 0.0)
                             ? -5.0 * alpha / 6.0 - std::pow(Q, 1.0 / 3.0)
                             : -5.0 * alpha / 6.0 - P / (3.0 * U) + U;

    std::complex<double> w = std::sqrt(alpha + 2.0 * y);
    std::complex<double> part1 = -B / (4.0 * A);
    std::complex<double> part2 = 3.0 * alpha + 2.0 * y;
    std::complex<double> part3 = 2.0 * beta / w;

    std::complex<double> complex_roots[4];
    complex_roots[0] = part1 + 0.5 * (w + std::sqrt(-(part2 + part3)));
    complex_roots[1] = part1 + 0.5 * (w - std::sqrt(-(part2 + part3)));
    complex_roots[2] = part1 + 0.5 * (-w + std::sqrt(-(part2 - part3)));
    complex_roots[3] = part1 + 0.5 * (-w - std::sqrt(-(part2 - part3)));

    for (int i = 0; i < 4; ++i)
        (*real_roots)[i] = complex_roots[i].real();
}


/**
 *
 * @param p1
 * @param p2
 * @param p3
 * @param f1
 * @param f2
 * @param f3
 * @param solutions
 */
void pose_p3p_kneip (
        math::Vec3d p1, math::Vec3d p2, math::Vec3d p3,
        math::Vec3d f1, math::Vec3d f2, math::Vec3d f3,
        std::vector<math::Matrix<double, 3, 4> >* solutions){

    // p1, p2, p3不能共面,即P1P2與P1P3的外積不可為0
    /* Check if points are co-linear. In this case return no solution. */
    double const colinear_threshold = 1e-10;
    if ((p2 - p1).cross(p3 - p1).square_norm() < colinear_threshold){
        solutions->clear();
        return;
    }

    /* Normalize directions if necessary. */
    // f1, f2, f3在normalize後才是論文裡所說的f1,f2,f3
    double const normalize_epsilon = 1e-10;
    if (!MATH_EPSILON_EQ(f1.square_norm(), 1.0, normalize_epsilon))
        f1.normalize();
    if (!MATH_EPSILON_EQ(f2.square_norm(), 1.0, normalize_epsilon))
        f2.normalize();
    if (!MATH_EPSILON_EQ(f3.square_norm(), 1.0, normalize_epsilon))
        f3.normalize();

    // 新相機座標系tau:有旋轉矩陣T
    /* Create camera frame. */
    math::Matrix3d T;
    {
        // e1,e2,e3為論文裡tau座標系的basis vector:tx,ty,tz
        math::Vec3d e1 = f1;
        math::Vec3d e3 = f1.cross(f2).normalized();
        math::Vec3d e2 = e3.cross(e1);
        std::copy(e1.begin(), e1.end(), T.begin() + 0);
        std::copy(e2.begin(), e2.end(), T.begin() + 3);
        std::copy(e3.begin(), e3.end(), T.begin() + 6);
        //等一下要用f3在tau座標系下的z座標來判斷平面PI的旋轉角度theta
        //所以這裡只轉f3
        f3 = T * f3;
    }

    // 希望f3的tz座標小於0,這樣f3才會跟P3一樣都在平面PI的後側,平面PI的旋轉角度theta才能落在[0,PI]裡
    /* Change camera frame and point order if f3[2] > 0. */
    if (f3[2] > 0.0)
    {
        std::swap(p1, p2);
        std::swap(f1, f2);

        math::Vec3d e1 = f1;
        math::Vec3d e3 = f1.cross(f2).normalized();
        math::Vec3d e2 = e3.cross(e1);
        std::copy(e1.begin(), e1.end(), T.begin() + 0);
        std::copy(e2.begin(), e2.end(), T.begin() + 3);
        std::copy(e3.begin(), e3.end(), T.begin() + 6);
        //這裡的f3是之前用T左乘過的f3(f3 = T * f3;)
        //第一次在乘的時候是不是應該備份一下原來的f3?!
        //或是在T被賦予新的值前,用T的轉置矩陣乘上f3?!
        f3 = T * f3;
    }

    // 新世界座標系: 原點p1, 旋轉矩陣N
    /* Create world frame. */
    math::Matrix3d N;
    {
        //論文裡新世界座標系的basis vector:nx,ny,nz
        math::Vec3d n1 = (p2 - p1).normalized();
        math::Vec3d n3 = n1.cross(p3 - p1).normalized();
        math::Vec3d n2 = n3.cross(n1);
        std::copy(n1.begin(), n1.end(), N.begin() + 0);
        std::copy(n2.begin(), n2.end(), N.begin() + 3);
        std::copy(n3.begin(), n3.end(), N.begin() + 6);
    }
    // p3在新世界座標系裡的表達
    p3 = N * (p3 - p1);

    /* Extraction of known parameters. */
    // 論文裡的d_12
    double d_12 = (p2 - p1).norm();
    // 論文裡的phi1
    double f_1 = f3[0] / f3[2];
    // 論文裡的phi2
    double f_2 = f3[1] / f3[2];
    // p3^eta的nx,ny座標
    double p_1 = p3[0];
    double p_2 = p3[1];

    double cos_beta = f1.dot(f2);
    // 論文裡的b = cot(beta)
    // 但是這裡的b = (cot(beta))^2
    double b = 1.0 / (1.0 - MATH_POW2(cos_beta)) - 1;

    // 開根號並決定正負號
    /*
    這裡用cos(beta)的正負號決定cot(beta)的正負號,原因如下:
    beta是向量f1與f2的夾角,我們可以限制它不超過180度
    觀察第一,二象限的cos正負號:+,-
    觀察第一,二象限的cot正負號:+,-
    所以在我們所關心的範圍內,cot是與cos同號的
    */
    if (cos_beta < 0.0)
        b = -std::sqrt(b);
    else
        b = std::sqrt(b);

    /* Temporary pre-computed variables. */
    double f_1_pw2 = MATH_POW2(f_1);
    double f_2_pw2 = MATH_POW2(f_2);
    double p_1_pw2 = MATH_POW2(p_1);
    double p_1_pw3 = p_1_pw2 * p_1;
    double p_1_pw4 = p_1_pw3 * p_1;
    double p_2_pw2 = MATH_POW2(p_2);
    double p_2_pw3 = p_2_pw2 * p_2;
    double p_2_pw4 = p_2_pw3 * p_2;
    double d_12_pw2 = MATH_POW2(d_12);
    double b_pw2 = MATH_POW2(b);

    // cos(theta)的四次多項式的係數
    /* Factors of the 4th degree polynomial. */
    math::Vec5d factors;
    // a4
    factors[0] = - f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;

    // a3
    factors[1] = 2.0 * p_2_pw3 * d_12 * b
                 + 2.0 * f_2_pw2 * p_2_pw3 * d_12 * b
                 - 2.0 * f_2 * p_2_pw3 * f_1 * d_12;

    // a2
    factors[2] = - f_2_pw2 * p_2_pw2 * p_1_pw2
                 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2
                 - f_2_pw2 * p_2_pw2 * d_12_pw2
                 + f_2_pw2 * p_2_pw4
                 + p_2_pw4 * f_1_pw2
                 + 2.0 * p_1 * p_2_pw2 * d_12
                 + 2.0 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * b
                 - p_2_pw2 * p_1_pw2 * f_1_pw2
                 + 2.0 * p_1 * p_2_pw2 * f_2_pw2 * d_12
                 - p_2_pw2 * d_12_pw2 * b_pw2
                 - 2.0 * p_1_pw2 * p_2_pw2;

    // a1
    factors[3] = 2.0 * p_1_pw2 * p_2 * d_12 * b
                 + 2.0 * f_2 * p_2_pw3 * f_1 * d_12
                 - 2.0 * f_2_pw2 * p_2_pw3 * d_12 * b
                 - 2.0 * p_1 * p_2 * d_12_pw2 * b;

    // a0
    factors[4] = -2.0 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * b
                 + f_2_pw2 * p_2_pw2 * d_12_pw2
                 + 2.0 * p_1_pw3 * d_12
                 - p_1_pw2 * d_12_pw2
                 + f_2_pw2 * p_2_pw2 * p_1_pw2
                 - p_1_pw4
                 - 2.0 * f_2_pw2 * p_2_pw2 * p_1 * d_12
                 + p_2_pw2 * f_1_pw2 * p_1_pw2
                 + f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;

    /* Solve for the roots of the polynomial. */
    math::Vec4d real_roots;
    solve_quartic_roots(factors, &real_roots);
    //這裡求得的根是cos(theta)

    /* Back-substitution of each solution. */
    solutions->clear();
    solutions->resize(4);
    for (int i = 0; i < 4; ++i)
    {
        // 論文公式(9):由cos(theta)得到cot(alpha)
        // 分子分母都比論文裡多了一個-1,結果是一樣的
        double cot_alpha = (-f_1 * p_1 / f_2 - real_roots[i] * p_2 + d_12 * b)
                           / (-f_1 * real_roots[i] * p_2 / f_2 + p_1 - d_12);

        double cos_theta = real_roots[i];
        // 因為限制theta小於180度,所以sin(theta)必為正值
        double sin_theta = std::sqrt(1.0 - MATH_POW2(real_roots[i]));
        // cot = cos/sin, cot^2+1 = cos^2/sin^2+1 = 1/sin^2
        // alpha是三角形CP1P2中P1的角,所以小於180度
        // sin(alpha)必為正值
        double sin_alpha = std::sqrt(1.0 / (MATH_POW2(cot_alpha) + 1));
        // 取決於alpha是否大於90度,cos(alpha)可能為正或負
        double cos_alpha = std::sqrt(1.0 - MATH_POW2(sin_alpha));

        // 決定cos(alpha)的正負號
        // 在一二象限cot(alpha)與cos(alpha)同號
        if (cot_alpha < 0.0)
            cos_alpha = -cos_alpha;

        // 論文公式(5): C^eta(alpha, theta)
        // 相機中心在新世界座標系裡的表達
        math::Vec3d C(
                d_12 * cos_alpha * (sin_alpha * b + cos_alpha),
                cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha),
                sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha));

        // 論文公式(12): 相機中心在原始世界座標系下的表達
        // 新世界 to 原始世界座標系
        C = p1 + N.transposed() * C;

        // 論文公式(6): 由新世界座標系到新相機座標系的旋轉矩陣,Q(alpha, theta)
        math::Matrix3d R;
        R[0] = -cos_alpha; R[1] = -sin_alpha * cos_theta; R[2] = -sin_alpha * sin_theta;
        R[3] = sin_alpha;  R[4] = -cos_alpha * cos_theta; R[5] = -cos_alpha * sin_theta;
        R[6] = 0.0;        R[7] = -sin_theta;             R[8] = cos_theta;

        // 論文公式(13): 由原始相機座標系到原始世界座標系的旋轉矩陣
        // 看論文圖(1): R的定義確實是這樣的
        R = N.transposed().mult(R.transposed()).mult(T);

        /* Convert camera position and cam-to-world rotation to pose. */
        // Xc = R*Xw+t -> Xw = R^T*Xc - R^T*t, Xc:某點在相機座標系下的表達,Xw:在世界座標系下的表達
        // Xc = R*O+t -> t = Xc, 又Cc = 0, 所以Cw = R^T*Cc - R^T*t = -R^T*Xc
        // R:原相機到原世界,R^T:原世界到原相機
        R = R.transposed();
        // 相機中心在世界座標系下表達
        C = -R * C;

        // 每個solution是由原始世界座標系到原始相機座標系及相機中心在世界座標系裡的表達?
        solutions->at(i) = R.hstack(C);
    }
}

/**
 *
 * @param corresp
 * @param inv_k_matrix
 * @param poses
 */
void compute_p3p (sfm::Correspondences2D3D const& corresp,
                            math::Matrix<double, 3, 3> const& inv_k_matrix,
                  PutativePoses* poses){

    if (corresp.size() < 3)
        throw std::invalid_argument("At least 3 correspondences required");

    /* Draw 3 unique random numbers. */
    std::set<int> result;
    while (result.size() < 3)
        result.insert(util::system::rand_int() % corresp.size());

    std::set<int>::const_iterator iter = result.begin();
    sfm::Correspondence2D3D const& c1(corresp[*iter++]);
    sfm::Correspondence2D3D const& c2(corresp[*iter++]);
    sfm::Correspondence2D3D const& c3(corresp[*iter]);

    //求解相機位姿
    pose_p3p_kneip(
            math::Vec3d(c1.p3d), math::Vec3d(c2.p3d), math::Vec3d(c3.p3d),
            inv_k_matrix.mult(math::Vec3d(c1.p2d[0], c1.p2d[1], 1.0)),
            inv_k_matrix.mult(math::Vec3d(c2.p2d[0], c2.p2d[1], 1.0)),
            inv_k_matrix.mult(math::Vec3d(c3.p2d[0], c3.p2d[1], 1.0)),
            poses);
}


int main(int argc, char*argv[]){

    // intrinsic matrix
    math::Matrix<double, 3, 3>k_matrix;
    k_matrix.fill(0.0);
    // fa
    k_matrix[0] = 0.972222;
    // 不是圖像左上方為原點?
    k_matrix[2] = 0.0; // cx =0 图像上的点已经进行了归一化（图像中心为原点，图像尺寸较长的边归一化为1）
    // fb
    k_matrix[4] = 0.972222;
    k_matrix[5] = 0.0; // cy=0  图像上的点已经进行了归一化（图像中心为原点，图像尺寸较长的边归一化为1）
    k_matrix[8] = 1.0;

    math::Matrix<double, 3, 3> inv_k_matrix = math::matrix_inverse(k_matrix);
    std::cout<<"inverse k matrix: "<<inv_k_matrix<<std::endl;
//    inverse k matrix: 1.02857 0 0
//    0 1.02857 0
//    0 0 1

    // 世界坐标系汇总3D点的坐标
    math::Vec3d p1(-2.57094,-0.217018, 6.05338);
    math::Vec3d p2(-0.803123, 0.251818, 6.98383);
    math::Vec3d p3(2.05584, -0.607918, 7.52573);

    // 对应的图像坐标系中的坐标（图像中心为原点，以图像长或宽归一化到[-0.5,0.5]之间。
    math::Vec2d uv1(-0.441758,-0.185523);
    math::Vec2d uv2(-0.135753,-0.0920593);
    math::Vec2d uv3(0.243795,-0.192743);

    // 计算相机坐标系中对应的摄线
    // 由圖像上轉到歸一化像平面上
    math::Vec3d f1 = inv_k_matrix.mult(math::Vec3d(uv1[0], uv1[1], 1.0));
    math::Vec3d f2 = inv_k_matrix.mult(math::Vec3d(uv2[0], uv2[1], 1.0));
    math::Vec3d f3 = inv_k_matrix.mult(math::Vec3d(uv3[0], uv3[1], 1.0));

//    math::Vec3d f1(-0.454379, -0.190824, 1);
//    math::Vec3d f2(-0.139631, -0.0946896, 1);
//    math::Vec3d f3(0.25076, -0.19825, 1);

   // kneip p3p计算出4组解
    std::vector<math::Matrix<double, 3, 4> >solutions;
    pose_p3p_kneip (p1, p2, p3, f1, f2, f3, &solutions);
    for(int i=0; i<solutions.size(); i++){
        std::cout<<"solution "<<i<<": "<<std::endl<<solutions[i]<<std::endl;
    }

    std::cout<<"the result should be \n"
    << "solution 0:"<<std::endl;
    std::cout<< "0.255193 -0.870436 -0.420972 3.11342\n"
    << "0.205372 0.474257 -0.856097 5.85432\n"
    << "0.944825 0.132022 0.299794 0.427496\n\n";

    std::cout<<"solution 1:"<<std::endl;
    std::cout<<"0.255203 -0.870431 -0.420976 3.11345\n"
    <<"0.205372 0.474257 -0.856097 5.85432\n"
    <<"0.944825 0.132022 0.299794 0.427496\n\n";

    std::cout<<"solution 2:"<<std::endl;
    std::cout<<"0.999829 -0.00839209 -0.0164611 -0.0488599\n"
    <<"0.00840016 0.999965 0.000421432 -0.905071\n"
    <<"0.016457 -0.000559636 0.999864 -0.0303736\n\n";

    std::cout<<"solution 3:"<<std::endl;
    std::cout<<"0.975996 0.122885 0.179806 -1.4207\n"
    <<"-0.213274 0.706483 0.67483 -5.68453\n"
    <<"-0.0441038 -0.69698 0.715733 1.71501\n\n";


    // 通过第4个点的投影计算正确的姿态
    math::Vec3d p4(-0.62611418962478638, -0.80525958538055419, 6.7783102989196777);
    math::Vec2d uv4(-0.11282696574926376,-0.24667978286743164);
    //const double thresh = 0.005;

    /* Check all putative solutions and count inliers. */
    for (std::size_t j = 0; j < solutions.size(); ++j){
        // 第四個點的齊次座標
        math::Vec4d p3d(p4[0], p4[1], p4[2], 1.0);
        // solutions[j] * p3d: 由原世界到原相機
        // 有了原世界到原相機座標系的轉換之後,再套用第一章的針孔相機模型公式即可
        // 課件裡是先除以zc再乘k_matrix,這裡是先乘k_matrix
        // k_matrix * : 由歸一化像平面到圖像
        math::Vec3d p2d = k_matrix * (solutions[j] * p3d);
        //p2d[0] / p2d[2], p2d[1] / p2d[2]: 除以zc,由相機座標系到歸一化像平面
        //重投影誤差
        double square_error = MATH_POW2(p2d[0] / p2d[2] - uv4[0])
                              + MATH_POW2(p2d[1] / p2d[2] - uv4[1]);
        std::cout<<"reproj err of solution "<<j<<" "<<square_error<<std::endl;
    }

    return 0;
}
