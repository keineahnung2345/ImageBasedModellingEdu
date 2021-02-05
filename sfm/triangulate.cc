/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <stdexcept>

#include "math/matrix_svd.h"
#include "sfm/triangulate.h"

SFM_NAMESPACE_BEGIN

/* ---------------- Low-level triangulation solver ---------------- */

math::Vector<double, 3>
triangulate_match (Correspondence2D2D const& match,
    CameraPose const& pose1, CameraPose const& pose2)
{
    // 由兩張圖像上的投影點及相機姿態恢復三維點座標
    /* The algorithm is described in HZ 12.2, page 312. */
    math::Matrix<double, 3, 4> P1, P2;
    pose1.fill_p_matrix(&P1);
    pose2.fill_p_matrix(&P2);

    std::cout<<"P1: "<<P1<<std::endl;
    std::cout<<"P2: "<<P2<<std::endl;

    math::Matrix<double, 4, 4> A;
    for (int i = 0; i < 4; ++i)
    {
        // x1 * P13 - P11
        A(0, i) = match.p1[0] * P1(2, i) - P1(0, i);
        // y1 * P13 - P12
        A(1, i) = match.p1[1] * P1(2, i) - P1(1, i);
        // x2 * P23 - P21
        A(2, i) = match.p2[0] * P2(2, i) - P2(0, i);
        // y2 * P23 - P22
        A(3, i) = match.p2[1] * P2(2, i) - P2(1, i);
    }
    std::cout<<"A: "<<A<<std::endl;

    math::Matrix<double, 4, 4> V;
    math::matrix_svd<double, 4, 4>(A, nullptr, nullptr, &V);
    // 空間中三維點的齊次座標
    math::Vector<double, 4> x = V.col(3);
    // 讓x的最後一個元素歸一化成1
    return math::Vector<double, 3>(x[0] / x[3], x[1] / x[3], x[2] / x[3]);
}

math::Vector<double, 3>
triangulate_track (std::vector<math::Vec2f> const& pos,
    std::vector<CameraPose const*> const& poses)
{
    // poses:相機姿態
    // pos:觀察點(二維點)的座標,長度大於等於二(triangulate_match函數只接受兩個觀察點)
    // 回傳"世界"座標系中的三維點座標
    if (pos.size() != poses.size() || pos.size() < 2)
        throw std::invalid_argument("Invalid number of positions/poses");

    std::vector<double> A(4 * 2 * poses.size(), 0.0);
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
        CameraPose const& pose = *poses[i];
        math::Vec2d p = pos[i];
        math::Matrix<double, 3, 4> p_mat;
        pose.fill_p_matrix(&p_mat);

        for (int j = 0; j < 4; ++j)
        {
            // xi * pi3 - pi1
            A[(2 * i + 0) * 4 + j] = p[0] * p_mat(2, j) - p_mat(0, j);
            // yi * pi3 - pi2
            A[(2 * i + 1) * 4 + j] = p[1] * p_mat(2, j) - p_mat(1, j);
        }
    }

    /* Compute SVD. */
    math::Matrix<double, 4, 4> mat_v;
    math::matrix_svd<double>(&A[0], 2 * poses.size(), 4,
        nullptr, nullptr, mat_v.begin());

    /* Consider the last column of V and extract 3D point. */
    math::Vector<double, 4> x = mat_v.col(3);
    return math::Vector<double, 3>(x[0] / x[3], x[1] / x[3], x[2] / x[3]);
}

bool
is_consistent_pose (Correspondence2D2D const& match,
    CameraPose const& pose1, CameraPose const& pose2)
{
    math::Vector<double, 3> x = triangulate_match(match, pose1, pose2);
    // 將三維點轉換到兩個相機的座標系中
    math::Vector<double, 3> x1 = pose1.R * x + pose1.t;
    math::Vector<double, 3> x2 = pose2.R * x + pose2.t;
    return x1[2] > 0.0f && x2[2] > 0.0f;
}

/* --------------- Higher-level triangulation class --------------- */

bool
Triangulate::triangulate (std::vector<CameraPose const*> const& poses,
    std::vector<math::Vec2f> const& positions,
    math::Vec3d* track_pos, Statistics* stats,
    std::vector<std::size_t>* outliers) const
{
    // poses: 相機姿態
    // positions: 觀察點(二維點)的位置
    if (poses.size() < 2)
        throw std::invalid_argument("At least two poses required");
    if (poses.size() != positions.size())
        throw std::invalid_argument("Poses and positions size mismatch");

    /* Check all possible pose pairs for successful triangulation */
    // 從poses中任選兩台相機來恢復三維點,選擇其中outlier(不能被滿足的相機)數最少的
    std::vector<std::size_t> best_outliers(positions.size());
    math::Vec3f best_pos(0.0f);
    for (std::size_t p1 = 0; p1 < poses.size(); ++p1)
        for (std::size_t p2 = p1 + 1; p2 < poses.size(); ++p2)
        {
            // p1, p2: 兩台相機
            // 用兩台相機恢復三維點
            /* Triangulate position from current pair */
            std::vector<CameraPose const*> pose_pair;
            std::vector<math::Vec2f> position_pair;
            pose_pair.push_back(poses[p1]);
            pose_pair.push_back(poses[p2]);
            position_pair.push_back(positions[p1]);
            position_pair.push_back(positions[p2]);
            // 世界座標系中的三維點座標
            math::Vec3d tmp_pos = triangulate_track(position_pair, pose_pair);
            if (MATH_ISNAN(tmp_pos[0]) || MATH_ISINF(tmp_pos[0]) ||
                MATH_ISNAN(tmp_pos[1]) || MATH_ISINF(tmp_pos[1]) ||
                MATH_ISNAN(tmp_pos[2]) || MATH_ISINF(tmp_pos[2]))
                continue;

            /* Check if pair has small triangulation angle. */
            // 檢查三維點與兩台相機的夾角
            // 是要保留夾角小的?!
            // ->還是說,如果夾角夠大,就可以略過下面的檢查
            if (this->opts.angle_threshold > 0.0)
            {
                math::Vec3d camera_pos;
                // 相機0在世界座標系中的位置
                pose_pair[0]->fill_camera_pos(&camera_pos);
                math::Vec3d ray0 = (tmp_pos - camera_pos).normalized();
                // 相機1在世界座標系中的位置
                pose_pair[1]->fill_camera_pos(&camera_pos);
                math::Vec3d ray1 = (tmp_pos - camera_pos).normalized();
                double const cos_angle = ray0.dot(ray1);
                // 角度超過threshold反而跳過?!
                if (cos_angle > this->cos_angle_thres)
                    continue;
            }

            // typo: Check
            /* Chek error in all input poses and find outliers. */
            /*
            檢查這個三維點在所有相機裡是否合法,
            如果不合法,就把不被滿足的相機id放到tmp_outliers裡
            */
            std::vector<std::size_t> tmp_outliers;
            for (std::size_t i = 0; i < poses.size(); ++i)
            {
                // tmp_pos: 世界座標系, x: 相機座標系
                math::Vec3d x = poses[i]->R * tmp_pos + poses[i]->t;

                /* Reject track if it appears behind the camera. */
                if (x[2] <= 0.0)
                {
                    tmp_outliers.push_back(i);
                    continue;
                }

                // 下面這兩步將相機座標系中的三維點x投影到圖像上
                x = poses[i]->K * x;
                math::Vec2d x2d(x[0] / x[2], x[1] / x[2]);
                // 重投影誤差
                double error = (positions[i] - x2d).norm();
                if (error > this->opts.error_threshold)
                    tmp_outliers.push_back(i);
            }

            /* Select triangulation with lowest amount of outliers. */
            // 這一對相機所生成的三維點造成的不合法的相機數較少
            if (tmp_outliers.size() < best_outliers.size())
            {
                best_pos = tmp_pos;
                std::swap(best_outliers, tmp_outliers);
            }

        }

    /* If all pairs have small angles pos will be 0 here. */
    /*
     best_pos.norm() == 0.0f代表它在上面的過程中一直沒被更新,
     也就是三維點與兩台相機的夾角太小,或不合法,或重投影誤差太大
    */
    if (best_pos.norm() == 0.0f)
    {
        if (stats != nullptr)
            stats->num_too_small_angle += 1;
        return false;
    }

    /* Check if required number of inliers is found. */
    // poses.size() - best_outliers.size()等於inlier數量,不能小於opts.min_num_views
    if (poses.size() < best_outliers.size() + this->opts.min_num_views)
    {
        if (stats != nullptr)
            stats->num_large_error += 1;
        return false;
    }

    /* Return final position and outliers. */
    *track_pos = best_pos;
    if (stats != nullptr)
        stats->num_new_tracks += 1;
    if (outliers != nullptr)
        std::swap(*outliers, best_outliers);

    return true;
}

void
Triangulate::print_statistics (Statistics const& stats, std::ostream& out) const
{
    int const num_rejected = stats.num_large_error
        + stats.num_behind_camera
        + stats.num_too_small_angle;

    out << "Triangulated " << stats.num_new_tracks
        << " new tracks, rejected " << num_rejected
        << " bad tracks." << std::endl;
    if (stats.num_large_error > 0)
        out << "  Rejected " << stats.num_large_error
            << " tracks with large error." << std::endl;
    if (stats.num_behind_camera > 0)
        out << "  Rejected " << stats.num_behind_camera
            << " tracks behind cameras." << std::endl;
    if (stats.num_too_small_angle > 0)
        out << "  Rejected " << stats.num_too_small_angle
            << " tracks with unstable angle." << std::endl;
}

SFM_NAMESPACE_END
