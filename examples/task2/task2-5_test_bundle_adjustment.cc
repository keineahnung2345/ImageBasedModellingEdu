#include <features/matching.h>
#include <sfm/ransac_fundamental.h>
#include <core/image_exif.h>
#include <fstream>
#include <cassert>
#include "math/matrix.h"
#include "math/vector.h"

#include "core/image_io.h"
#include "core/image.h"
#include "core/image_tools.h"
#include "core/image_drawing.h"

#include "sfm/camera_pose.h"
#include "sfm/fundamental.h"

#include "sfm/feature_set.h"
#include "sfm/correspondence.h"
#include "sfm/bundle_adjustment.h"
#include "sfm/correspondence.h"

#include "sfm/camera_database.h"
#include "sfm/extract_focal_length.h"

#include "sfm/triangulate.h"

#define MAX_PIXELS 1000000


void convert_sift_discriptors(features::Sift::Descriptors const&sift_descrs,
                              util::AlignedMemory<math::Vec128f, 16> *aligned_descr)
{
    aligned_descr->resize(sift_descrs.size());
    float * data_ptr=aligned_descr->data()->begin();
    for(int i=0; i<sift_descrs.size(); ++i, data_ptr+=128)
    {
        features::Sift::Descriptor const& descr = sift_descrs[i];
        std::copy(descr.data.begin(), descr.data.end(), data_ptr);
    }

}

float extract_focal_len(const std::string& img_name)
{
    std::string exif_str;
    core::image::load_jpg_file(img_name.c_str(), &exif_str);
    core::image::ExifInfo exif = core::image::exif_extract(exif_str.c_str(), exif_str.size(), false);
    sfm::FocalLengthEstimate fl = sfm::extract_focal_length(exif);
    std::cout <<"Focal length: " <<fl.first << " " << fl.second << std::endl;
    return fl.first;
}


sfm::Correspondences2D2D sift_feature_matching(sfm::FeatureSet &feat1
        , sfm::FeatureSet&feat2)
{

    /* 1.0 特征匹配*/
    // 进行数据转换
    util::AlignedMemory<math::Vec128f, 16> aligned_descrs1, aligned_descrs2;
    convert_sift_discriptors(feat1.sift_descriptors, &aligned_descrs1);
    convert_sift_discriptors(feat2.sift_descriptors, &aligned_descrs2);

    // 特征匹配参数设置
    features::Matching::Options matching_opts;
    matching_opts.descriptor_length = 128;
    matching_opts.distance_threshold = 1.0f;
    matching_opts.lowe_ratio_threshold = 0.8f;

    // 特征匹配
    features::Matching::Result matching_result;
    features::Matching::twoway_match(matching_opts, aligned_descrs1.data()->begin()
            , feat1.sift_descriptors.size()
            ,aligned_descrs2.data()->begin()
            , feat2.sift_descriptors.size(),&matching_result);
    // 去除不一致的匹配对
    features::Matching::remove_inconsistent_matches(&matching_result);
    int n_consitent_matches = features::Matching::count_consistent_matches(matching_result);
    std::cout << "Consistent Sift Matches: "
              << n_consitent_matches
              << std::endl;

    /*2.0 利用本征矩阵对数据进行*/
    // 进行特征点坐标归一化，归一化之后坐标中心位于(0,0), 范围[-0.5, 0.5]。坐标归一化有助于
    // 保持计算的稳定性
    feat1.normalize_feature_positions();
    feat2.normalize_feature_positions();

    sfm::Correspondences2D2D corr_all;
    std::vector<int> const & m12 = matching_result.matches_1_2;
    for(int i=0; i<m12.size(); i++)
    {
        if(m12[i]<0)continue;

        sfm::Correspondence2D2D c2d2d;
        c2d2d.p1[0] = feat1.positions[i][0];
        c2d2d.p1[1] = feat1.positions[i][1];
        c2d2d.p2[0] = feat2.positions[m12[i]][0];
        c2d2d.p2[1] = feat2.positions[m12[i]][1];

        corr_all.push_back(c2d2d);
    }
    /* RANSAC 估计本征矩阵, 并对特征匹配对进行筛选*/
    sfm::RansacFundamental::Options ransac_fundamental_opts;
    ransac_fundamental_opts.max_iterations =1000;
    ransac_fundamental_opts.verbose_output = true;
    sfm::RansacFundamental ransac_fundamental(ransac_fundamental_opts);
    sfm::RansacFundamental::Result ransac_fundamental_result;
    ransac_fundamental.estimate(corr_all, &ransac_fundamental_result);
    // 根据估计的Fundamental矩阵对特征匹配对进行筛选
    sfm::Correspondences2D2D corr_f;
    for(int i=0; i<ransac_fundamental_result.inliers.size(); ++i)
    {
        int inlier_id = ransac_fundamental_result.inliers[i];
        corr_f.push_back(corr_all[inlier_id]);
    }

    std::cout<<"F: "<<ransac_fundamental_result.fundamental<<std::endl;
    return corr_f;
}


bool calc_cam_poses(sfm::Correspondences2D2D const &matches
                  , sfm::CameraPose* pose1
                  , sfm::CameraPose* pose2
                  , float f1
                  , float f2)
{
    // calculate fundamental matrix from  pair correspondences
    sfm::FundamentalMatrix F;
    sfm::fundamental_least_squares(matches, &F);
    sfm::enforce_essential_constraints(&F);

    // 设置相机1的内参矩阵
    pose1->set_k_matrix(f1, 0.0, 0.0);
    // 相机1为参考矩阵，将其姿态设置成单位矩阵

    pose1->init_canonical_form();
    pose2->set_k_matrix(f2, 0.0, 0.0);

    // todo 从本征矩阵中计算本质矩阵
    sfm::EssentialMatrix E = pose2->K.transpose()*F*pose1->K;
    std::cout<<"E: "<<E<<std::endl;

    // 从本质矩阵中计算相机姿态,一共有四种情况
    std::vector<sfm::CameraPose> poses;
    sfm::pose_from_essential(E, &poses);

    // 利用匹配点对从4中相机姿态中找到最合适的(利用姿态对匹配点进行三角化得到三维点，计算三维点的相机坐标，
    // 在两个相机的中的z坐标必须为正
    bool found_pose=false;
    for(std::size_t i=0; i< poses.size(); ++i){
        poses[i].K = pose2->K;
        if(sfm::is_consistent_pose(matches[0],*pose1, poses[i]))
        {
            *pose2 = poses[i];
            found_pose = true;
            break;
        }
    }
    return found_pose;
}

void radial_distort (double* x, double* y, double const* dist)
{
    double const radius2 = *x * *x + *y * *y;
    double const factor = 1.0 + radius2 * (dist[0] + dist[1] * radius2);
    *x *= factor;
    *y *= factor;
}

void reproject(std::vector<double>& rps
        , std::vector<sfm::ba::Camera>& cameras
        , std::vector<sfm::ba::Point3D>& points
        ,std::vector<sfm::ba::Observation>& observations){
    if (rps.size() != observations.size() * 2)
        rps.resize(observations.size() * 2);

    // reproject new_pts_3d to image and show
    #pragma omp parallel for
    for (std::size_t i = 0; i < observations.size(); ++i)
    {
        //二維平面上觀察到的點
        sfm::ba::Observation const& obs = observations.at(i);
        sfm::ba::Point3D const& p3d = points.at(obs.point_id);
        sfm::ba::Camera const& cam = cameras.at(obs.camera_id);

        //sfm::ba::Camera裡focal_length是double,其餘是double的array
        double const* flen = &cam.focal_length; // 相机焦距
        double const* dist = cam.distortion;    // 径向畸变系数
        double const* rot = cam.rotation;       // 相机旋转矩阵
        double const* trans = cam.translation;  // 相机平移向量
        double const* point = p3d.pos;          // 三维点坐标

        sfm::ba::Point3D new_point;
        sfm::ba::Camera new_camera;

        /* Project point onto image plane. */
        double rp[] = { 0.0, 0.0, 0.0 };
        for (int d = 0; d < 3; ++d)
        {
            rp[0] += rot[0 + d] * point[d];
            rp[1] += rot[3 + d] * point[d];
            rp[2] += rot[6 + d] * point[d];
        }
        rp[2] = (rp[2] + trans[2]);
        rp[0] = (rp[0] + trans[0]) / rp[2];
        rp[1] = (rp[1] + trans[1]) / rp[2];

        /* Distort reprojections. */
        radial_distort(rp + 0, rp + 1, dist);

        /* Compute reprojection error. */
        rps.at(i * 2 + 0) = rp[0] * (*flen);
        rps.at(i * 2 + 1) = rp[1] * (*flen);
    }
};

void denormalize(const float& fnorm, 
        const float& fwidth, 
        const float& fheight,
        double& u, 
        double& v){
    u = u * fnorm + fwidth / 2.0f - 0.5f;
    v = v * fnorm + fheight / 2.0f - 0.5f;
};

int
main (int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "Syntax: " << argv[0] << " <img1> <img2>" << std::endl;
        return 1;
    }

    /* 1.0 加载图像. */
    std::string fname1(argv[1]);
    std::string fname2(argv[2]);
    std::cout << "Loading " << fname1 << "..." << std::endl;
    core::ByteImage::Ptr img1 = core::image::load_file(fname1);
    std::cout << "Loading " << fname2 << "..." << std::endl;
    core::ByteImage::Ptr img2 = core::image::load_file(fname2);
    // 控制图像尺寸
    while(img1->get_pixel_amount() > MAX_PIXELS){
        img1=core::image::rescale_half_size<uint8_t>(img1);
    }
    while(img2->get_pixel_amount() > MAX_PIXELS){
        img2=core::image::rescale_half_size<uint8_t>(img2);
    }

    /* 提取相机焦距*/
    float f1  = extract_focal_len(argv[1]);
    float f2 = extract_focal_len(argv[2]);
    std::cout<<"focal length: f1 "<<f1<<" f2: "<<f2<<std::endl;

    /*2.0 计算sift特征点 */
    sfm::FeatureSet::Options feature_set_opts;
    feature_set_opts.feature_types = sfm::FeatureSet::FEATURE_SIFT;
    feature_set_opts.sift_opts.verbose_output = true;

    sfm::FeatureSet feat1(feature_set_opts);
    feat1.compute_features(img1);

    sfm::FeatureSet feat2(feature_set_opts);
    feat2.compute_features(img2);

    std::cout << "Image 1 (" << img1->width() << "x" << img1->height() << ") "
              << feat1.sift_descriptors.size() << " descriptors." << std::endl;
    std::cout << "Image 2 (" << img2->width() << "x" << img2->height() << ") "
              << feat2.sift_descriptors.size() << " descriptors." << std::endl;

    // 对特征坐标进行归一化，使得求解计算稳定
    sfm::Correspondences2D2D corrs = sift_feature_matching(feat1, feat2);
    std::cout<<"Number of Matching pairs is "<< corrs.size()<<std::endl;

//    std::cout<<"correspondence: "<<std::endl;
//    for(int i=0; i< corrs.size(); i++) {
//        std::cout<<corrs[i].p1[0]<<" "<<corrs[i].p1[1]
//                 <<" "<< corrs[i].p2[0]<<" "<<corrs[i].p2[1]<<std::endl;
//    }

    core::ByteImage::Ptr img1_drawn(img1), img2_drawn(img2);
    unsigned char color_rgb_red[3] = { 255, 0, 0 };
    unsigned char color_rgb_blue[3] = { 0, 0, 255 };
    float const f1width = static_cast<float>(feat1.width);
    float const f1height = static_cast<float>(feat1.height);
    float const f1norm = std::max(f1width, f1height);
    float const f2width = static_cast<float>(feat2.width);
    float const f2height = static_cast<float>(feat2.height);
    float const f2norm = std::max(f2width, f2height);
    for(sfm::Correspondence2D2D& corr : corrs){
        // denormalize image coordinates
        double p1u = corr.p1[0];
        double p1v = corr.p1[1];
        double p2u = corr.p2[0];
        double p2v = corr.p2[1];
        denormalize(f1norm, f1width, f1height, p1u, p1v);
        denormalize(f2norm, f2width, f2height, p2u, p2v);
        
        std::cout << "corr: (" << p1u << ", " << p1v << 
            ") <-> (" << p2u << ", " << p2v << ")" << std::endl;
        core::image::draw_circle<uint8_t>(*img1_drawn, p1u, p1v, 2, color_rgb_red);
        core::image::draw_circle<uint8_t>(*img2_drawn, p2u, p2v, 2, color_rgb_red);
    }
    core::image::save_file(img1_drawn, std::string("img1_drawn.jpg"));
    core::image::save_file(img2_drawn, std::string("img2_drawn.jpg"));

    if(corrs.size()<8){
        std::cerr<<" Number of matching pairs should not be less than 8."<<std::endl;
    }


    /* 计算相机姿态 */
    sfm::CameraPose pose1, pose2;
    if(!calc_cam_poses(corrs, &pose1, &pose2, f1, f2))
    {
        std::cerr<<"Error to find corrent camera poses!"<<std::endl;
        return -1;
    }

    /* 三角化 */
    std::vector<math::Vec3f> pts_3d;
    sfm::Correspondences2D2D corrs_filtered;
    for(int i=0; i<corrs.size(); i++)
    {
        math::Vec3f pt_3d = sfm::triangulate_match(corrs[i], pose1, pose2);
        if (MATH_ISNAN(pt_3d[0]) || MATH_ISINF(pt_3d[0]) ||
            MATH_ISNAN(pt_3d[1]) || MATH_ISINF(pt_3d[1]) ||
            MATH_ISNAN(pt_3d[2]) || MATH_ISINF(pt_3d[2]))
            continue;

        pts_3d.push_back(pt_3d);
        corrs_filtered.push_back(corrs[i]);
    }
    std::cout<<"Successful triangulation:  "<<pts_3d.size()<<" points"<<std::endl;
//    for(int i=0; i<pts_3d.size(); i++)
//    {
//        std::cout<<pts_3d[i][0]<<" "<<pts_3d[i][1]<<" "<<pts_3d[i][2]<<std::endl;
//    }

    std::ofstream out("./examples/task2/test_ba.txt");
    assert(out.is_open());

    /*捆绑调整*/
    std::vector<sfm::ba::Camera> cams(2);
    out<<"n_cam "<< cams.size()<<std::endl;

    cams[0].focal_length = pose1.get_focal_length();
    std::copy(pose1.t.begin(), pose1.t.end(), cams[0].translation);
    std::copy(pose1.R.begin(), pose1.R.end(), cams[0].rotation);
    std::fill(cams[0].distortion, cams[0].distortion + 2, 0.0);

    out<<cams[0].focal_length<<" ";
    out<<cams[0].distortion[0]<<" "<< cams[0].distortion[1]<<" ";
    for(int i=0; i<3; i++) out<<cams[0].translation[i]<<" ";
    for(int i=0; i<9; i++) out<<cams[0].rotation[i]<<" ";
    out<<std::endl;

    cams[1].focal_length = pose2.get_focal_length();
    std::copy(pose2.t.begin(), pose2.t.end(), cams[1].translation);
    std::copy(pose2.R.begin(), pose2.R.end(), cams[1].rotation);
    std::fill(cams[1].distortion, cams[1].distortion + 2, 0.0);

    out<<cams[1].focal_length<<" ";
    out<<cams[1].distortion[0]<<" "<< cams[1].distortion[1]<<" ";
    for(int i=0; i<3; i++) out<<cams[1].translation[i]<<" ";
    for(int i=0; i<9; i++) out<<cams[1].rotation[i]<<" ";
    out<<std::endl;

    // 3d 点
    std::vector<sfm::ba::Point3D> p3ds;
    for(int i=0; i< pts_3d.size(); i++)
    {
        sfm::ba::Point3D pt;
        pt.pos[0] = pts_3d[i][0];
        pt.pos[1] = pts_3d[i][1];
        pt.pos[2] = pts_3d[i][2];
        p3ds.push_back(pt);
    }

    out<<"n_points "<< p3ds.size()<<std::endl;
    for(int i=0; i< p3ds.size(); i++){
        out<<p3ds[i].pos[0]<<" "<<p3ds[i].pos[1]<<" "<<p3ds[i].pos[2]<<std::endl;
    }

    assert(p3ds.size()==corrs.size());
    // 观察点
    std::vector<sfm::ba::Observation> observations;
    for (std::size_t i = 0; i < p3ds.size(); ++i)
    {
        sfm::ba::Observation obs1;
        obs1.camera_id = 0;
        obs1.point_id = i;
        std::copy(corrs[i].p1, corrs[i].p1+2, obs1.pos);


        sfm::ba::Observation obs2;
        obs2.camera_id = 1;
        obs2.point_id = i;
        std::copy(corrs[i].p2, corrs[i].p2+2, obs2.pos);

        observations.push_back(obs1);
        observations.push_back(obs2);
    }

    out<<"n_observations "<<observations.size()<<std::endl;
    for(int i=0; i<observations.size(); i++){
        out<<observations[i].camera_id<<" "<<observations[i].point_id<<" "
           <<observations[i].pos[0]<<" "<<observations[i].pos[1]<<std::endl;
    }


    // ba优化
    sfm::ba::BundleAdjustment::Options ba_opts;
    ba_opts.verbose_output = true;
    ba_opts.lm_mse_threshold = 1e-16;
    ba_opts.lm_delta_threshold = 1e-8;
    sfm::ba::BundleAdjustment ba(ba_opts);
    ba.set_cameras(&cams);
    ba.set_points(&p3ds);
    ba.set_observations(&observations);
    ba.optimize();
    ba.print_status();

    // 将优化后的结果重新赋值
    std::vector<sfm::CameraPose> new_cam_poses(2);
    std::vector<math::Vec2f> radial_distortion(2);
    std::vector<math::Vec3f> new_pts_3d(pts_3d.size());
    for(int i=0; i<cams.size(); i++) {
        std::copy(cams[i].translation, cams[i].translation + 3, new_cam_poses[i].t.begin());
        std::copy(cams[i].rotation, cams[i].rotation + 9, new_cam_poses[i].R.begin());
        radial_distortion[i]=math::Vec2f(cams[i].distortion[0], cams[i].distortion[1]);
        new_cam_poses[i].set_k_matrix(cams[i].focal_length, 0.0, 0.0);
    }
    for(int i=0; i<new_pts_3d.size(); i++) {
        std::copy(p3ds[i].pos, p3ds[i].pos+3, new_pts_3d[i].begin());
    }

    // 输出优化信息
    std::cout<<"# Cam 0 #"<<std::endl;
    std::cout<<"Params before BA: "<<std::endl;
    std::cout<<"  f: "<<pose1.get_focal_length()<<std::endl;
    std::cout<<"  distortion: "<<0<<", "<<0<<std::endl;
    std::cout<<"  R: "<<pose1.R<<std::endl;
    std::cout<<"  t: "<<pose1.t<<std::endl;
    std::cout<<"Params after BA: "<<std::endl;
    std::cout<<"  f: "<<new_cam_poses[0].get_focal_length()<<std::endl;
    std::cout<<"  distortion: "<<radial_distortion[0][0]<<", "<<radial_distortion[0][1]<<std::endl;
    std::cout<<"  R: "<<new_cam_poses[0].R<<std::endl;
    std::cout<<"  t: "<<new_cam_poses[0] .t<<std::endl;


    // 输出优化信息
    std::cout<<"# Cam 1 #"<<std::endl;
    std::cout<<"Params before BA: "<<std::endl;
    std::cout<<"  f: "<<pose2.get_focal_length()<<std::endl;
    std::cout<<"  distortion: "<<0<<", "<<0<<std::endl;
    std::cout<<"  R: "<<pose2.R<<std::endl;
    std::cout<<"  t: "<<pose2.t<<std::endl;
    std::cout<<"Params after BA: "<<std::endl;
    std::cout<<"  f: "<<new_cam_poses[1].get_focal_length()<<std::endl;
    std::cout<<"  distortion: "<<radial_distortion[1][0]<<", "<<radial_distortion[1][1]<<std::endl;
    std::cout<<"  R: "<<new_cam_poses[1].R<<std::endl;
    std::cout<<"  t: "<<new_cam_poses[1] .t<<std::endl;

    std::ofstream fold_pts_3d("pts_3d.txt");
    std::ofstream fnew_pts_3d("new_pts_3d.txt");
    std::cout<<"points 3d: "<<std::endl;
    for(int i=0; i<pts_3d.size(); i++) {
        std::cout<<"( "<<pts_3d[i][0]<<", "<<pts_3d[i][1]<<", "<<pts_3d[i][2]<<" )";
        std::cout<<"-->";
        std::cout<<"( "<<new_pts_3d[i][0]<<", "<<new_pts_3d[i][1]<<", "<<new_pts_3d[i][2]<<" )"<<std::endl;
        fold_pts_3d << pts_3d[i][0]<<" "<<pts_3d[i][1]<<" "<<pts_3d[i][2] << "\n";
        fnew_pts_3d << new_pts_3d[i][0]<<" "<<new_pts_3d[i][1]<<" "<<new_pts_3d[i][2] << "\n";
    }
    fold_pts_3d.close();
    fnew_pts_3d.close();

    std::vector<double> rps;
    //注意p3ds是BA優化過的
    reproject(rps, cams, p3ds, observations);

    std::vector<core::ByteImage::Ptr> reprojected_imgs(2);
    reprojected_imgs[0] = core::ByteImage::Ptr(img1);
    reprojected_imgs[1] = core::ByteImage::Ptr(img2);
    for(int i = 0; i < corrs_filtered.size(); ++i){
        // denormalize image coordinates
        double p1u = corrs_filtered[i].p1[0];
        double p1v = corrs_filtered[i].p1[1];
        double p2u = corrs_filtered[i].p2[0];
        double p2v = corrs_filtered[i].p2[1];
        denormalize(f1norm, f1width, f1height, p1u, p1v);
        denormalize(f2norm, f2width, f2height, p2u, p2v);

        // std::cout << "corr: (" << p1u << ", " << p1v << 
        //     ") <-> (" << p2u << ", " << p2v << ")" << std::endl;
        
        core::image::draw_circle<uint8_t>(*reprojected_imgs[0], 
            p1u, p1v, 3, color_rgb_red);
        core::image::draw_circle<uint8_t>(*reprojected_imgs[1], 
            p2u, p2v, 3, color_rgb_red);
        // 每一個corr對應兩個二維點
        p1u = rps[i*4];
        p1v = rps[i*4+1];
        p2u = rps[i*4+2];
        p2v = rps[i*4+3];
        denormalize(f1norm, f1width, f1height, p1u, p1v);
        denormalize(f2norm, f2width, f2height, p2u, p2v);
        core::image::draw_circle<uint8_t>(*reprojected_imgs[0], 
            p1u, p1v, 2, color_rgb_blue);
        core::image::draw_circle<uint8_t>(*reprojected_imgs[1], 
            p2u, p2v, 2, color_rgb_blue);
    }
    core::image::save_file(reprojected_imgs[0], std::string("img1_reproject.jpg"));
    core::image::save_file(reprojected_imgs[1], std::string("img2_reproject.jpg"));

//    math::Matrix<double, 3, 4> P1, P2;
//    new_cam_poses[0].fill_p_matrix(&P1);
//    new_cam_poses[1].fill_p_matrix(&P2);
//    std::cout<<"P1: "<<P1;
//    std::cout<<"P2: "<<P2;

    return 0;
}
