/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <set>

#include "core/image_tools.h"
#include "core/image_drawing.h"
#include "sfm/bundler_tracks.h"

SFM_NAMESPACE_BEGIN
SFM_BUNDLER_NAMESPACE_BEGIN

/*
 * Merges tracks and updates viewports accordingly.
 */
void
unify_tracks(int view1_tid, int view2_tid,
    TrackList* tracks, ViewportList* viewports)
{
    /* Unify in larger track. */
    // 為何可以交換?
    if (tracks->at(view1_tid).features.size()
        < tracks->at(view2_tid).features.size())
        std::swap(view1_tid, view2_tid);

    Track& track1 = tracks->at(view1_tid);
    Track& track2 = tracks->at(view2_tid);

    for (std::size_t k = 0; k < track2.features.size(); ++k)
    {
        // FeatureReferenceList: std::vector<FeatureReference>
        // FeatureReference: 記錄view_id和feature_id
        int const view_id = track2.features[k].view_id;
        int const feat_id = track2.features[k].feature_id;
        // 把第view_id個view的第feat_id的track_id改成view1_tid
        viewports->at(view_id).track_ids[feat_id] = view1_tid;
    }
    // 把track2.features併入track1.features
    track1.features.insert(track1.features.end(),
        track2.features.begin(), track2.features.end());
    /* Free old track's memory. clear() does not work. */
    // features: std::vector<FeatureReference>, FeatureReference: struct
    track2.features = FeatureReferenceList();
}

/* ---------------------------------------------------------------- */

// 輸入: matchings, viewports
// 輸出: viewports, tracks
void
Tracks::compute (PairwiseMatching const& matching,
    ViewportList* viewports, TrackList* tracks)
{
    /* Initialize per-viewport track IDs. */
    for (std::size_t i = 0; i < viewports->size(); ++i)
    {
        Viewport& viewport = viewports->at(i);
        viewport.track_ids.resize(viewport.features.positions.size(), -1);
    }

    /* Propagate track IDs. */
    if (this->opts.verbose_output)
        std::cout << "Propagating track IDs..." << std::endl;

    /* Iterate over all pairwise matchings and create tracks. */
    tracks->clear();
    for (std::size_t i = 0; i < matching.size(); ++i)
    {
        // 第i對用來做匹配的圖像
        TwoViewMatching const& tvm = matching[i];
        Viewport& viewport1 = viewports->at(tvm.view_1_id);
        Viewport& viewport2 = viewports->at(tvm.view_2_id);

        /* Iterate over matches for a pair. */
        // 遍歷兩個view間的所有匹配對
        for (std::size_t j = 0; j < tvm.matches.size(); ++j)
        {
            CorrespondenceIndex idx = tvm.matches[j];
            // viewport1的第idx.first個特徵點的track id
            int const view1_tid = viewport1.track_ids[idx.first];
            int const view2_tid = viewport2.track_ids[idx.second];
            // 兩個特徵點的track id都是-1表示它們都還沒被記錄到tracks裡
            if (view1_tid == -1 && view2_tid == -1)
            {
                /* No track ID associated with the match. Create track. */
                // track id從0開始,所以新加進來的track是第tracks->size()個
                viewport1.track_ids[idx.first] = tracks->size();
                viewport2.track_ids[idx.second] = tracks->size();
                tracks->push_back(Track());
                // 記錄屬於這個track的這兩個特徵的view_id及feature_id
                tracks->back().features.push_back(
                    FeatureReference(tvm.view_1_id, idx.first));
                tracks->back().features.push_back(
                    FeatureReference(tvm.view_2_id, idx.second));
            }
            else if (view1_tid == -1 && view2_tid != -1)
            {
                /* Propagate track ID from first to second view. */
                /*
                view2裡的feature已經屬於某個track了,
                這裡再把view1裡的feature加進來
                */
                viewport1.track_ids[idx.first] = view2_tid;
                tracks->at(view2_tid).features.push_back(
                    FeatureReference(tvm.view_1_id, idx.first));
            }
            else if (view1_tid != -1 && view2_tid == -1)
            {
                /* Propagate track ID from second to first view. */
                viewport2.track_ids[idx.second] = view1_tid;
                tracks->at(view1_tid).features.push_back(
                    FeatureReference(tvm.view_2_id, idx.second));
            }
            else if (view1_tid == view2_tid)
            {
                /* Track ID already propagated. */
            }
            else
            {
                /* 如果匹配的两个特征点对应的track id不一样，则将track进行融合
                 * A track ID is already associated with both ends of a match,
                 * however, is not consistent. Unify tracks.
                 */
                unify_tracks(view1_tid, view2_tid, tracks, viewports);
            }
        }
    }

    /* Find and remove invalid tracks or tracks with conflicts. */
    if (this->opts.verbose_output)
        std::cout << "Removing tracks with conflicts..." << std::flush;

    // 删除不合理的track(同一个track,包含同一副图像中的多个特征点）
    std::size_t const num_invalid_tracks = this->remove_invalid_tracks(viewports, tracks);
    if (this->opts.verbose_output)
        std::cout << " deleted " << num_invalid_tracks << " tracks." << std::endl;

    /* Compute color for every track. */
    if (this->opts.verbose_output)
        std::cout << "Colorizing tracks..." << std::endl;
    for (std::size_t i = 0; i < tracks->size(); ++i)
    {
        Track& track = tracks->at(i);
        math::Vec4f color(0.0f, 0.0f, 0.0f, 0.0f);
        for (std::size_t j = 0; j < track.features.size(); ++j)
        {
            FeatureReference const& ref = track.features[j];
            FeatureSet const& features = viewports->at(ref.view_id).features;
            math::Vec3f const feature_color(features.colors[ref.feature_id]);
            // 利用Vec4f的最後一個元素當counter
            color += math::Vec4f(feature_color, 1.0f);
        }
        // /color[3]:做平均, +0.5: 四捨五入
        track.color[0] = static_cast<uint8_t>(color[0] / color[3] + 0.5f);
        track.color[1] = static_cast<uint8_t>(color[1] / color[3] + 0.5f);
        track.color[2] = static_cast<uint8_t>(color[2] / color[3] + 0.5f);
    }
}

/* ---------------------------------------------------------------- */

int
Tracks::remove_invalid_tracks (ViewportList* viewports, TrackList* tracks)
{
    /*
     * Detect invalid tracks where a track contains no features, or
     * multiple features from a single view.
     */
    // 删除tracks没有特征点，或者是包含同一幅图像中的多个特征点
    std::vector<bool> delete_tracks(tracks->size());
    int num_invalid_tracks = 0;
    for (std::size_t i = 0; i < tracks->size(); ++i)
    {
        if (tracks->at(i).features.empty()) {
            delete_tracks[i] = true;
            continue;
        }

        std::set<int> view_ids;
        for (std::size_t j = 0; j < tracks->at(i).features.size(); ++j)
        {
            FeatureReference const& ref = tracks->at(i).features[j];
            // 這個view_id已經在tracks->at(i)裡出現過了
            if (view_ids.insert(ref.view_id).second == false) {
                num_invalid_tracks += 1;
                delete_tracks[i] = true;
                break;
            }
        }
    }

    /* Create a mapping from old to new track IDs. */
    // 例:把[0,1,d,3,d,5]對應到[0,1,2,3]
    std::vector<int> id_mapping(delete_tracks.size(), -1);
    int valid_track_counter = 0;
    for (std::size_t i = 0; i < delete_tracks.size(); ++i)
    {
        if (delete_tracks[i])
            continue;
        id_mapping[i] = valid_track_counter;
        valid_track_counter += 1;
    }

    /* Fix track IDs stored in the viewports. */
    for (std::size_t i = 0; i < viewports->size(); ++i)
    {
        std::vector<int>& track_ids = viewports->at(i).track_ids;
        for (std::size_t j = 0; j < track_ids.size(); ++j)
            if (track_ids[j] >= 0)
                track_ids[j] = id_mapping[track_ids[j]];
    }

    /* Clean the tracks from the vector. */
    math::algo::vector_clean(delete_tracks, tracks);

    return num_invalid_tracks;
}


SFM_BUNDLER_NAMESPACE_END
SFM_NAMESPACE_END
