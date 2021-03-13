/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <stdexcept>
#include <list>
#include <iostream>
#include <algorithm>

#include "util/timer.h"
#include "core/mesh_io.h"
#include "surface/octree.h"

FSSR_NAMESPACE_BEGIN

Octree::Node*
Octree::Iterator::first_node (void) {

    // current node is set as the oactree root
    this->current = this->root;

    // level of the root node is 0
    this->level = 0;

    // path of the root node is 0
    this->path = 0;

    return this->current;
}

Octree::Node*
Octree::Iterator::first_leaf (void) {

    // move the iterator to the first node
    this->first_node();

    // if the current node has children
    while (this->current->children != nullptr) {
        // set the current node as the first chidren
        this->current = this->current->children;
        this->level = this->level + 1;
        // 一直往左邊找path會是0,應該不算bug
        this->path = this->path << 3; // equals to multiply it as 8. fixMe? path will always be 0!!
    }
    return this->current;
}

// depth first search
Octree::Node*
Octree::Iterator::next_node (void)
{
    // 拜訪順序:祖父-父親-子-父親的弟弟-父親的弟弟的兒子
    // if the current node has no children, then switch to next branch
    if (this->current->children == nullptr)
        return this->next_branch();

    this->current = this->current->children;
    this->level = this->level + 1;
    // 空出最後一位數,等會要填上?
    this->path = this->path << 3;
    return this->current;
}

// bread first search
Octree::Node*
Octree::Iterator::next_bread_first(void)
{
    /*todo implement bread first search here*/
    // 拜訪順序:祖父-父親-父親的弟弟-子-父親的弟弟的兒子
}

// 尋找同一層(?)的下一個sibling?
Octree::Node*
Octree::Iterator::next_branch (void) {

    // root node has no branch, nullptr will be returned
    // 如果是root就返回nullptr
    if (this->current->parent == nullptr) {
        this->current = nullptr;
        return nullptr;
    }

    // current node is the last child, then switch to the next
    // branch in a recursive way
    /*
    this->current->parent->children指向第一個孩子
    如果this->current與它的距離為7,表示它是parent的第8個孩子
    這時候要找父親的弟弟(不是父親的弟弟的孩子?)
    如果父親沒有弟弟,會找父親的父親的弟弟
    */
    if (this->current - this->current->parent->children == 7) {
        this->current = this->current->parent;
        this->level = this->level - 1;
        this->path = this->path >> 3;
        return this->next_branch();
    }

    // if the current node is the inner child, then the next branch
    // is indeed the next child
    // 找弟弟
    this->current += 1;
    this->path += 1;
    return this->current;
}

Octree::Node*
Octree::Iterator::next_leaf (void) {

    // the current node has any children
    if (this->current->children != nullptr)
    {
        while (this->current->children != nullptr)
        {
            // set the current node as the children
            this->current = this->current->children;
            this->level = this->level + 1;
            this->path = this->path << 3;
        }
        return this->current;
    }

    // if the current node is a leaf node, switch to next branch
    // 找弟弟或父親的弟弟
    this->next_branch();
    if (this->current == nullptr)
        return nullptr;
    while (this->current->children != nullptr) {
        this->current = this->current->children;
        this->level = this->level + 1;
        this->path = this->path << 3;
    }
    return this->current;
}

// visit the octant-th child
Octree::Iterator
Octree::Iterator::descend (int octant) const
{
    Iterator iter(*this);
    iter.current = iter.current->children + octant;
    iter.level = iter.level + 1;
    iter.path = (iter.path << 3) | octant;
    return iter;
}

Octree::Iterator
Octree::Iterator::descend (uint8_t level, uint64_t path) const
{
    Iterator iter;
    iter.root = this->root;
    iter.current = this->root;
    iter.path = 0;
    iter.level = 0;
    for (int i = 0; i < level; ++i) {

        if (iter.current->children == nullptr){
            iter.current = nullptr;
            return iter;
        }

        // fixme?? find the first child in the required level ??
        int const octant = (path >> ((level - i - 1) * 3)) & 7;
        iter = iter.descend(octant);
    }

    if (iter.path != path || iter.level != level)
        throw std::runtime_error("descend(): failed");

    return iter;
}

Octree::Iterator
Octree::Iterator::ascend (void) const
{
    Iterator iter;
    iter.root = this->root;
    // set the current as the parent
    iter.current = this->current->parent;
    iter.path = this->path >> 3; // divide it by 8
    iter.level = this->level - 1;
    return iter;
}

/* -------------------------------------------------------------------- */

void
Octree::insert_samples (SampleList const& samples) {

    // for each sample
    for (std::size_t i = 0; i < samples.size(); i++) {
        this->insert_sample(samples[i]);
    }
}

void
Octree::insert_sample (Sample const& sample)
{
    // creat a new octree
    if (this->root == nullptr) {
        this->root = new Node();

        // center of the root is set as the first sample's position
        this->root_center = sample.pos;

        // size of the root is set as the first sample' scale
        this->root_size = sample.scale;
        this->num_nodes = 1;
    }

    /* Expand octree root if sample is outside the octree. */
    while (!this->is_inside_octree(sample.pos)) {
        // double the size of the octree once a time, until the octree
        // contains the sample
        this->expand_root_for_point(sample.pos);
    }

    /* Find node by expanding the root or descending the tree. */
    // 尋找適合存放sample的node
    Node* node = nullptr;
    if (sample.scale >= this->root_size * 2.0)
        // 往尺度大的地方找
        node = this->find_node_expand(sample);
    else
        // 往尺度相當或較小的地方找
        node = this->find_node_descend(sample, this->get_iterator_for_root());

    node->samples.push_back(sample);
    this->num_samples += 1;
}

void
Octree::create_children (Node* node)
{
    if (node->children != nullptr)
        throw std::runtime_error("create_children(): Children exist!");
    node->children = new Node[8];
    this->num_nodes += 8;
    for (int i = 0; i < 8; ++i)
        node->children[i].parent = node;
}

bool
Octree::is_inside_octree (math::Vec3d const& pos) {

    double const len2 = this->root_size / 2.0;

    // whether pos is in a cube defined by the root
    for (int i = 0; i < 3; ++i)
        if (pos[i] < this->root_center[i] - len2
            || pos[i] > this->root_center[i] + len2)
            return false;
    return true;
}

void
Octree::expand_root_for_point (math::Vec3d const& pos)
{
    /* Compute old root octant and new root center and size. */
    // current root is the octant child of the new create root
    int octant = 0;
    for (int i = 0; i < 3; ++i) {
        // 往pos[i]所在的方向擴張
        if (pos[i] > this->root_center[i]) {
            this->root_center[i] += this->root_size / 2.0;
        } else {
            // 往左邊擴張,所以老octant是新octant右邊的孩子,octant在第i個維度的值為1
            octant |= (1 << i);
            this->root_center[i] -= this->root_size / 2.0;
        }
    }

    this->root_size *= 2.0;

    /* Create new root. */
    Node* new_root = new Node();
    this->create_children(new_root);
    // 把當前root所擁有的children,samples複製給new_root的第octant個孩子
    std::swap(new_root->children[octant].children, this->root->children);
    std::swap(new_root->children[octant].samples, this->root->samples);
    delete this->root;
    // this->root現在變為剛剛設好的new_root
    this->root = new_root;

    /* Fix parent pointers of old child nodes. */
    if (this->root->children[octant].children != nullptr)
    {
        // 當前節點的孩子
        Node* children = this->root->children[octant].children;
        // 當前節點的孩子的父親是當前節點
        for (int i = 0; i < 8; ++i)
            children[i].parent = this->root->children + octant;
    }
}

Octree::Node*
Octree::find_node_descend (Sample const& sample, Iterator const& iter)
{
    // valid when: sample.scale小於等於node_size * 2.0
    math::Vec3d node_center;
    double node_size;
    this->node_center_and_size(iter, &node_center, &node_size);

    // scale那麼大就不必往下找
    if (sample.scale > node_size * 2.0)
        throw std::runtime_error("find_node_descend(): Sanity check failed!");

    /*
     * The current level l is appropriate if sample scale s is
     * scale(l) <= s < scale(l) * 2. As a sanity check, this function
     * must not be called if s >= scale(l) * 2. If the current level is
     * the maximum allowed level, return this node also. Descend otherwise.
     */
    // 如果s大於node_size,停在這個level就行,不必繼續往下找
    // 如果iter.level太深,也是停在這個level
    if (node_size <= sample.scale || iter.level >= this->max_level)
        return iter.current;

    /* Descend octree. Find octant and create children if required. */
    // 還要繼續往下,所以先創造它的孩子
    // octant: 第幾個孩子,由sample.pos與node_center的相對位置來決定
    int octant = 0;
    for (int i = 0; i < 3; ++i)
        if (sample.pos[i] > node_center[i])
            octant |= (1 << i);
    if (iter.current->children == nullptr)
        this->create_children(iter.current);
    return this->find_node_descend(sample, iter.descend(octant));
}

Octree::Node*
Octree::find_node_expand (Sample const& sample)
{
    // 如果sample.scale比較小,就用不著expand
    if (this->root_size > sample.scale)
        throw std::runtime_error("find_node_expand(): Sanity check failed!");

    /*
     * The current level l is appropriate if sample scale s is
     * scale(l) <= scale < scale(l) * 2. As a sanity check, this function
     * must not be called if scale(l) > s. Otherwise expand.
     */
    // 前面已經確定了sample.scale大於等於this->root_size,
    // 即scale >= scale(l)
    if (sample.scale < this->root_size * 2.0)
        return this->root;

    // 到這邊sample.scale >= this->root_size * 2.0,
    // 所以要expand
    this->expand_root_for_point(sample.pos);
    // recursive地往外找
    return this->find_node_expand(sample);
}

int
Octree::get_num_levels (Node const* node) const
{
    if (node == nullptr)
        return 0;
    if (node->children == nullptr)
        return 1;
    int depth = 0;
    for (int i = 0; i < 8; ++i)
        depth = std::max(depth, this->get_num_levels(node->children + i));
    return depth + 1;
}

void
Octree::get_samples_per_level (std::vector<std::size_t>* stats,
    Node const* node, std::size_t level) const
{
    if (node == nullptr)
        return;
    if (stats->size() <= level)
        stats->resize(level + 1, 0);
    stats->at(level) += node->samples.size();

    /* Descend into octree. */
    if (node->children == nullptr)
        return;
    for (int i = 0; i < 8; ++i)
        this->get_samples_per_level(stats, node->children + i, level + 1);
}

void
Octree::node_center_and_size (Iterator const& iter,
    math::Vec3d* center, double *size) const
{
    // 獲取iter所指向的octant處的center及size
    *center = this->root_center;
    *size = this->root_size;
    for (int i = 0; i < iter.level; ++i)
    {
        // 每一個level佔用3個bit來表示path
        // iter.level - i - 1不會變負的?
        // octant表示iter.path在第i個level的序號?
        int const octant = iter.path >> ((iter.level - i - 1) * 3);
        // 除以4?
        double const offset = *size / 4.0;
        for (int j = 0; j < 3; ++j)
            // ((octant & (1 << j)): octant在第j個維度上是在左還是右
            (*center)[j] += ((octant & (1 << j)) ? offset : -offset);
        *size /= 2.0;
    }
}

Octree::Iterator
Octree::get_iterator_for_root (void) const
{
    if (this->root == nullptr)
        throw std::logic_error("Iterator request on empty octree");

    Iterator iter;
    iter.root = this->root;
    iter.first_node();
    return iter;
}

void
Octree::influence_query (math::Vec3d const& pos, double factor,
    std::vector<Sample const*>* result, Iterator const& iter,
    math::Vec3d const& parent_node_center) const
{
    if (iter.current == nullptr)
        return;

    /*
     * Strategy is the following: Try to rule out this octree node. Assume
     * the largest scale sample (node_size * 2) in this node and compute
     * an estimate for the closest possible distance of any sample in the
     * node to the query. If 'factor' times the largest scale is less than
     * the closest distance, the node can be skipped and traversal stops.
     * Otherwise all samples in the node have to be tested.
     *
     * - Note: The 'factor' depends on the basis/weighting function. In this
     *   implementation, factor is always 3.0.
     * - Note: Nodes can contain samples with scale values much smaller than
     *   node_size. This is because the octree depth is limited.
     */
    /* Compute current node center based on parent's. */
    uint32_t x = (iter.path & 1) >> 0;
    uint32_t y = (iter.path & 2) >> 1;
    uint32_t z = (iter.path & 4) >> 2;
    double node_size = this->root_size / (1 << iter.level);
    double offset = (iter.level > 0) * node_size / 2.0;
    math::Vec3d node_center(
        parent_node_center[0] - offset + x * node_size,
        parent_node_center[1] - offset + y * node_size,
        parent_node_center[2] - offset + z * node_size);

    /* Estimate for the minimum distance. No sample is closer to pos. */
    double const min_distance = (pos - node_center).norm() - MATH_SQRT3 * node_size / 2.0;
    double const max_scale = node_size * 2.0;
    if (min_distance > max_scale * factor)
        return;

    /* Node could not be ruled out. Test all samples. */
    for (std::size_t i = 0; i < iter.current->samples.size(); ++i)
    {
        Sample const& s = iter.current->samples[i];
        if ((pos - s.pos).square_norm() > MATH_POW2(factor * s.scale))
            continue;
        result->push_back(&s);
    }

    /* Descend into octree. */
    if (iter.current->children == nullptr)
        return;
    for (int i = 0; i < 8; ++i)
        this->influence_query(pos, factor, result, iter.descend(i),
            node_center);
}

void
Octree::refine_octree (void)
{
    // 希望octree是full的
    if (this->root == nullptr)
        return;

    std::list<Node*> queue;
    queue.push_back(this->root);
    while (!queue.empty())
    {
        Node* node = queue.front();
        queue.pop_front();

        if (node->children == nullptr)
            // 對於octree中的inner node要加入孩子
            this->create_children(node);
        else
            for (int i = 0; i < 8; ++i)
                queue.push_back(node->children + i);
    }
}

void
Octree::limit_octree_level (void)
{
    std::cout << "Limiting octree to "
        << this->max_level << " levels..." << std::endl;

    if (this->root == nullptr)
        return;
    this->limit_octree_level(this->root, nullptr, 0);
}

void
Octree::limit_octree_level (Node* node, Node* parent, int level)
{
    if (level == this->max_level)
        // 這樣在下一次遞迴的時候它的孩子才會把samples送給它
        parent = node;

    if (level > this->max_level)
    {
        // 把node的samples都移到parent那邊去
        parent->samples.insert(parent->samples.end(),
            node->samples.begin(), node->samples.end());
        node->samples.clear();
    }

    if (node->children != nullptr)
        for (int i = 0; i < 8; ++i)
            // 限制每個孩子的level
            this->limit_octree_level(node->children + i, parent, level + 1);

    // 為何不跟上面合併?
    if (level > this->max_level)
        this->num_nodes -= 1;

    // node是最後一層,不應該有小孩
    if (level == this->max_level)
    {
        delete [] node->children;
        node->children = nullptr;
    }
}

void
Octree::print_stats (std::ostream& out)
{
    out << "Octree contains " << this->get_num_samples()
        << " samples in " << this->get_num_nodes() << " nodes on "
        << this->get_num_levels() << " levels." << std::endl;

    std::vector<std::size_t> octree_stats;
    this->get_samples_per_level(&octree_stats);

    bool printed = false;
    for (std::size_t i = 0; i < octree_stats.size(); ++i)
    {
        if (!printed && octree_stats[i] == 0)
            continue;
        else
        {
            out << "  Level " << i << ": "
                << octree_stats[i] << " samples" << std::endl;
            printed = true;
        }
    }
}

FSSR_NAMESPACE_END
