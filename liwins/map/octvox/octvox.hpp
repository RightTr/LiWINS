#ifndef OCTVOX_HPP
#define OCTVOX_HPP

#include <set>
#include <list>
#include <queue>
#include <vector>
#include <memory>
#include <cstring>
#include <iostream>
#include <execution>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>

#include "tsl/robin_map.h"
#include "HKNN_list60_gem.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "../pointMapBase.hpp"

template<typename PointType>
float squaredNorm(const PointType& pt1, const PointType& pt2) {
  return (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) + (pt1.z - pt2.z) * (pt1.z - pt2.z);
}

template<int K, typename PointType>
class KNNHeap {
public:
  KNNHeap() : count(0), worst_(0), max_dist2_(0.0f) {
    memset(dist2_, 0, sizeof(dist2_));
  }

  void reset() {
    count = 0;
    worst_ = 0;
    max_dist2_ = 0.0f;
    memset(dist2_, 0, sizeof(dist2_));
  }

  uint8_t count;
  uint8_t worst_;
  float max_dist2_;
  float dist2_[K];
  std::array<PointType, K> points_;

  inline void try_insert(float dist2, const PointType& pt) {
    const bool not_full = (count < K);
    const bool should_insert = not_full || (dist2 < max_dist2_);
    
    if (should_insert) {
      const uint8_t insert_idx = not_full ? count : worst_;
      
      dist2_[insert_idx] = dist2;
      points_[insert_idx] = pt;
      
      if (not_full) {
        count++;
        if (dist2 > max_dist2_) {
          max_dist2_ = dist2;
          worst_ = insert_idx;
        }
      } else {
        update_worst_unrolled();
      }
    }
  }

private:
  inline void update_worst_unrolled() {
    float d0 = dist2_[0], d1 = dist2_[1], d2 = dist2_[2], d3 = dist2_[3], d4 = dist2_[4];
    
    uint8_t idx01 = d0 > d1 ? 0 : 1;
    float max01 = d0 > d1 ? d0 : d1;
    
    uint8_t idx23 = d2 > d3 ? 2 : 3;
    float max23 = d2 > d3 ? d2 : d3;
    
    uint8_t idx0123 = max01 > max23 ? idx01 : idx23;
    float max0123 = max01 > max23 ? max01 : max23;
    
    worst_ = max0123 > d4 ? idx0123 : 4;
    max_dist2_ = max0123 > d4 ? max0123 : d4;
  }

public:
  inline float max_dist2() const { return max_dist2_; }
};


template<typename PointType>
class OctVox{
public:
  OctVox(const PointType& pt, uint8_t local_idx)
  {
    counts_.fill(UNINIT_MASK);
    points_[local_idx] = pt;
    counts_[local_idx] = 1;
  }

  ~OctVox() {}

  void AddPoint(const PointType& pt, uint8_t local_idx) {
    uint8_t& count = counts_[local_idx];
    PointType& stored_point = points_[local_idx];
    if(count == UNINIT_MASK) {
      stored_point = pt;
      count = 1;
      return;
    }

    if(count >= MAX_POINTS_PER_SUBVOXEL) return;
    if (squaredNorm(pt, stored_point) > DISTANCE_THRESHOLD_SQ) return;

    Eigen::Vector3f result = (stored_point.getVector3fMap() * count + pt.getVector3fMap()) / (count + 1);
    stored_point.x = result.x();
    stored_point.y = result.y();
    stored_point.z = result.z();
    ++count;
  }

  bool getPoint(const uint8_t local_idx, PointType& pt) const {
    if (counts_[local_idx] == UNINIT_MASK) return false;
    pt = points_[local_idx];
    return true;
  }

  void DeletePoint(const uint8_t local_idx) {
    counts_[local_idx] = UNINIT_MASK;
  }

  bool isEmpty() const {
    for (const auto& count : counts_) {
      if (count != UNINIT_MASK) return false;
    }
    return true;
  }

  static constexpr uint8_t UNINIT_MASK = 0x00;
  static constexpr uint8_t MAX_POINTS_PER_SUBVOXEL = 20;
  static constexpr double DISTANCE_THRESHOLD_SQ = 0.1 * 0.1;

  std::array<uint8_t, 8> counts_;
  std::array<PointType, 8> points_;
};

template<typename PointType, typename Scalar>
class OctVoxMap : public ::PointMapBase<PointType> {
public:
  using Ptr = std::shared_ptr<OctVoxMap>;
  using KEY = Eigen::Vector3i;
  using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
  using KNNHeapType = KNNHeap<5, PointType>;
  using OctVoxType = OctVox<PointType>;
  using MapConfig = typename ::PointMapBase<PointType>::MapConfig;

  OctVoxMap();
  OctVoxMap(float resolution, std::size_t capacity);

  ~OctVoxMap() {
    grids_.clear();
    data_.clear();
  }
  void SetOptions(float resolution, std::size_t capacity);

  void insert(const PointVector& cloud_world);
  void resetMap(const std::vector<float>&);
  void Clear() override;

  void getTopK(const PointType& point, KNNHeapType& top_K) const;
  void getTopK_VN(const PointType& point, KNNHeapType& top_K) const;

  int Delete_by_range(BoxPointType boxpoint);

  void GetMap(PointVector& MapPoints) override;

  void reset_max_group(){
    group_idx_max_ = flat_search_order_offsets.size() - 1;
  }

  void decrease_max_group(){
    if(group_idx_max_ > 4) group_idx_max_--;
  }

  void InsertPoints(PointVector& Points, bool downsample = false) override {
    insert(Points);
  }

  void Nearest_Search(const PointType& PointQuery, 
      int k_nearest, PointVector& Nearest_Points, 
      std::vector<float>& Point_Distance, 
      float max_dist = INFINITY) override;

  void Build(PointVector input) override;

  int Size() override {
    return data_.size();
  }

  bool IsInitialized() override {
    return !data_.empty();
  }

  int Delete_Point_Boxes(std::vector<BoxPointType> &BoxPoints) override;

  void SetConfig(const MapConfig& config) override {
    SetOptions(config.resolution_, config.capacity_);
  }

private:
  float resolution_ = 0.5;
  float inv_resolution_ = 1.0;
  float sub_resolution_ = 0.25;
  float sub_inv_resolution_ = 4.0;
  std::size_t capacity_ = 1000000;

  bool reset_map_ = false;
  int reset_map_count_ = 0;

  const KEY nearby_grids_[19] = {
    KEY(0, 0, 0),
    KEY(-1, -1, 0), KEY(-1, 0, 0), KEY(-1, 1, 0), 
    KEY(0, -1, 0), KEY(0, 1, 0), 
    KEY(1, -1, 0), KEY(1, 0, 0), KEY(1, 1, 0), 
    KEY(0, 0, -1), KEY(1, 0, -1), KEY(-1, 0, -1), 
    KEY(0, 1, -1), KEY(0, -1, -1), 
    KEY(0, 0, 1), KEY(1, 0, 1), KEY(-1, 0, 1), 
    KEY(0, 1, 1), KEY(0, -1, 1)
  };

  /// HashShiftMix
  struct HASH_VEC {
    std::size_t operator()(const KEY &v) const {
      size_t h = static_cast<size_t>(v[0]);
      h ^= v[1] * 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= v[2] * 0x85ebca6b + (h << 6) + (h >> 2);
      return h;
    }
  };

  using DATA_LIST = std::list<std::pair<KEY, OctVoxType>>;
  using DATA_ITER = typename DATA_LIST::iterator;

  DATA_LIST data_;
  tsl::robin_map<KEY, DATA_ITER, HASH_VEC> grids_;

  std::vector<uint8_t*> flat_search_ptrs_;
  int group_idx_max_;

};

template class OctVoxMap<pcl::PointXYZ, float>;
template class OctVoxMap<pcl::PointXYZI, float>;
template class OctVoxMap<pcl::PointXYZINormal, float>;
template class OctVoxMap<pcl::PointXYZ, double>;
template class OctVoxMap<pcl::PointXYZI, double>;
template class OctVoxMap<pcl::PointXYZINormal, double>;

#endif // OCTVOX_HPP