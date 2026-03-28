#include "octvox.hpp"

#include <chrono>

using namespace std;

template<typename PointType, typename Scalar>
void OctVoxMap<PointType, Scalar>::insert(const PointVector& cloud_world){
  if(reset_map_){
    reset_map_count_--;
    if(reset_map_count_ > 0){
      std::cout << "OctVoxMap::insert skip: reset_map_count_ = " << reset_map_count_ << std::endl;
      return;
    } 
    reset_map_ = false;
  }

  for(auto& pt : cloud_world){
    KEY fine_key = (pt.getVector3fMap() * sub_inv_resolution_).array().floor().template cast<int>();
    KEY key;
    key[0] = fine_key[0] >> 1;
    key[1] = fine_key[1] >> 1;
    key[2] = fine_key[2] >> 1;

    uint8_t dx = fine_key[0] & 1;
    uint8_t dy = fine_key[1] & 1;
    uint8_t dz = fine_key[2] & 1;
    uint8_t local_idx = (dz << 2) | (dy << 1) | dx;

    auto iter = grids_.find(key);
    if (iter == grids_.end()) {
      data_.emplace_front(std::piecewise_construct,
        std::forward_as_tuple(key),
        std::forward_as_tuple(pt, local_idx));
      grids_.insert(std::make_pair(key, data_.begin()));
      
      if (data_.size() >= capacity_) {
        grids_.erase(data_.back().first);
        data_.pop_back();
      }
    } else {
      iter->second->second.AddPoint(pt, local_idx);
      data_.splice(data_.begin(), data_, iter->second);
    }
  }
}

template<typename PointType, typename Scalar>
void OctVoxMap<PointType, Scalar>::getTopK(const PointType& point, KNNHeapType& top_K) const {
  const KEY fine_key = (point.getVector3fMap() * sub_inv_resolution_).array().floor().template cast<int>();
  KEY key;
  key[0] = fine_key[0] >> 1;
  key[1] = fine_key[1] >> 1;
  key[2] = fine_key[2] >> 1;

  const int dx = fine_key[0] & 1;
  const int dy = fine_key[1] & 1;
  const int dz = fine_key[2] & 1;
  const int local_idx = (dz << 2) | (dy << 1) | dx;
  const KEY mirror_axis = KEY(1 - (dx << 1), 1 - (dy << 1), 1 - (dz << 1));
  
  const int pre_voxel_ptr_size = 8;
  OctVoxType* top_voxels_2_search[pre_voxel_ptr_size];
  std::fill_n(top_voxels_2_search, pre_voxel_ptr_size, nullptr);
  
  for(uint8_t i = 0; i < pre_voxel_ptr_size; ++i)
  {
    KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[i]);
    KEY n_key = key + delta_key;
    if (auto iter = grids_.find(n_key); iter != grids_.end()) {
      top_voxels_2_search[i] = &iter->second->second;
    }
  }

  PointType __sub_point;

  for (int group_idx = 0; group_idx < group_idx_max_; ++group_idx) {
    const uint8_t* group_it = flat_search_ptrs_[group_idx];
    const uint8_t* group_end = flat_search_ptrs_[group_idx + 1];

    while(group_it < group_end){
      const uint8_t neighbor_idx = *group_it++;
      uint8_t data_size = *group_it++;
      
      if(neighbor_idx < pre_voxel_ptr_size)
      {
        OctVoxType* voxel_ptr = top_voxels_2_search[neighbor_idx];
        if (voxel_ptr) {
          while (data_size--) {
            uint8_t _local_idx = (*group_it++)^local_idx;
            if (voxel_ptr->getPoint(_local_idx, __sub_point)) {
              const float dist2 = squaredNorm(__sub_point, point);
              top_K.try_insert(dist2, __sub_point);
            }
          }
        }
        else group_it+=data_size;
        continue;
      }

      KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[neighbor_idx]);
      const KEY n_key = key + delta_key;

      if (auto iter = grids_.find(n_key); iter != grids_.end()){
        OctVoxType* voxel_ptr = &iter->second->second;
        while (data_size--){
          const uint8_t _local_idx = (*group_it++)^local_idx;
          if (voxel_ptr->getPoint(_local_idx, __sub_point)) {
            const float dist2 = squaredNorm(__sub_point, point);
            top_K.try_insert(dist2, __sub_point);
          }
        }
      }
      else group_it+=data_size;
    }

    if (top_K.count == 5)
      if (top_K.max_dist2_ < orders_min_dis2[group_idx]){
        break;
      }

  }
}

template<typename PointType, typename Scalar>
void OctVoxMap<PointType, Scalar>::getTopK_VN(const PointType& point, KNNHeapType& top_K) const{
  KEY key = (point.getVector3fMap() * inv_resolution_).array().floor().template cast<int>();

  std::vector<OctVoxType*> voxels_2_search;
  voxels_2_search.reserve(19);
  for(std::size_t i = 0; i < 19; ++i) {
    KEY n_key = key + nearby_grids_[i];
    if (auto iter = grids_.find(n_key); iter != grids_.end()) {
      voxels_2_search.emplace_back(&iter->second->second);
    }
  }

  PointType pt;
  for(auto& voxel : voxels_2_search) {
    for(uint8_t _i = 0; _i < 8; ++_i) {
      if(!voxel->getPoint(_i, pt)) continue;
      const float dist2 = squaredNorm(pt, point);
      top_K.try_insert(dist2, pt);
    }
  }
}

template<typename PointType, typename Scalar>
void OctVoxMap<PointType, Scalar>::GetMap(PointVector& MapPoints) {
  MapPoints.clear();
  PointType point;
  for (const auto& voxel_pair : data_) {
    const OctVoxType& voxel = voxel_pair.second;
    for(uint8_t i = 0; i < 8; ++i) {
      if (!voxel.getPoint(i, point)) continue;
      MapPoints.push_back(point);
    }
  }
}

template<typename PointType, typename Scalar>
void OctVoxMap<PointType, Scalar>::Build(PointVector input) {
  if (input.empty()) return;
  Clear();

  insert(input);
  reset_map_ = true;
  reset_map_count_ = 10;
}

template<typename PointType, typename Scalar>
void OctVoxMap<PointType, Scalar>::Clear() {
  grids_.clear();
  data_.clear();
}

template<typename PointType, typename Scalar>
void OctVoxMap<PointType, Scalar>::SetOptions(float resolution, std::size_t capacity)
{
  resolution_ = resolution;
  capacity_ = capacity;
  inv_resolution_ = 1.0 / resolution_;
  sub_resolution_ = resolution_ / 2.0;
  sub_inv_resolution_ = 1.0 / sub_resolution_;
}

template<typename PointType, typename Scalar>
OctVoxMap<PointType, Scalar>::OctVoxMap(float resolution, std::size_t capacity) {
  SetOptions(resolution, capacity);
  flat_search_ptrs_.reserve(flat_search_order_offsets.size());
  for(std::size_t i = 0; i < flat_search_order_offsets.size(); i++){
    uint16_t start = flat_search_order_offsets[i];
    flat_search_ptrs_.push_back(const_cast<uint8_t*>(flat_search_order.data() + start));
  }
  group_idx_max_ = flat_search_order_offsets.size() - 1;
}

template<typename PointType, typename Scalar>
OctVoxMap<PointType, Scalar>::OctVoxMap() {
  flat_search_ptrs_.reserve(flat_search_order_offsets.size());
  for(std::size_t i = 0; i < flat_search_order_offsets.size(); i++){
    uint16_t start = flat_search_order_offsets[i];
    flat_search_ptrs_.push_back(const_cast<uint8_t*>(flat_search_order.data() + start));
  }
  group_idx_max_ = flat_search_order_offsets.size() - 1;
}

template<typename PointType, typename Scalar>
void OctVoxMap<PointType, Scalar>::Nearest_Search(
    const PointType& PointQuery, int k_nearest, 
    PointVector& Nearest_Points, std::vector<float>& Point_Distance, 
    float max_dist) {
  
  KNNHeapType top_K;
  getTopK(PointQuery, top_K);

  vector<float>().swap(Point_Distance);
  PointVector().swap(Nearest_Points);

  const int search_count = std::min(k_nearest, (int)top_K.count);
  const float max_dist2 = max_dist * max_dist;

  // TODO: add sort?
  
  for(int i = 0; i < search_count; ++i) {
    if(top_K.dist2_[i] > max_dist2) continue;
    Nearest_Points.emplace_back(top_K.points_[i]);
    Point_Distance.emplace_back(std::sqrt(top_K.dist2_[i]));
  }
}

template<typename PointType, typename Scalar>
int OctVoxMap<PointType, Scalar>::Delete_Point_Boxes(vector<BoxPointType> &BoxPoints) {
  int tmp_counter = 0;
  for (int i = 0; i < BoxPoints.size(); i++)
  {
    tmp_counter += Delete_by_range(BoxPoints[i]);
  }
  return tmp_counter;
}

template<typename PointType, typename Scalar>
int OctVoxMap<PointType, Scalar>::Delete_by_range(BoxPointType boxpoint){
  int tmp_counter = 0;
  for (auto iter = data_.begin(); iter != data_.end(); ) {
    OctVoxType& voxel = iter->second;
    bool voxel_deleted = false;
    for(uint8_t i = 0; i < 8; ++i) {
      PointType pt;
      if (!voxel.getPoint(i, pt)) continue;
      if (pt.x >= boxpoint.vertex_min[0] && pt.x <= boxpoint.vertex_max[0] &&
          pt.y >= boxpoint.vertex_min[1] && pt.y <= boxpoint.vertex_max[1] &&
          pt.z >= boxpoint.vertex_min[2] && pt.z <= boxpoint.vertex_max[2]) {
        voxel.DeletePoint(i);
        tmp_counter++;
      }
    }
    if (voxel.isEmpty()) {
      grids_.erase(iter->first);
      iter = data_.erase(iter);
    } else {
      ++iter;
    }
  }
  return tmp_counter;
}