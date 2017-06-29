/*
 * map.h
 *
 *  Created on: Dec 12, 2016
 *      Author: mufferm
 *  Updated on: June 27, 2017
 *      Author: Artem Artemev, @artemav
 */

#ifndef MAP_H_
#define MAP_H_

#include <cassert>
#include <iostream>
#include <vector>

#include "helper_functions.h"
#include "nanoflann.hpp"

using nanoflann::KDTreeSingleIndexAdaptorParams;
using nanoflann::KDTreeSingleIndexAdaptor;
using nanoflann::KNNResultSet;
using nanoflann::SearchParams;
using nanoflann::L2_Adaptor;

using KDTreeParams = KDTreeSingleIndexAdaptorParams;

struct Landmark {
  // Landmark x-position in the map (global coordinates)
  double x;
  // Landmark y-position in the map (global coordinates)
  double y;
  // Landmark ID
  int id;
};

class MapLandmarks {
public:
  static const int dim = 2;

  MapLandmarks() {}
  ~MapLandmarks() {}

  template <typename Distance, typename DatasetAdaptor, int DIM, typename IndexType>
  friend class KDTreeSingleIndexAdaptor;

  template <typename T, typename DataSource, typename _DistanceType>
  friend struct L2_Adaptor;

  void AddLandmark(double x, double y, int id = 0) {
    landmarks_.push_back(Landmark{x, y, id});
  }

  const Landmark &operator[](size_t id) const { return landmarks_[id]; }

  void Print() const {
    for (const auto &landmark : landmarks_) {
      std::cout << landmark.id << ","
                << landmark.x << ","
                << landmark.y << "\n";
    }
  }

protected:
  size_t kdtree_get_point_count() const {
    return landmarks_.size();
  }

  double kdtree_distance(
      const double pt[dim],
      const size_t pt_idx,
      size_t) const {
    assert(pt_idx >= 0 && pt_idx < landmarks_.size());
    const double dx = pt[0] - landmarks_[pt_idx].x;
    const double dy = pt[1] - landmarks_[pt_idx].y;
    return dx * dx + dy * dy;
  }

  double kdtree_get_pt(const size_t idx, int idx_dim) const {
    assert(idx_dim == 0 || idx_dim == 1);
    if (idx_dim == 0) {
      return landmarks_[idx].x;
    }
    return landmarks_[idx].y;
  }

  template <typename Bbox> bool kdtree_get_bbox(Bbox &) const { return false; }

private:
  std::vector<Landmark> landmarks_;
};

class Map {
public:
  explicit Map(const MapLandmarks &marks)
      : landmarks_(marks), map_(MapLandmarks::dim, landmarks_, KDTreeParams()) {
    map_.buildIndex();
  }

  void Print() const { landmarks_.Print(); }

  bool SearchNearestLandmark(double x, double y, Landmark *landmark) const {
    static const int num_results = 1;
    const double pt[2] = {x, y};
    KNNResultSet<double> result{num_results};
    size_t id = 0;
    double dist_sq = 0;
    result.init(&id, &dist_sq);
    bool found = map_.findNeighbors(result, pt, SearchParams());
    if (!found) {
      return false;
    }
    *landmark = landmarks_[id];
    return true;
  }

private:
  using MapDistance = L2_Adaptor<double, MapLandmarks>;
  using KdTree_ =
      KDTreeSingleIndexAdaptor<MapDistance, MapLandmarks, MapLandmarks::dim>;
  MapLandmarks landmarks_;
  KdTree_ map_;
};

#endif /* MAP_H_ */
