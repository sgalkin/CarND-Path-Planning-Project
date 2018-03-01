#pragma once

#include <numeric>
#include <cassert>
#include <limits>
#include <tuple>
#include <array>

#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "nanoflann.hpp"
#pragma GCC diagnostic pop

#include "point.h"

class Map {
  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, Map>, Map, 2>;

public:
  using Record = std::tuple<Point, float, Point>;
  using const_iterator = std::vector<Point>::const_iterator;
  using const_reverse_iterator = std::vector<Point>::const_reverse_iterator;
  
  template<typename I>
  Map(I begin, I end)
    : index_{2 /*dim*/, *this, nanoflann::KDTreeSingleIndexAdaptorParams(8 /* max leaf */)} {
    float s_max = -std::numeric_limits<float>::infinity();
    std::for_each(begin, end, [this, &s_max](const typename I::value_type& v) {
        Point wp;
        float f;
        Point n;
        std::tie(wp, f, n) = v;
        waypoints_.emplace_back(std::move(wp));
        if(f < s_max) {
          throw std::runtime_error("bad file, s-values inconsistent");
        }
        s_max = f;
        s_.emplace_back(f);
        normals_.emplace_back(std::move(n));
    });
    assert(waypoints_.size() == s_.size());
    assert(s_.size() == normals_.size());
    
    auto c = std::accumulate(std::begin(waypoints_), std::end(waypoints_),
                             Point{0, 0},
                             [](Point a, Point p) {
                               return Point{a.x + p.x, a.y + p.y};
                             });
    center_ = {c.x/waypoints_.size(), c.y/waypoints_.size()};
    index_.buildIndex();
  }

  size_t size() const { return waypoints_.size(); }

  const_iterator begin() const { return waypoints_.begin(); }
  const_iterator end() const { return waypoints_.end(); }

  const_reverse_iterator rbegin() const { return waypoints_.rbegin(); }
  const_reverse_iterator rend() const { return waypoints_.rend(); }

  Point center() const { return center_; }
  Point waypoint(size_t i) const { return waypoints_[i]; }
  float s(size_t i) const { return s_[i]; }
  
  size_t nearest(Point p) const {
    // do a knn search
    nanoflann::KNNResultSet<float> result{1};
    std::array<float, 2> q{{p.x, p.y}};
    size_t index{0};
    float distance{std::numeric_limits<float>::infinity()};
    result.init(&index, &distance);
    index_.findNeighbors(result, &q[0], nanoflann::SearchParams{10});
    return index;
  }

  size_t s_upper_bound(float s) const {
    auto it = std::upper_bound(std::begin(s_), std::end(s_), s);
    return it == std::end(s_) ? s_.size() - 1 : distance(std::begin(s_), it);
  }
  
public:
  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return waypoints_.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate value, the
  //  "if/else's" are actually solved at compile time.
  inline float kdtree_get_pt(const size_t idx, int dim) const {
    if(dim == 0) return waypoints_[idx].x;
    if(dim == 1) return waypoints_[idx].y;
    assert(false);
    return std::numeric_limits<float>::infinity();
  }

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }


private:
  std::vector<Point> waypoints_;
  std::vector<float> s_;
  std::vector<Point> normals_;
  Point center_;

  KDTree index_;
};

template<typename IS>
IS& operator>> (IS& is, Map::Record& rec) {
  return is >> std::get<0>(rec) >> std::get<1>(rec) >> std::get<2>(rec);
}
