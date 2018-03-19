#pragma once

#include <cassert>
#include <vector>
#include <algorithm>

template<typename C, typename F,
         typename R=std::vector<typename std::result_of<F(const typename C::value_type&)>::type>>
R to_vector(const C& c, F get) {
  R r;
  r.reserve(c.size());
  std::transform(begin(c), end(c), std::back_inserter(r), get);
  return r;
}

template<typename C>
std::vector<C> to_vector(const std::vector<float>&& x,
                         const std::vector<float>&& y) {
  assert(x.size() == y.size());
  std::vector<C> r(x.size());
  std::transform(begin(x), end(x), begin(y), begin(r),
                 [](float x, float y) -> C { return C{x, y}; });
  return r;
}
