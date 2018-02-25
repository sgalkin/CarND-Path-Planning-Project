#pragma once

#include <type_traits>
#include <utility>

template <typename H, typename... T>
class Compose {
private:
  H head_;
  Compose<T...> tail_;

public:
  Compose() : head_(H()), tail_(T()...) {}  
  Compose(H h, T... t) : head_(h), tail_(t...) {}

  template <typename... Args>
  auto operator() (Args&&... args) ->
    typename std::result_of<
      H(decltype(tail_(std::forward<Args&&...>(args...))))
    >::type {
    return head_(tail_(std::forward<Args&&...>(args...)));
  }
};


template <typename F>
class Compose<F> {
private:
  F f_;

public:
  Compose(F f = F()) : f_(f) {}

  template <typename... Args>
  auto operator() (Args&&... args) ->
    typename std::result_of<F(Args&&...)>::type {
    return f_(std::forward<Args&&...>(args...));
  }
};


template <typename... F>
Compose<F...> compose(F... f) {
  return Compose<F...>(f...);
}
