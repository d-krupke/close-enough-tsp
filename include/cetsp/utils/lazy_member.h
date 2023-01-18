//
// Created by Dominik Krupke on 13.01.23.
//

#ifndef CETSP_LAZY_MEMBER_H
#define CETSP_LAZY_MEMBER_H
#include "doctest/doctest.h"
#include <functional>
#include <optional>
namespace cetsp::utils {

/**
 * The Lazy-class shall make the implementation of lazy evaluation much easier.
 * @tparam T The type to be lazy evaluated.
 */
template <typename T> class Lazy {
public:
  Lazy() = default;

  explicit Lazy(std::function<T(void)> evaluation)
      : evaluation{std::move(evaluation)} {}

  bool trigger() const {
    if (!member) {
      member = evaluation();
      return true;
    }
    return false;
  }

  Lazy<T> &operator=(T &m) {
    member = m;
    return *this;
  }

  Lazy<T> &operator=(T &&m) {
    member = std::move(m);
    return *this;
  }

  Lazy<T> &operator=(Lazy<T> &&m) noexcept = default;

  Lazy<T> &operator=(std::function<T(void)> evaluation_) {
    evaluation = std::move(evaluation_);
    return *this;
  }

  T *operator->() { return &(**this); }

  const T *operator->() const {
    const auto &m = **this;
    return &m;
  }
  const T &operator*() const {
    trigger();
    return *member;
  }

  T &operator*() {
    trigger();
    return *member;
  }

private:
  std::function<T(void)> evaluation;
  mutable std::optional<T> member;
};

TEST_CASE("Lazy Container") {
  int x = 3;
  Lazy<int> l([]() { return 1; });
  CHECK(*l == 1);
  l = 2;
  CHECK(*l == 2);
  Lazy<int> l2([&]() { return x; });
  CHECK(*l2 == 3);
}

} // namespace cetsp::utils
#endif // CETSP_LAZY_MEMBER_H
