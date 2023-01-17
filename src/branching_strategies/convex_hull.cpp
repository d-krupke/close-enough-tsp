//
// Created by Dominik Krupke on 17.01.23.
//
#include "cetsp/details/branching_strategy.h"

namespace cetsp {
class ConvexHullRule : public SequenceRule {
public:
  virtual void setup(const Instance *instance, std::shared_ptr<Node> &root,
                     SolutionPool *solution_pool) {
    order_values.resize(instance->size());
    is_ordered.resize(instance->size(), false);
    compute_weights(instance, root);

    if (!sequence_is_ch_ordered(root->get_fixed_sequence())) {
      for (auto i : root->get_fixed_sequence()) {
        std::cout << i << ":  " << order_values[i] << " " << is_ordered[i]
                  << std::endl;
      }
      throw std::invalid_argument("Root does not obey the convex  hull.");
    }
  }
  virtual bool is_ok(const std::vector<int> &seq) {
    auto is_ok = sequence_is_ch_ordered(seq);
    return is_ok;
  }

protected:
  bool sequence_is_ch_ordered(const std::vector<int> &seqeuence) {
    std::vector<double> order_values_;
    for (const auto &i : seqeuence) {
      if (is_ordered[i]) {
        order_values_.push_back(order_values[i]);
      }
    }
    // The minimal element may be in the middle. So we rotate the minimal
    // element to the front.
    auto min_ = std::min_element(order_values_.begin(), order_values_.end());
    std::rotate(order_values_.begin(), min_, order_values_.end());
    return std::is_sorted(order_values_.begin(), order_values_.end());
  }

  std::vector<Point> get_circle_centers(const Instance &instance) const {
    std::vector<Point> points;
    points.reserve(instance.size());
    for (const auto &c : instance) {
      points.push_back(c.center);
    }
    return points;
  }

  void compute_weights(const Instance *instance, std::shared_ptr<Node> &root) {
    // Compute the weights used to check if the partial solution
    // obeys the convex hull.
    auto points = get_circle_centers(*instance);
    details::ConvexHullOrder vho(points);
    for (unsigned i = 0; i < instance->size(); ++i) {
      const auto weight = vho((*instance)[i]);
      if (weight) {
        is_ordered[i] = true;
        order_values[i] = *weight;
      } else {
        is_ordered[i] = false;
      }
    }
  }

private:
  std::vector<double> order_values;
  std::vector<bool> is_ordered;
};

ChFarthestCircle::ChFarthestCircle(bool simplify) : FarthestCircle(simplify) {
  std::cout << "Using ChFarthestCircle-Branching" << std::endl;
  add_rule(std::make_unique<ConvexHullRule>());
}
} // namespace cetsp
