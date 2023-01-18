//
// Created by Dominik Krupke on 17.01.23.
//
#include "cetsp/details/branching_strategy.h"

namespace cetsp {
class ConvexHullRule : public SequenceRule {
public:
  virtual void setup(const Instance *instance_, std::shared_ptr<Node> &root,
                     SolutionPool *solution_pool) {
    instance = instance_;
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

  static bool
  is_path_sequence_possible(const std::vector<int> &sequence, unsigned int n,
                            const std::vector<bool> &is_in_ch,
                            const std::vector<double> &order_values) {
    /* Create convex hull */
    /* hull[] = {disc_index, order_value[disc_index]} */
    std::vector<std::pair<unsigned int, double>> hull;
    for (const auto &i : sequence) {
      if (is_in_ch[i]) {
        hull.push_back({i, order_values[i]});
      }
    }
    /* Sort by order on the CH */
    std::sort(std::begin(hull), std::end(hull),
              [](const auto &a, const auto &b) { return a.second < b.second; });
    if (hull.size() <= 2)
      return true;

    /* ch_numbers[disc_index] = index on CH */
    std::vector<unsigned int> ch_numbers(n);
    for (unsigned int i = 0; i < hull.size(); i++) {
      ch_numbers[hull[i].first] = i;
    }

    /* Compute the order the CH vertices are visited by the sequence */
    std::vector<unsigned int> ch_order(hull.size());
    unsigned int steps = 0;
    for (const auto &i : sequence) {
      if (is_in_ch[i]) {
        ch_order[ch_numbers[i]] = steps++;
      }
    }

    /* Rotate the order such that 0,1 are the first two elements in ch_order */
    auto first_on_ch = std::find(std::begin(ch_order), std::end(ch_order), 0);
    std::rotate(std::begin(ch_order), first_on_ch, std::end(ch_order));
    unsigned int first_on_ch_idx =
        std::distance(std::begin(ch_order), first_on_ch);
    bool is_rotated = ch_order[(first_on_ch_idx + 1) % ch_order.size()] != 1;
    if (is_rotated) {
      std::reverse(std::begin(ch_order), std::end(ch_order));
      std::rotate(std::begin(ch_order), std::end(ch_order) - 1,
                  std::end(ch_order));
    }

    /* Check if ch_order is composed of a monotone increasing sequence followed
     * by a monotone decreasing sequence */
    unsigned int i = 0;
    for (; i < ch_order.size()-1; i++) {
      if (ch_order[i] > ch_order[i + 1]) {
        break;
      }
    }
    for (; i < ch_order.size()-1; i++) {
      if (ch_order[i] < ch_order[i + 1]) {
        break;
      }
    }
    return i == ch_order.size()-1;
  }

  bool sequence_is_ch_ordered(const std::vector<int> &sequence) {
    if (instance->is_path()) {
      return is_path_sequence_possible(sequence, instance->size(), is_ordered,
                                       order_values);

    } else { /* tour */
      std::vector<double> order_values_;
      for (const auto &i : sequence) {
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
  const Instance *instance = nullptr;
  std::vector<double> order_values;
  std::vector<bool> is_ordered;
};

ChFarthestCircle::ChFarthestCircle(bool simplify) : FarthestCircle(simplify) {
  std::cout << "Using ChFarthestCircle-Branching" << std::endl;
  add_rule(std::make_unique<ConvexHullRule>());
}
} // namespace cetsp
