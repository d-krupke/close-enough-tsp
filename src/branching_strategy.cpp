//
// Created by Dominik Krupke on 21.12.22.
//
#include "cetsp/details/branching_strategy.h"
namespace cetsp {

/**
 * Find the circle that is most distanced to the current (relaxed) solution.
 * This is a good circle to branch upon. If no circle is uncovered, it returns
 * nothing.
 * @param solution The relaxed solution.
 * @param instance The instance with the circles.
 * @return The index of the most distanced circle in the solution or nothing
 *          if all circles are included.
 */
std::optional<int>
get_index_of_most_distanced_circle(const Trajectory &solution,
                                   const Instance &instance) {
  const auto n = instance.size();
  std::vector<double> distances(n);
  for (unsigned i = 0; i < n; ++i) {
    distances[i] = solution.distance(instance[i]);
  }
  auto max_dist = std::max_element(distances.begin(), distances.end());
  if (*max_dist <= 0) {
    return {};
  }
  const int c = std::distance(distances.begin(), max_dist);
  return {c};
}

bool FarthestCircle::branch(Node &node) {
  const auto c = get_index_of_most_distanced_circle(node.get_relaxed_solution(),
                                                    *instance);
  if (!c) {
    return false;
  }
  std::vector<Node> children;
  std::vector<int> seqeuence;
  if (simplify) {
    seqeuence = node.get_spanning_sequence();
  } else {
    seqeuence = node.get_fixed_sequence();
  }
  seqeuence.push_back(*c);
  if (instance->is_path()) {
    // for path, this position may not be symmetric.
    children.emplace_back(seqeuence, instance, &node);
  }
  for (int i = seqeuence.size() - 1; i > 0; --i) {
    seqeuence[i] = seqeuence[i - 1];
    seqeuence[i - 1] = *c;
    if (is_sequence_ok(seqeuence)) {
      children.emplace_back(seqeuence, instance, &node);
    }
  }
  node.branch(std::move(children));
  return true;
}
} // namespace cetsp
