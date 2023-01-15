//
// Created by Dominik Krupke on 21.12.22.
//
#include "cetsp/details/branching_strategy.h"
#include <boost/thread/thread.hpp>
// #include <execution>
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
get_index_of_most_distanced_circle(const PartialSequenceSolution &solution,
                                   const Instance &instance) {
  const auto n = instance.size();
  std::vector<double> distances(n);
  for (unsigned i = 0; i < n; ++i) {
    if (solution.covers(i)) {
      distances[i] = 0;
    } else {
      distances[i] = solution.distance(i);
    }
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
  std::vector<std::shared_ptr<Node>> children;
  std::vector<int> seqeuence;
  if (simplify) {
    seqeuence = node.get_spanning_sequence();
  } else {
    seqeuence = node.get_fixed_sequence();
  }
  seqeuence.push_back(*c);
  if (instance->is_path()) {
    // for path, this position may not be symmetric.
    if (is_sequence_ok(seqeuence)) {
      children.push_back(std::make_shared<Node>(seqeuence, instance, &node));
    }
  }
  for (int i = seqeuence.size() - 1; i > 0; --i) {
    seqeuence[i] = seqeuence[i - 1];
    seqeuence[i - 1] = *c;
    if (is_sequence_ok(seqeuence)) {
      children.push_back(std::make_shared<Node>(seqeuence, instance, &node));
      /**
      if (simplify) {
        children.back()->simplify();
      }**/
    }
  }
  boost::thread_group tg;
  for(auto& child: children) {
    tg.create_thread([&](){child->simplify();});
  }
  tg.join_all();
  // for_each(std::execution::par, children.begin(), children.end(), [](auto&
  // child){child.trigger_lazy_evaluation();});
  node.branch(children);
  return true;
}
} // namespace cetsp
