//
// Created by Barak Ugav on 20.01.23.
//

#ifndef CETSP_LAYERED_CONVEX_HULL_RULE_H
#define CETSP_LAYERED_CONVEX_HULL_RULE_H

#include "cetsp/common.h"
#include "cetsp/details/solution_pool.h"
#include "cetsp/node.h"
#include "cetsp/strategies/rule.h"

namespace cetsp {

class ConvexHullLayer {
public:
  /*
   * Each convex hull vertex is assigned an number in range [0, CH size)
   * by their counter-clockwise order starting from an arbitrary one.
   * The first top most level CH is the familiar CH. The next layered CH is
   * achieved by removing all points contained in the top most level CH, and
   * calculating a CH on the remaining sub set. This procedure is repeated as
   * long as there are some points not included in any layer. This class
   * represent a single such layer.
   */

  /* For each index p \in [0, n) of an input point, global_to_hull_map[p] is
   * present iff p is in the layer CH, and if it present the value is the CH
   * number of p. */
  std::vector<std::optional<unsigned int>> global_to_hull_map;
  /* For each CH index q, hull_to_global_map[q] is the global index of
   * q in the input. The size of hull_to_global_map is the number of vertices in
   * the layer CH. */
  std::vector<unsigned int> hull_to_global_map;
};

class LayeredConvexHullRule : public SequenceRule {
public:
  void setup(const Instance *instance, std::shared_ptr<Node> &root,
             SolutionPool *solution_pool) override;

  bool is_ok(const std::vector<int> &seq, const Node &parent) override;

private:
  bool is_ok(const std::vector<int> &seq, unsigned int layer) const;

  const Instance *instance = nullptr;
  std::vector<ConvexHullLayer> layers;
};

} // namespace cetsp
#endif // CETSP_LAYERED_CONVEX_HULL_RULE_H
