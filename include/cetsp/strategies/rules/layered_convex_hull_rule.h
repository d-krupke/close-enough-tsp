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
  std::vector<std::optional<unsigned int>> global_to_hull_map;
  std::vector<unsigned int> hull_to_global_map;
};

class LayeredConvexHullRule : public SequenceRule {
public:
  virtual void setup(const Instance *instance, std::shared_ptr<Node> &root,
                     SolutionPool *solution_pool);

  virtual bool is_ok(const std::vector<int> &seq);

private:
  bool is_ok(const std::vector<int> &seq, unsigned int layer) const;
  unsigned int get_hull_size(unsigned int layer) const;

  const Instance *instance = nullptr;
  std::vector<ConvexHullLayer> layers;
};

} // namespace cetsp
#endif // CETSP_LAYERED_CONVEX_HULL_RULE_H
