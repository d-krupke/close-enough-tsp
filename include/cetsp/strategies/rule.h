//
// Created by Dominik Krupke on 19.01.23.
//

#ifndef CETSP_RULE_H
#define CETSP_RULE_H
#include "cetsp/common.h"
#include "cetsp/details/solution_pool.h"
#include "cetsp/node.h"
namespace cetsp {
class SequenceRule {
public:
  virtual void setup(const Instance *instance, std::shared_ptr<Node> &root,
                     SolutionPool *solution_pool) = 0;
  virtual bool is_ok(const std::vector<int> &seq) = 0;
  virtual ~SequenceRule() = default;
};

} // namespace cetsp
#endif // CETSP_RULE_H
