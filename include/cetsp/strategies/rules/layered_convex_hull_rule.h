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
   * The first top most level CH is the 'regular' CH. The next layered CH is
   * achieved by removing all points that lie on the top most level CH, and
   * calculating the CH of the remaining sub set. This procedure is repeated as
   * long as there are some points not included in any layer. This class
   * represent a single such layer.
   */

  /* For each index p \in [0, n) of an input point, global_to_hull_map[p] is
   * present iff p is in the layer CH, and if it present the value is the CH
   * index of p. */
  std::vector<std::optional<unsigned int>> global_to_hull_map;
  /* For each CH index q, hull_to_global_map[q] is the global index of
   * q in the input. The size of hull_to_global_map is the number of vertices in
   * the layer CH. */
  std::vector<unsigned int> hull_to_global_map;

  bool is_in_hull(unsigned int i) const {
    auto opt = global_to_hull_map[i];
    return opt ? true : false;
  }

  static std::vector<ConvexHullLayer> calc_ch_layers(const Instance &instance);
};

class LayeredConvexHullRule : public SequenceRule {
public:
  void setup(const Instance *instance, std::shared_ptr<Node> &root,
             SolutionPool *solution_pool) override;

  bool is_ok(const std::vector<int> &seq, const Node &parent) override;

  const ConvexHullLayer &get_layer(unsigned int layer_idx) const {
    assert(layer_idx < layers.size());
    return layers[layer_idx];
  }

  unsigned int get_number_of_layers() const { return layers.size(); }

private:
  bool is_ok(const std::vector<int> &seq, unsigned int layer) const;

  const Instance *instance = nullptr;
  std::vector<ConvexHullLayer> layers;
};

TEST_CASE("LayeredConvexHull_layers_calc") {
  std::vector<Circle> instance_ = {
      {{-12, -12}, 1}, {{-12, 12}, 1}, {{12, -12}, 1}, {{12, 12}, 1},
      {{-9, -9}, 1},   {{-9, 9}, 1},   {{9, -9}, 1},   {{9, 9}, 1},
      {{-6, -6}, 1},   {{-6, 6}, 1},   {{6, -6}, 1},   {{6, 6}, 1},
      {{-3, -3}, 1},   {{-3, 3}, 1},   {{3, -3}, 1},   {{3, 3}, 1},
      {{0, 0}, 1},
  };
  Instance instance(instance_);
  auto layers = ConvexHullLayer::calc_ch_layers(instance);

  auto is_layer_eq = [&layers](unsigned int layer_idx,
                               const std::vector<unsigned int> &elms) {
    const auto &layer = layers[layer_idx];
    std::unordered_set<unsigned int> expected(elms.begin(), elms.end());
    std::unordered_set<unsigned int> actual(layer.hull_to_global_map.begin(),
                                            layer.hull_to_global_map.end());

    /* Check there are no duplications in the layer */
    CHECK(actual.size() == layer.hull_to_global_map.size());
    return expected == actual;
  };

  CHECK(layers.size() == 5);
  CHECK(is_layer_eq(0, {0, 1, 2, 3}));
  CHECK(is_layer_eq(1, {4, 5, 6, 7}));
  CHECK(is_layer_eq(2, {8, 9, 10, 11}));
  CHECK(is_layer_eq(3, {12, 13, 14, 15}));
  CHECK(is_layer_eq(4, {16}));
}

} // namespace cetsp
#endif // CETSP_LAYERED_CONVEX_HULL_RULE_H
