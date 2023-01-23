//
// Created by Barak Ugav on 20.01.23.
//

#include "cetsp/strategies/rules/layered_convex_hull_rule.h"
#include "cetsp/strategies/branching_strategy.h"

namespace cetsp {

static std::vector<ConvexHullLayer> calc_ch_layers(const Instance &instance) {
  std::vector<ConvexHullLayer> layers;
  // handled[circle_idx] = the circle is included in the already created layers
  std::vector<bool> handled(instance.size(), false);

  for (unsigned int unhandled_num;;) {
    unhandled_num = std::count(handled.begin(), handled.end(), false);
    if (unhandled_num == 0)
      break;

    /* ConvexHullOrder is operating on a continues range of indices, so we need
     * to map from the global indices to a smaller range [0, # of unhandled).
     * unhandled[unhandled_idx] = global_idx */
    std::vector<unsigned int> unhandled;
    unhandled.reserve(unhandled_num);
    for (unsigned int global_idx = 0; global_idx < instance.size();
         global_idx++) {
      if (!handled[global_idx]) {
        unhandled.push_back(global_idx);
      }
    }

    /* Calculate the convex hull of all unhandled circles */
    std::vector<Point> unhandled_points;
    unhandled_points.reserve(unhandled.size());
    for (unsigned int i : unhandled) {
      unhandled_points.push_back(instance[i].center);
    }
    details::ConvexHullOrder vho(unhandled_points);
    std::vector<bool> is_in_layer_hull(unhandled.size(), false);
    std::vector<std::pair<unsigned int, double>> layer_hull;
    for (unsigned unhandled_idx = 0; unhandled_idx < unhandled.size();
         unhandled_idx++) {
      const auto weight = vho(instance[unhandled[unhandled_idx]]);
      if (weight) {
        is_in_layer_hull[unhandled_idx] = true;
        layer_hull.push_back({unhandled_idx, *weight});
      } else {
        is_in_layer_hull[unhandled_idx] = false;
      }
    }
    std::sort(layer_hull.begin(), layer_hull.end(),
              [](const auto &a, const auto &b) { return a.second < b.second; });

    ConvexHullLayer layer;
    layer.global_to_hull_map =
        std::vector<std::optional<unsigned int>>(instance.size());
    layer.hull_to_global_map = std::vector<unsigned int>();
    layer.hull_to_global_map.reserve(layer_hull.size());
    for (unsigned int hull_idx = 0; hull_idx < layer_hull.size(); hull_idx++) {
      unsigned int unhandled_idx = layer_hull[hull_idx].first;
      unsigned int global_idx = unhandled[unhandled_idx];
      layer.global_to_hull_map[global_idx] = hull_idx;
      layer.hull_to_global_map.push_back(global_idx);
      handled[global_idx] = true;
    }
    layers.push_back(layer);
  }
  return layers;
}

void LayeredConvexHullRule::setup(const Instance *instance_,
                                  std::shared_ptr<Node> &root,
                                  SolutionPool *solution_pool) {
  instance = instance_;
  layers = calc_ch_layers(*instance);

  if (!is_ok(root->get_fixed_sequence(), 0)) {
    throw std::invalid_argument("Root does not obey the layered convex hull.");
  }
}

class HullVisitor {
public:
  HullVisitor(const ConvexHullLayer &layer, const std::vector<int> &seq) {
    unsigned int hull_size = layer.hull_to_global_map.size();
    /* Compute the order the CH vertices are visited by the sequence */
    std::vector<std::optional<unsigned int>> hull_visits_full(hull_size);
    hull_vertex_to_seq_idx_map =
        std::vector<std::optional<unsigned int>>(hull_size);
    unsigned int visit_num = 0;
    for (unsigned int seq_idx = 0; seq_idx < seq.size(); seq_idx++) {
      auto hull_idx = layer.global_to_hull_map[seq[seq_idx]];
      if (hull_idx) {
        hull_visits_full[*hull_idx] = visit_num++;
        hull_vertex_to_seq_idx_map[*hull_idx] = seq_idx;
      }
    }
    if (visit_num <= 2)
      return;
    hull_visits.reserve(visit_num);
    visit_to_hull_idx_map = std::vector<unsigned int>(hull_visits.size());
    for (unsigned int hull_idx = 0; hull_idx < hull_size; hull_idx++) {
      auto visit_idx = hull_visits_full[hull_idx];
      if (visit_idx) {
        hull_visits.push_back(*visit_idx);
        visit_to_hull_idx_map[*visit_idx] = hull_idx;
      }
    }

    /* Rotate hull_visits such that 0,1 are the first two elements */
    std::rotate(hull_visits.begin(),
                std::find(hull_visits.begin(), hull_visits.end(), 0),
                hull_visits.end());
    is_hull_visits_reversed = hull_visits[1] != 1;
    if (is_hull_visits_reversed) {
      std::reverse(hull_visits.begin(), hull_visits.end());
      std::rotate(hull_visits.begin(), hull_visits.end() - 1,
                  hull_visits.end());
    }
    assert(hull_visits[0] == 0);
  }

  unsigned int get_hull_visit(unsigned int idx) const {
    assert(idx < hull_visits.size());
    return hull_visits[idx];
  }

  unsigned int map_visit_to_hull_idx(unsigned int visit_idx) const {
    assert(visit_idx < visit_to_hull_idx_map.size());
    return visit_to_hull_idx_map[visit_idx];
  }

  std::optional<unsigned int>
  map_hull_vertex_to_seq_idx(unsigned int hull_vertex_idx) const {
    assert(hull_vertex_idx < hull_vertex_to_seq_idx_map.size());
    return hull_vertex_to_seq_idx_map[hull_vertex_idx];
  }

  unsigned int get_visits_num() const { return hull_visits.size(); }

  /* Assuming the input sequence is a tour, check if its ok */
  bool is_tour_ok() const {
    unsigned int visits_num = get_visits_num();
    if (visits_num <= 3)
      return true;

    for (unsigned int i = 0; i < visits_num - 1; i++) {
      if (get_hull_visit(i) > get_hull_visit(i + 1)) {
        return false;
      }
    }
    return true;
  }

  /* Assuming the input sequence is a path, check if its ok */
  bool is_path_ok() const {
    unsigned int visits_num = get_visits_num();
    if (visits_num <= 4)
      return true;

    /* Make sure hull_visits is composed of a monotone increasing sequence
     * followed by a monotone decreasing sequence */
    unsigned int i = 0;
    for (; i < visits_num - 1; i++)
      if (get_hull_visit(i) > get_hull_visit(i + 1))
        break;
    for (; i < visits_num - 1; i++)
      if (get_hull_visit(i) < get_hull_visit(i + 1))
        break;
    return i == visits_num - 1;
  }

  bool is_hull_visits_reversed;

private:
  /* Following the given sequence, the visitor will assign a visit index
   * 0,1,2,... to each CH vertex it encounters. Than, the visitor will iterate
   * over the CH vertices in counter-clockwise order starting from the CH vertex
   * assigned visit index 0 and will fill the hull_visits array. */
  std::vector<unsigned int> hull_visits;
  /* Map from visit_idx to hull_idx */
  std::vector<unsigned int> visit_to_hull_idx_map;
  /* Map from hull_idx to seq_idx (aka the index of the vertex in the input
   * sequence) */
  std::vector<std::optional<unsigned int>> hull_vertex_to_seq_idx_map;
};

bool LayeredConvexHullRule::is_ok(const std::vector<int> &seq,
                                  const Node &parent) {
  return is_ok(seq, /* layer = */ 0);
}

bool LayeredConvexHullRule::is_ok(const std::vector<int> &seq,
                                  unsigned int layer_idx) const {
  if (layer_idx >= layers.size()) {
    return true;
  }
  unsigned int hull_size = layers[layer_idx].hull_to_global_map.size();
  HullVisitor visitor(layers[layer_idx], seq);
  unsigned int visits_num = visitor.get_visits_num();
  if (visits_num <= 2)
    return true;

  bool is_path = layer_idx > 0 || instance->is_path();
  if (is_path) {
    /* Make sure the sequence follow the hull constraint in the current layer */
    if (!visitor.is_path_ok())
      return false;

    /* Check lower layers */
    // TODO

  } else { /* tour */
    /* Make sure the sequence follow the hull constraint in the current layer */
    if (!visitor.is_tour_ok())
      return false;

    /* Check lower layers */
    for (unsigned int i = 0; i < visits_num; i++) {
      auto v1 = visitor.get_hull_visit(i),
           v2 = visitor.get_hull_visit((i + 1) % visits_num);
      unsigned int a = visitor.map_visit_to_hull_idx(v1),
                   b = visitor.map_visit_to_hull_idx(v2);
      bool are_consecutive = ((b - a) % hull_size) == 1;
      /* If the sequence visit two consecutive vertices of the convex hull, we
       * know it is not allowed to visit any other hull vertex between them.
       * We check that the subpath between them visit the lower layer convex
       * hull in a valid sequence. */
      if (are_consecutive) {
        auto sub_begin_opt = visitor.map_hull_vertex_to_seq_idx(a);
        auto sub_end_opt = visitor.map_hull_vertex_to_seq_idx(b);
        assert(sub_begin_opt && sub_end_opt);
        unsigned int sub_begin = *sub_begin_opt, sub_end = *sub_end_opt;
        if (visitor.is_hull_visits_reversed) {
          std::swap(sub_begin, sub_end);
        }
        std::vector<int> sub_seq;
        unsigned int i = sub_begin;
        for (; i > sub_end; i = (i + 1) % seq.size()) {
          sub_seq.push_back(seq[i]);
        }
        for (; i <= sub_end; ++i) {
          sub_seq.push_back(seq[i]);
        }
        if (!is_ok(sub_seq, layer_idx + 1)) {
          return false;
        }
      }
    }
  }
  return true;
}

} // namespace cetsp
