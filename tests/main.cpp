#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "cetsp/bnb.h"
#include "cetsp/common.h"
#include "cetsp/details/convex_hull_order.h"
#include "cetsp/heuristics.h"
#include "cetsp/node.h"
#include "cetsp/relaxed_solution.h"
#include "cetsp/soc.h"
#include "cetsp/strategies/branching_strategy.h"
#include "cetsp/strategies/root_node_strategy.h"
#include "cetsp/strategies/search_strategy.h"
#include "cetsp/utils/geometry.h"
#include "cetsp/strategies/rules/convex_hull_rule.h"
#include "doctest/doctest.h"
