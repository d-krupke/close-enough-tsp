#include <gtest/gtest.h>
#include "cetsp_solver/search_manager.hpp"

TEST(SearchManagerTest, BasicTest) {
    
    using namespace cetsp_solver;
    Instance instance{{{0,0}, 0}, {{1,0}, 0}, {{1,1}, 0}, {{0,1}, 0}};
    SearchManager search_manager{};
    ASSERT_EQ(search_manager.size(), 0);
    ASSERT_EQ(search_manager.get_lower_bound(), std::numeric_limits<double>::infinity());
    
    NodeFactory nf;
    auto root_node = nf.create_root_node({{0, 1, 2, 3}});
    root_node->lb = 4.0;
    search_manager.enqueue_node(std::move(root_node));
    ASSERT_EQ(search_manager.size(), 1);
    ASSERT_EQ(search_manager.get_lower_bound(), 4.0);

    auto rnode = search_manager.get_next_node(min_lb);
    ASSERT_EQ(search_manager.size(), 1);
    ASSERT_NE(rnode, nullptr);
    auto node = search_manager.get_next_node(min_lb);
    ASSERT_EQ(node, nullptr);
    
    search_manager.enqueue_node(nf.create_child_node(*rnode, {{1,2,3}}));
    search_manager.enqueue_node(nf.create_child_node(*rnode, {{2,1,3}}));
    search_manager.close_node(rnode);
    ASSERT_EQ(search_manager.size(), 2);
    node = search_manager.get_next_node(min_lb);
    ASSERT_NE(node, nullptr);
}