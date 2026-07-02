#include "osr/cch_preprocessing/node_ordering.h"

#include "fmt/core.h"
#include "fmt/std.h"

#include "utl/helpers/algorithm.h"

namespace osr::cch_preprocessing {

    template <HasNodeImportance T>
    node_ordering node_ordering::import(T const& t) {
        node_ordering ordering;
        ordering.old_to_new_.resize(t.node_important_.size());
        ordering.new_to_old_.resize(t.node_important_.size());
        vec<std::pair<std::uint32_t, node_idx_t>> nodes;
        for (auto const& [node, importance] : t.node_important_) {
            nodes.emplace_back(importance, node);
        }
        utl::sort(nodes);
        for (std::size_t i = 0U; i < nodes.size(); ++i) {
            auto const node = nodes[i].second;
            ordering.old_to_new_[node] = static_cast<node_idx_t>(i);
            ordering.new_to_old_[static_cast<node_idx_t>(i)] = node;
        }
        return ordering;
    }

    node_ordering node_ordering::randomize(std::size_t num_nodes, std::uint32_t seed) {
        node_ordering ordering;
        ordering.old_to_new_.resize(num_nodes);
        ordering.new_to_old_.resize(num_nodes);
        vec<node_idx_t> nodes(num_nodes);
        std::iota(begin(nodes), end(nodes), static_cast<node_idx_t>(0U));
        std::mt19937 rng(seed);
        std::shuffle(begin(nodes), end(nodes), rng);
        for (std::size_t i = 0U; i < nodes.size(); ++i) {
            auto const node = nodes[i];
            ordering.old_to_new_[node] = static_cast<node_idx_t>(i);
            ordering.new_to_old_[static_cast<node_idx_t>(i)] = node;
        }
        return ordering;
    }
}