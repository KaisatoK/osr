#pragma once

#include <stack>
#include <queue>
#include <array>

#include "osr/cch_preprocessing/customized_cost.h"

namespace osr {
    namespace prep = cch_preprocessing;

    template <Profile P>
    struct cch_query {
        using profile_t = P;
        using key = typename P::key;
        using label = typename P::label;
        using node = typename P::node;
        using entry = typename P::entry;
        using hash = typename P::hash;

        struct node_entry {
            static constexpr auto const kMaxPredSize = 4U;

            node_entry(node n, cost_t cost) : curr_(n), cost_(cost) {
                std::fill(pred_.begin(), pred_.end(), node::invalid());
            }

            static constexpr node_entry invalid() {
                return node_entry{node::invalid(), kInfeasible, {node::invalid(), node::invalid(), node::invalid(), node::invalid()}};
            }

            void update(cost_t const new_cost, node const pred) {
                if (new_cost < cost_) {
                    cost_ = new_cost;
                    pred_[0] = pred;
                    std::fill(pred_.begin() + 1, pred_.end(), node::invalid());
                } else if (new_cost == cost_) {
                    for (auto& p : pred_) {
                        if (p == node::invalid()) {
                            p = pred;
                            break;
                        }
                    }
                }
            }

            node curr_;
            cost_t cost_;
            std::array<node, kMaxPredSize> pred_;
        };

        cch_query(prep::customized_cost_stored<P> const& cost_function)
            : cost_function_{cost_function},
              node_costs_(cost_function_.virtual_nodes_.size()),
              is_marked_(cost_function_.virtual_nodes_.size()),
              starts(),
              dests() {
            for (std::size_t i = 0; i < cost_function_.virtual_nodes_.size(); ++i) {
                node_costs_[i].resize(cost_function_.virtual_nodes_[i].size(), node_entry::invalid());
                is_marked_[i].resize(cost_function_.virtual_nodes_[i].size(), false);
            }
        }

        cch_query(std::filesystem::path path) : cch_query{prep::customized_cost_stored<P>::read(path)} {}

        void reset() {
            starts.clear();
            dests.clear();
            cost_function_.for_each_vir_node<false>([&](node const n, prep::ext_node_idx_t const idx) {
                node_costs_[idx.first][idx.second] = node_entry::invalid();
                is_marked_[idx.first][idx.second] = false;
            });
        }

        void add_start(node const n) {
            auto const start_idx = cost_function_.get_virtual_node_idx(n);
            if (start_idx) {
                starts.emplace_back(n);
                node_costs_[start_idx->first][start_idx->second] = node_entry{n, 0};
            }
        }

        void add_dest(node const n) {
            auto const dest_idx = cost_function_.get_virtual_node_idx(n);
            if (dest_idx) {
                dests.emplace_back(n);
            }
        }

        template <typename Container>
        void add(prep::ext_node_idx_t u, Container& container) {
            if (is_marked_[u.first][u.second]) {
                return;
            }
            std::stack<prep::ext_node_idx_t> st{u};
            while (true) {
                is_marked_[u.first][u.second] = true;
                auto par = cost_function_.get_parent(u);
                if (par && *par != u && !is_marked_[par->first][par->second]) {
                    st.push(*par);
                } else {
                    break;
                }
                u = *par;
            }
            while (!st.empty()) {
                auto const u = st.top();
                st.pop();
                container.emplace(u);
            }
        }

        bool run() {
            // first phase
            std::stack<prep::ext_node_idx_t> prep_order1{};
            for (auto const& start : starts) {
                auto start_idx = cost_function_.get_virtual_node_idx(start);
                add(*start_idx, prep_order1);
            }
            while (!prep_order1.empty()) {
                auto const u = prep_order1.top();
                prep_order1.pop();
                for (auto const& uv : cost_function_.get_upward_edges(u)) {
                    auto const v = cost_function_.extended_edges_[uv].to_;
                    auto const new_cost = node_costs_[u.first][u.second] + cost_function_.extended_edges_[uv].cost_;
                    node_costs_[v.first][v.second].update(new_cost, cost_function_.get_virtual_node(u));
                }
                is_marked_[u.first][u.second] = false;
            }

            // second phase
            std::queue<prep::ext_node_idx_t> prep_order2{};
            for (auto const& dest : dests) {
                auto dest_idx = cost_function_.get_virtual_node_idx(dest);
                add(*dest_idx, prep_order2);
            }
            while (!prep_order2.empty()) {
                auto const u = prep_order2.front();
                prep_order2.pop();
                for (auto const& uv : cost_function_.get_downward_edges(u)) {
                    auto const v = cost_function_.extended_edges_[uv].to_;
                    auto const new_cost = node_costs_[u.first][u.second] + cost_function_.extended_edges_[uv].cost_;
                    node_costs_[v.first][v.second].update(new_cost, cost_function_.get_virtual_node(u));
                }
                is_marked_[u.first][u.second] = false;
            }

            return true;
        }

        prep::customized_cost_stored<P> const& cost_function_;
        std::vector<std::vector<node_entry>> node_costs_;
        std::vector<std::vector<bool>> is_marked_;
        std::vector<node> starts, dests;
    };
} // namespace osr