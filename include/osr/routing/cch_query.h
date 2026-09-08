#pragma once

#include <stack>
#include <queue>
#include <array>

#include "osr/cch_preprocessing/extended_type.h"
#include "osr/cch_preprocessing/preprocessed_data.h"

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

        // cch_query(std::filesystem::path const& path) : cost_function_{*prep::customized_cost_stored<P>::read(path)} {}

        void reset() {
            starts_.clear();
            dests_.clear();
            node_costs_.clear();
            is_marked_.clear();
        }

        prep::node_entry const get_node_entry(node const n) const {
            auto const idx = cost_function_.get_flatten_node_idx(cost_function_.get_virtual_node_idx(n, ordering_));
            if (node_costs_.count(idx)) {
                return node_costs_.at(idx);
            }
            return prep::node_entry::invalid();
        }

        prep::node_entry const get_node_entry(prep::ext_node const n) const {
            auto const idx = cost_function_.get_flatten_node_idx(n);
            if (node_costs_.count(idx)) {
                return node_costs_.at(idx);
            }
            return prep::node_entry::invalid();
        }

        void add_start(node const n, cost_t const start_cost) {
            auto const start_idx = cost_function_.get_virtual_node_idx(n, ordering_);
            if (start_idx != prep::ext_node::invalid()) {
                starts_.emplace_back(start_idx);
                auto content = prep::node_entry::invalid();
                content.curr_ = start_idx;
                content.cost_ = start_cost;
                node_costs_[cost_function_.get_flatten_node_idx(start_idx)] = content;
            }
        }

        void add_dest(node const n) {
            auto const dest_idx = cost_function_.get_virtual_node_idx(n, ordering_);
            if (dest_idx != prep::ext_node::invalid()) {
                dests_.emplace_back(dest_idx);
            }
        }

        cost_t get_best_cost() const {
            cost_t best_cost = kInfeasible;
            for (auto const& dest : dests_) {
                auto const entry = get_node_entry(dest);
                best_cost = std::min(best_cost, entry.cost_);
            }
            return best_cost;
        }

        template <typename Container>
        void add(prep::ext_node u, Container& container) {
            if (is_marked_.count(cost_function_.get_flatten_node_idx(u))) {
                return;
            }
            std::stack<prep::ext_node> st;
            st.push(u);
            while (true) {
                is_marked_[cost_function_.get_flatten_node_idx(u)] = true;
                auto par = cost_function_.get_parent(u, tree_);
                if (par != prep::ext_node::invalid() && par != u && !is_marked_.count(cost_function_.get_flatten_node_idx(par))) {
                    st.push(par);
                } else {
                    break;
                }
                u = par;
            }
            while (!st.empty()) {
                auto const u = st.top();
                st.pop();
                container.emplace(u);
            }
        }

        bool run() {
            // first phase
            std::stack<prep::ext_node> prep_order1{};
            for (auto const& start_idx : starts_) {
                if (start_idx == prep::ext_node::invalid()) {
                    continue;
                }
                add(start_idx, prep_order1);
            }
            while (!prep_order1.empty()) {
                auto const u = prep_order1.top();
                prep_order1.pop();
                if (node_costs_.count(cost_function_.get_flatten_node_idx(u)) == 0) {
                    continue;
                }
                auto const curr_cost = node_costs_[cost_function_.get_flatten_node_idx(u)].cost_;
                for (auto const& uv : cost_function_.get_upward_edges(u)) {
                    utl::verify(cost_function_.extended_edges_[uv].from_ == u, "upward edge from mismatch, expected {} but got {}", prep::to_string(u), prep::to_string(cost_function_.extended_edges_[uv].from_));
                    if (curr_cost >= kInfeasible - cost_function_.extended_edges_[uv].cost_) {
                        continue;
                    }
                    auto const v = cost_function_.extended_edges_[uv].to_;
                    auto const v_idx = cost_function_.get_flatten_node_idx(v);
                    auto const new_cost = curr_cost + cost_function_.extended_edges_[uv].cost_;

                    if (!node_costs_.count(v_idx)) {
                        node_costs_[v_idx] = prep::node_entry::invalid();
                    }
                    node_costs_[v_idx].update(new_cost, uv);
                }
            }
            is_marked_.clear();

            // second phase
            std::queue<prep::ext_node> prep_order2{};
            for (auto const& dest_idx : dests_) {
                if (dest_idx == prep::ext_node::invalid()) {
                    continue;
                }
                add(dest_idx, prep_order2);
            }
            while (!prep_order2.empty()) {
                auto const u = prep_order2.front();
                prep_order2.pop();
                if (node_costs_.count(cost_function_.get_flatten_node_idx(u)) == 0) {
                    continue;
                }
                auto const curr_cost = node_costs_[cost_function_.get_flatten_node_idx(u)].cost_;
                for (auto const& uv : cost_function_.get_downward_edges(u)) {
                    utl::verify(cost_function_.extended_edges_[uv].from_ == u, "downward edge from mismatch, expected {} but got {}", prep::to_string(u), prep::to_string(cost_function_.extended_edges_[uv].from_));
                    if (curr_cost >= kInfeasible - cost_function_.extended_edges_[uv].cost_) {
                        continue;
                    }
                    auto const v = cost_function_.extended_edges_[uv].to_;
                    auto const v_idx = cost_function_.get_flatten_node_idx(v);
                    auto const new_cost = curr_cost + cost_function_.extended_edges_[uv].cost_;

                    if (!node_costs_.count(v_idx)) {
                        node_costs_[v_idx] = prep::node_entry::invalid();
                    }
                    node_costs_[v_idx].update(new_cost, uv);
                }
            }
            is_marked_.clear();

            return true;
        }

        hash_map<node_idx_t, prep::node_entry> node_costs_;
        hash_map<node_idx_t, bool> is_marked_;
        std::vector<prep::ext_node> starts_, dests_;

        prep::customized_cost_stored<P> const& cost_function_ = prep::preprocessed_data::get_customized_cost<P>();
        prep::elimination_tree const& tree_                   = prep::preprocessed_data::get_elimination_tree();
        prep::node_ordering const& ordering_                  = prep::preprocessed_data::get_ordering();
    };
} // namespace osr