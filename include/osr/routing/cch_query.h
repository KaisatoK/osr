#pragma once

#include <stack>
#include <queue>
#include <array>

#include "osr/cch_preprocessing/extended_type.h"
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

            static constexpr node_entry const invalid() {
                return node_entry{
                    .curr_ = node::invalid(), 
                    .cost_ = kInfeasible, 
                    .pred_ = {prep::ext_edge_idx_t::invalid(), prep::ext_edge_idx_t::invalid(), prep::ext_edge_idx_t::invalid(), prep::ext_edge_idx_t::invalid()}
                };
            }

            prep::ext_edge_idx_t const pred() const {
                for (auto const& p : pred_) {
                    if (p != prep::ext_edge_idx_t::invalid()) {
                        return p;
                    }
                }
                return prep::ext_edge_idx_t::invalid();
            }

            void update(cost_t const new_cost, prep::ext_edge_idx_t const pred) {
                if (new_cost < cost_) {
                    cost_ = new_cost;
                    std::fill(pred_.begin(), pred_.end(), prep::ext_edge_idx_t::invalid());
                    pred_[0] = pred;
                } else if (new_cost == cost_) {
                    for (auto& p : pred_) {
                        if (p == prep::ext_edge_idx_t::invalid()) {
                            p = pred;
                            break;
                        }
                    }
                }
            }

            node curr_;
            cost_t cost_;
            std::array<prep::ext_edge_idx_t, kMaxPredSize> pred_;
        };

        cch_query(prep::customized_cost_stored<P>&& cost_function) : cost_function_{std::forward<prep::customized_cost_stored<P>>(cost_function)} {}

        cch_query(std::filesystem::path const& path) : cost_function_{*prep::customized_cost_stored<P>::read(path)} {}

        void reset() {
            starts_.clear();
            dests_.clear();
            node_costs_.clear();
            is_marked_.clear();
        }

        node_idx_t get_flatten_node_idx(prep::ext_node_idx_t const idx) const {
            return static_cast<node_idx_t>(cost_function_.idx_ranges_.at(idx.first) + idx.second);
        }

        node_entry const get_node_entry(node const n) const {
            auto const idx = get_flatten_node_idx(cost_function_.get_virtual_node_idx(n).value());
            if (node_costs_.count(idx)) {
                return node_costs_.at(idx);
            }
            return node_entry::invalid();
        }

        void add_start(node const n, cost_t const start_cost) {
            auto const start_idx = cost_function_.get_virtual_node_idx(n);
            if (start_idx) {
                starts_.emplace_back(n);
                auto content = node_entry::invalid();
                content.curr_ = n;
                content.cost_ = start_cost;
                node_costs_[get_flatten_node_idx(*start_idx)] = content;
            }
        }

        void add_dest(node const n) {
            auto const dest_idx = cost_function_.get_virtual_node_idx(n);
            if (dest_idx) {
                dests_.emplace_back(n);
            }
        }

        cost_t get_best_cost() const {
            cost_t best_cost = kInfeasible;
            for (auto const& dest : dests_) {
                auto const entry = get_node_entry(dest);
                if (entry.cost_ < best_cost) {
                    best_cost = entry.cost_;
                }
            }
            return best_cost;
        }

        template <typename Container>
        void add(prep::ext_node_idx_t u, Container& container) {
            if (is_marked_.count(get_flatten_node_idx(u))) {
                return;
            }
            std::stack<prep::ext_node_idx_t> st;
            st.push(u);
            while (true) {
                is_marked_[get_flatten_node_idx(u)] = true;
                auto par = cost_function_.get_parent(u);
                if (par && *par != u && !is_marked_.count(get_flatten_node_idx(*par))) {
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
            for (auto const& start : starts_) {
                auto start_idx = cost_function_.get_virtual_node_idx(start);
                add(*start_idx, prep_order1);
            }
            while (!prep_order1.empty()) {
                auto const u = prep_order1.top();
                prep_order1.pop();
                if (node_costs_.count(get_flatten_node_idx(u)) == 0) {
                    continue;
                }
                auto const curr_cost = node_costs_[get_flatten_node_idx(u)].cost_;
                for (auto const& uv : cost_function_.get_upward_edges(u)) {
                    utl::verify(cost_function_.extended_edges_[uv].from_ == u, "upward edge from mismatch, expected {} but got {}", osr::cch_preprocessing::to_string(u), osr::cch_preprocessing::to_string(cost_function_.extended_edges_[uv].from_));
                    if (curr_cost >= kInfeasible - cost_function_.extended_edges_[uv].cost_) {
                        continue;
                    }
                    auto const v = cost_function_.extended_edges_[uv].to_;
                    auto const v_idx = get_flatten_node_idx(v);
                    auto const new_cost = curr_cost + cost_function_.extended_edges_[uv].cost_;

                    if (!node_costs_.count(v_idx)) {
                        node_costs_[v_idx] = node_entry::invalid();
                    }
                    node_costs_[v_idx].update(new_cost, uv);
                }
            }
            is_marked_.clear();

            // second phase
            std::queue<prep::ext_node_idx_t> prep_order2{};
            for (auto const& dest : dests_) {
                auto dest_idx = cost_function_.get_virtual_node_idx(dest);
                add(*dest_idx, prep_order2);
            }
            while (!prep_order2.empty()) {
                auto const u = prep_order2.front();
                prep_order2.pop();
                if (node_costs_.count(get_flatten_node_idx(u)) == 0) {
                    continue;
                }
                auto const curr_cost = node_costs_[get_flatten_node_idx(u)].cost_;
                for (auto const& uv : cost_function_.get_downward_edges(u)) {
                    utl::verify(cost_function_.extended_edges_[uv].from_ == u, "downward edge from mismatch, expected {} but got {}", osr::cch_preprocessing::to_string(u), osr::cch_preprocessing::to_string(cost_function_.extended_edges_[uv].from_));
                    if (curr_cost >= kInfeasible - cost_function_.extended_edges_[uv].cost_) {
                        continue;
                    }
                    auto const v = cost_function_.extended_edges_[uv].to_;
                    auto const v_idx = get_flatten_node_idx(v);
                    auto const new_cost = curr_cost + cost_function_.extended_edges_[uv].cost_;

                    if (!node_costs_.count(v_idx)) {
                        node_costs_[v_idx] = node_entry::invalid();
                    }
                    node_costs_[v_idx].update(new_cost, uv);
                }
            }
            is_marked_.clear();

            return true;
        }

        prep::customized_cost_stored<P> const cost_function_;
        hash_map<node_idx_t, node_entry> node_costs_;
        hash_map<node_idx_t, bool> is_marked_;
        std::vector<node> starts_, dests_;
    };
} // namespace osr