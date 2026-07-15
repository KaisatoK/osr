#pragma once

#include <algorithm>
#include <filesystem>
#include <iterator>
#include <list>
#include <stack>
#include <optional>
#include <utility>
#include <vector>

#include "fmt/core.h"
#include "fmt/std.h"

#include "utl/progress_tracker.h"

#include "osr/types.h"
#include "osr/ways.h"
#include "osr/elevation_storage.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/profile.h"
#include "osr/cch_preprocessing/contracted_graph.h"
#include "osr/cch_preprocessing/node_ordering.h"
#include "osr/cch_preprocessing/profile_name.h"

namespace osr::cch_preprocessing {

    namespace fs = std::filesystem;

    using ext_node_idx_t = pair<node_idx_t, std::uint16_t>;
    using ext_edge_idx_t = way_idx_t;

    std::string to_string(ext_node_idx_t idx) {
        return fmt::format("({}, {})", idx.first, idx.second);
    }

    std::string to_string(ext_edge_idx_t idx) {
        return fmt::format("({})", idx);
    }

    struct ext_edge {
        static constexpr std::size_t const kMaxTracebackSize = 4U;

        inline bool is_original() const{
            return traceback_.empty();
        }

        void update(cost_t const new_cost, ext_edge_idx_t const first_half, ext_edge_idx_t const second_half) {
            if (new_cost < cost_) {
                cost_ = new_cost;
                traceback_.clear();
                traceback_.emplace_back(pair(first_half, second_half));
            } else if (new_cost == cost_ && traceback_.size() < kMaxTracebackSize) {
                traceback_.emplace_back(pair(first_half, second_half));
            }
        }

        void update(cost_t const new_cost) {
            if (new_cost < cost_) {
                cost_ = new_cost;
                traceback_.clear();
            }
        }

        ext_node_idx_t from_, to_;
        cost_t cost_ = kInfeasible;
        vec<pair<ext_edge_idx_t, ext_edge_idx_t>> traceback_{};
    };

    template <Profile P>
    struct customized_cost_stored {
        using profile_t = P;
        using key = typename P::key;
        using label = typename P::label;
        using node = typename P::node;
        using entry = typename P::entry;
        using hash = typename P::hash;

        static inline auto const FILENAME = "customized_cost_stored_" + std::string(profile_name<P>::value) + ".bin";

        customized_cost_stored(node_ordering const& ordering) : ordering_{ordering} {
            virtual_nodes_  = vecvec<node_idx_t, node>();
            idx_ranges_     = vec_map<node_idx_t, std::uint32_t>();
            extended_edges_ = vec_map<ext_edge_idx_t, ext_edge>();
            upward_edges_   = vecvec<node_idx_t, ext_edge_idx_t>();
            downward_edges_ = vecvec<node_idx_t, ext_edge_idx_t>();
        }

        std::vector<ext_edge> trace_sg_edge(ext_edge_idx_t const& edge_idx) const {
            std::vector<ext_edge> res{0};
            std::stack<ext_edge_idx_t> st{};
            st.push(edge_idx);
            while (!st.empty()) {
                auto const uv = st.top();
                auto const edge = extended_edges_.at(uv);
                st.pop();

                if (edge.is_original()) {
                    res.emplace_back(edge);
                } else { // we only care about the first pair for now
                    utl::verify(get_edge(edge.traceback_.front().first).cost_ + get_edge(edge.traceback_.front().second).cost_ == edge.cost_,
                                "traceback cost mismatch, from tuple {}, {} and {}",
                                to_string(uv), to_string(edge.traceback_.front().first), to_string(edge.traceback_.front().second));

                    utl::verify(edge.from_ == get_edge(edge.traceback_.front().first).from_ && edge.to_ == get_edge(edge.traceback_.front().second).to_,
                                "traceback from/to mismatch, from tuple {}, {} and {}",
                                to_string(uv), to_string(edge.traceback_.front().first), to_string(edge.traceback_.front().second));

                    utl::verify(get_edge(edge.traceback_.front().first).to_ == get_edge(edge.traceback_.front().second).from_,
                                "traceback transit mismatch, from tuple {}, {} and {}",
                                to_string(uv), to_string(edge.traceback_.front().first), to_string(edge.traceback_.front().second));
                    
                    st.push(edge.traceback_.front().second);
                    st.push(edge.traceback_.front().first);
                }
            }
            return res;
        }

        ext_edge get_edge(ext_edge_idx_t const& idx) const {
            return extended_edges_.at(idx);
        }

        std::optional<ext_node_idx_t> get_virtual_node_idx(node const& n) const {
            if (n.get_node() == node_idx_t::invalid() || n.get_node() >= static_cast<node_idx_t>(virtual_nodes_.size())) {
                return std::nullopt;
            }

            auto const& vnodes = virtual_nodes_[n.get_node()];
            auto it = std::lower_bound(vnodes.begin(), vnodes.end(), n);

            if (it == vnodes.end() || !(*it == n)) {
                return std::nullopt;
            }

            return ext_node_idx_t{n.get_node(), static_cast<std::uint16_t>(std::distance(vnodes.begin(), it))};
        }

        node get_virtual_node(ext_node_idx_t const& idx) const {
            return virtual_nodes_[idx.first][idx.second];
        }

        std::optional<ext_node_idx_t> get_parent(ext_node_idx_t const& idx) const {
            if (get_upward_edges(idx).empty()) {
                return std::nullopt;
            }
            return extended_edges_[get_upward_edges(idx).front()].to_;
        }

        std::optional<ext_node_idx_t> get_parent(node const& n) const {
            return get_parent(get_virtual_node_idx(n));
        }

        vecvec<node_idx_t, ext_edge_idx_t>::const_bucket get_upward_edges(ext_node_idx_t const& idx) const {
            return upward_edges_[static_cast<node_idx_t>(idx_ranges_.at(idx.first) + idx.second)];
        }

        vecvec<node_idx_t, ext_edge_idx_t>::const_bucket get_downward_edges(ext_node_idx_t const& idx) const {
            return downward_edges_[static_cast<node_idx_t>(idx_ranges_.at(idx.first) + idx.second)];
        }

        template <std::size_t NMaxTypes>
        friend constexpr auto static_type_hash(
            customized_cost_stored<P> const*, cista::hash_data<NMaxTypes> h) noexcept {
            using cista::static_type_hash;
            h = h.combine(cista::hash("customized_cost_stored v1.0"));
            h = static_type_hash(cista::null<P>(), h);
            return h;
        }

        template <typename Ctx>
        friend void serialize(Ctx&, customized_cost_stored<P> const*, cista::offset_t) {}

        template <typename Ctx>
        friend void deserialize(Ctx const&, customized_cost_stored<P>*) {}
        
        static cista::wrapped<customized_cost_stored<P>> read(fs::path const& path) {
            return cista::read<customized_cost_stored<P>>(path / fs::path{FILENAME});
        }

        void write(fs::path const& path) const {
            return cista::write(path / fs::path{FILENAME}, *this);
        }

        node_ordering const& ordering_;
        vecvec<node_idx_t, node> virtual_nodes_;
        vec_map<node_idx_t, std::uint32_t> idx_ranges_;
        vec_map<ext_edge_idx_t, ext_edge> extended_edges_;
        vecvec<node_idx_t, ext_edge_idx_t> upward_edges_, downward_edges_;
    };

    template <Profile P>
    struct customized_cost {
        using profile_t = P;
        using key = typename P::key;
        using label = typename P::label;
        using node = typename P::node;
        using entry = typename P::entry;
        using hash = typename P::hash;

        using list_it = std::list<ext_edge_idx_t>::iterator;

        customized_cost(node_ordering const& ordering) : ordering_{ordering} {
            virtual_nodes_  = std::vector<std::vector<node>>(ordering.size());
            extended_edges_ = std::vector<std::pair<list_it, ext_edge>>();
            upward_edges_   = std::vector<std::vector<std::list<ext_edge_idx_t>>>(ordering.size());
            downward_edges_ = std::vector<std::vector<std::list<ext_edge_idx_t>>>(ordering.size());
        };

        std::list<ext_edge_idx_t> const& get_upward_edges(ext_node_idx_t const& idx) const {
            return upward_edges_[idx.first.v_][idx.second];
        }

        std::list<ext_edge_idx_t>& get_upward_edges(ext_node_idx_t const& idx) {
            return upward_edges_[idx.first.v_][idx.second];
        }

        std::list<ext_edge_idx_t> const& get_downward_edges(ext_node_idx_t const& idx) const {
            return downward_edges_[idx.first.v_][idx.second];
        }

        std::list<ext_edge_idx_t>& get_downward_edges(ext_node_idx_t const& idx) {
            return downward_edges_[idx.first.v_][idx.second];
        }

        ext_edge const& get_edge(ext_edge_idx_t const& idx) const {
            return extended_edges_[idx.v_].second;
        }

        ext_edge& get_edge(ext_edge_idx_t const& idx) {
            return extended_edges_[idx.v_].second;
        }

        ext_edge_idx_t get_rev_edge(ext_edge_idx_t const& uv) const {
            return static_cast<ext_edge_idx_t>(uv.v_ ^ 1);
        }

        bool ext_nodes_comp(ext_node_idx_t const& a, ext_node_idx_t const& b) {
            return std::tie(ordering_.get_ordering(a.first), a.second) < std::tie(ordering_.get_ordering(b.first), b.second);
        }

        template <bool IsReversed, typename Fn>
        void for_each_vir_node(Fn&& fn) {
            bool isDebug = true;
            if constexpr (IsReversed) {
                for (auto it = ordering_.new_to_old_.rbegin(); it != ordering_.new_to_old_.rend(); ++it) {
                    auto const& nodes = virtual_nodes_[it->v_];
                    for (std::size_t sub_idx = nodes.size(); sub_idx > 0; --sub_idx) {
                        fn(nodes[sub_idx - 1], ext_node_idx_t{*it, static_cast<std::uint16_t>(sub_idx - 1)});
                    }
                }
            } else {
                for (auto const& idx : ordering_.new_to_old_) {
                    utl::verify(idx.v_ < virtual_nodes_.size(), "Invalid node index {} for virtual nodes of size {}", idx, virtual_nodes_.size());
                    auto const& nodes = virtual_nodes_[idx.v_];
                    isDebug = (ordering_.get_ordering(idx).v_ % 10000 == 0);
                    if (isDebug) {
                        fmt::println("Processing virtual nodes for original node {}: {}", idx, nodes.size());
                    }
                    for (auto const& [sub_idx, node] : utl::enumerate(nodes)) {
                        fn(node, ext_node_idx_t{idx, static_cast<std::uint16_t>(sub_idx)});
                    }
                }
            }
        }

        std::optional<ext_node_idx_t> get_virtual_node_idx(node const& n) const {
            if (n.get_node() == node_idx_t::invalid() || n.get_node() >= static_cast<node_idx_t>(virtual_nodes_.size())) {
                return std::nullopt;
            }

            auto const& vnodes = virtual_nodes_[n.get_node().v_];
            auto it = std::lower_bound(vnodes.begin(), vnodes.end(), n);

            if (it == vnodes.end() || !(*it == n)) {
                return std::nullopt;
            }

            return ext_node_idx_t{n.get_node(), static_cast<std::uint16_t>(std::distance(vnodes.begin(), it))};
        }

        void add_edge(ext_node_idx_t const& from_idx, ext_node_idx_t const& to_idx, std::optional<list_it> from_ins_pos = std::nullopt) {
            utl::verify(!ext_nodes_comp(to_idx, from_idx), "Expected upward edge from {} to {}, but ordering is not correct, odering {}, {}", to_string(from_idx), to_string(to_idx), ordering_.get_ordering(from_idx.first), ordering_.get_ordering(to_idx.first));

            if (!from_ins_pos) from_ins_pos = std::make_optional(get_upward_edges(from_idx).end());

            get_upward_edges(from_idx).emplace(*from_ins_pos, extended_edges_.size());
            extended_edges_.emplace_back(std::make_pair(std::prev(*from_ins_pos), ext_edge{from_idx, to_idx}));
            get_downward_edges(to_idx).emplace_back(extended_edges_.size());
            extended_edges_.emplace_back(std::make_pair(std::prev(get_downward_edges(to_idx).end()), ext_edge{to_idx, from_idx}));
        }

        void build_node_mapping(ways::routing const& r) {
            auto pt = utl::get_active_progress_tracker_or_activate("osr-cch-preprocess");

            pt->status("Build virtual node mapping").in_high(ordering_.size()).out_bounds(10, 20);

            for (auto const& cur_node : ordering_.new_to_old_) {
                P::resolve_all(r, cur_node, kNoLevel, [&](node const n) {
                    virtual_nodes_[cur_node.v_].emplace_back(n);
                });
                utl::sort(virtual_nodes_[cur_node.v_]);
            }
            for (auto const& cur_node : ordering_.new_to_old_) {
                upward_edges_[cur_node.v_].resize(virtual_nodes_[cur_node.v_].size());
                downward_edges_[cur_node.v_].resize(virtual_nodes_[cur_node.v_].size());
            }

            pt->update(ordering_.size());
        }

        void transfer_edges_cost(typename P::parameters const& params,
                                ways::routing const& r, 
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const* additional,
                                elevation_storage const* elevation) {

            auto pt = utl::get_active_progress_tracker_or_activate("osr-cch-preprocess");

            pt->status("Transfer edges cost").in_high(ordering_.size()).out_bounds(20, 40);
            for_each_vir_node<false>(
                [&](node const& from, ext_node_idx_t const& from_idx) {
                    bool isDebug = (ordering_.get_ordering(from_idx.first).v_ % 10000 == 0);
                    P::template adjacent<direction::kForward, false>(params, r, from, duration_t{0}, std::nullopt, blocked, additional, elevation, 
                        [&](node const to,
                            std::uint32_t const cost,
                            duration_t, distance_t, way_idx_t const, std::uint16_t, std::uint16_t, elevation_storage::elevation, bool const) {
                                
                            utl::verify(get_virtual_node_idx(to).has_value(), "Virtual node index for node {} not found", to.get_node());
                            auto const to_idx = *get_virtual_node_idx(to);
                            if (ext_nodes_comp(from_idx, to_idx)) {
                                if (isDebug) {
                                    fmt::println("Adding/updating edge from {} to {} with cost {}\n", to_string(from_idx), to_string(to_idx), cost);
                                }

                                add_edge(from_idx, to_idx);

                                if (isDebug) {
                                    fmt::println("Current adjacent edges from {}: {}", to_string(from_idx), get_upward_edges(from_idx).size());
                                    fmt::println("Current adjacent edges to {}: {}", to_string(to_idx), get_downward_edges(to_idx).size());
                                }

                                get_edge(get_upward_edges(from_idx).back()).update(cost);

                                if (isDebug) {
                                    fmt::println("Edge from {} to {} has cost {}\n", to_string(from_idx), to_string(to_idx), get_edge(get_upward_edges(from_idx).back()).cost_);
                                }
                            } else {
                                bool found = false;
                                for (auto const& adj_edge : get_downward_edges(from_idx)) {
                                    if (get_edge(adj_edge).to_ == to_idx) {
                                        get_edge(adj_edge).update(cost);
                                        if (isDebug) {
                                            fmt::println("Updating edge from {} to {} with cost {}\n", to_string(get_edge(adj_edge).from_), to_string(get_edge(adj_edge).to_), cost);
                                        }
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found) {
                                    add_edge(to_idx, from_idx);

                                    if (isDebug) {
                                        fmt::println("Adding edge from {} to {} with cost {}\n", to_string(to_idx), to_string(from_idx), cost);
                                        fmt::println("Current adjacent edges from {}: {}", to_string(to_idx), get_upward_edges(to_idx).size());
                                        fmt::println("Current adjacent edges to {}: {}", to_string(from_idx), get_downward_edges(from_idx).size());
                                    }

                                    get_edge(get_downward_edges(from_idx).back()).update(cost);

                                    if (isDebug) {
                                        fmt::println("Edge from {} to {} has cost {}\n", to_string(get_edge(get_downward_edges(from_idx).back()).from_), to_string(get_edge(get_downward_edges(from_idx).back()).to_), get_edge(get_downward_edges(from_idx).back()).cost_);
                                    }
                                }
                            }
                        }
                    );
                }
            );

            pt->update(ordering_.size());
        }

        void customize(typename P::parameters const& params,
                        ways::routing const& r,
                        bitvec<node_idx_t> const* blocked = nullptr,
                        sharing_data const* additional = nullptr,
                        elevation_storage const* elevation = nullptr) {
            build_node_mapping(r);
            transfer_edges_cost(params, r, blocked, additional, elevation);
            
            auto pt = utl::get_active_progress_tracker_or_activate("osr-cch-preprocess");

            auto const feasible_check = [&](ext_edge_idx_t const& uv, ext_edge_idx_t const& vw) -> bool {
                return get_edge(uv).cost_ != kInfeasible && get_edge(vw).cost_ != kInfeasible;
            };

            auto const add_cost = [&](ext_edge_idx_t const& uv, ext_edge_idx_t const& vw, ext_edge_idx_t const& uw) {
                if (!feasible_check(uv, vw)) {
                    return;
                }
                auto const new_cost = get_edge(uv).cost_ + get_edge(vw).cost_;
                get_edge(uw).update(new_cost, uv, vw);
            };

            pt->status("Customizing cost").in_high(ordering_.size()).out_bounds(40, 80);

            for_each_vir_node<false>(
                [&](node const&, ext_node_idx_t const& u_idx) {
                    get_upward_edges(u_idx).sort([&](ext_edge_idx_t const& a, ext_edge_idx_t const& b) {
                        auto const& ato = get_edge(a).to_;
                        auto const& bto = get_edge(b).to_;
                        return ext_nodes_comp(ato, bto);
                    });

                    for (auto const& uv : get_downward_edges(u_idx)) {
                        if (!feasible_check(get_rev_edge(uv), get_rev_edge(uv)) || !feasible_check(uv, uv)) {
                            continue;
                        }

                        auto const& v_idx = get_edge(uv).to_;
                        auto const& v_edges = get_upward_edges(v_idx);
                        auto u_edge_it = get_upward_edges(u_idx).end();

                        for (auto v_edge_it = v_edges.end(); std::prev(v_edge_it) != v_edges.begin(); v_edge_it = std::prev(v_edge_it)) {
                            auto const& vw = *std::prev(v_edge_it);
                            auto const& w_idx = get_edge(vw).to_;

                            if (!ext_nodes_comp(u_idx, w_idx)) {
                                break;
                            }

                            if (!feasible_check(uv, vw) && !feasible_check(get_rev_edge(vw), get_rev_edge(uv))) {
                                continue;
                            }

                            while (u_edge_it != get_upward_edges(u_idx).begin() && ext_nodes_comp(w_idx, get_edge(*std::prev(u_edge_it)).to_)) {
                                u_edge_it = std::prev(u_edge_it);
                            }
                            if (u_edge_it == get_upward_edges(u_idx).begin() || ext_nodes_comp(get_edge(*std::prev(u_edge_it)).to_, w_idx)) {
                                add_edge(u_idx, w_idx, u_edge_it);
                            }

                            auto const& uw = *std::prev(u_edge_it);

                            if (ordering_.get_ordering(u_idx.first).v_ % 10000 == 0) {
                                fmt::println("Customizing cost for edge from {} to {} via {}: uv {}, vw {}, uw {}", to_string(u_idx), to_string(w_idx), to_string(v_idx), uv.v_, vw.v_, uw.v_);
                            }

                            add_cost(uv, vw, uw);
                            add_cost(get_rev_edge(vw), get_rev_edge(uv), get_rev_edge(uw));
                        }
                    }
                }
            );

            pt->update(ordering_.size());
        }

        customized_cost_stored<P> store() {
            auto pt = utl::get_active_progress_tracker_or_activate("osr-cch-preprocess");

            pt->status("Store customized cost").in_high(ordering_.size()).out_bounds(80, 100);

            customized_cost_stored<P> stored{ordering_};
            
            std::size_t total_idxes = 0;
            for (auto const& nodes : virtual_nodes_) {
                stored.virtual_nodes_.emplace_back(nodes);
                stored.idx_ranges_.emplace_back(static_cast<std::uint32_t>(total_idxes));
                total_idxes += nodes.size();
            }

            for (auto const& [it, edge] : extended_edges_) {
                stored.extended_edges_.emplace_back(edge);
            }

            for_each_vir_node<false>(
                [&](node const&, ext_node_idx_t const& idx) {
                    stored.upward_edges_.emplace_back(get_upward_edges(idx));
                    stored.downward_edges_.emplace_back(get_downward_edges(idx));
                }
            );

            pt->update(ordering_.size());

            return stored;
        }

        node_ordering const& ordering_;
        std::vector<std::vector<node>> virtual_nodes_;
        std::vector<std::pair<list_it, ext_edge>> extended_edges_;
        std::vector<std::vector<std::list<ext_edge_idx_t>>> upward_edges_, downward_edges_;
    };
} // namespace osr::cch_preprocessing