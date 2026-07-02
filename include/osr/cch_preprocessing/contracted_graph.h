#pragma once

#include <deque>
#include <map>

#include "osr/types.h"
#include "osr/ways.h"
#include "osr/cch_preprocessing/node_ordering.h"

#include "utl/verify.h"
#include "utl/erase_duplicates.h"
#include "utl/enumerate.h"
#include "utl/zip.h"

namespace osr::cch_preprocessing {

    using edge_idx_t = way_idx_t;
    using simple_edge = pair<node_idx_t, node_idx_t>;

    struct contracted_graph {
        contracted_graph(node_ordering const&, ways::routing const&);

        void build_graph(ways::routing const&);
        void add_edge(node_idx_t from, node_idx_t to);
        constexpr edge_idx_t get_edge_idx(node_idx_t const&, node_idx_t const&) const;
        constexpr simple_edge get_edge(edge_idx_t idx) const;

        template <std::size_t NMaxTypes>
        friend constexpr auto static_type_hash(
            contracted_graph const*, cista::hash_data<NMaxTypes> h) noexcept {
            return h.combine(cista::hash("contracted_graph v1.0"));
        }

        template <typename Ctx>
        friend void serialize(Ctx&, contracted_graph const*, cista::offset_t) {}

        template <typename Ctx>
        friend void deserialize(Ctx const&, contracted_graph*) {}

        static cista::wrapped<contracted_graph> read(std::filesystem::path const&);
        void write(std::filesystem::path const&) const;

        node_ordering const& ordering_;
        vec_map<node_idx_t, node_idx_t> elimination_tree_;

        vec_map<edge_idx_t, simple_edge> edges_;
        hash_map<simple_edge, edge_idx_t> edge_to_idx_; // TODO: optimize lookup to work in O(1)
        
        vecvec<node_idx_t, edge_idx_t> upward_edges_;
        vecvec<node_idx_t, edge_idx_t> downward_edges_;
    };
}