#pragma once

#include <filesystem>
#include <random>

#include "osr/types.h"
#include "osr/cch_preprocessing/extended_type.h"

#include "utl/verify.h"

namespace osr::cch_preprocessing {
    struct node_ordering {

        std::size_t size() const {
            utl::verify(old_to_new_.size() == new_to_old_.size(),
                        "old_to_new and new_to_old size mismatch, expected {} but got {}",
                        old_to_new_.size(), new_to_old_.size());
            return old_to_new_.size();
        }

        node_idx_t const get_ordering(node_idx_t const node) const {
            utl::verify(node < old_to_new_.size(), "node index {} out of bounds, max {}", node, old_to_new_.size());
            return old_to_new_.at(node);
        }

        node_idx_t const get_node(node_idx_t const ordering) const {
            utl::verify(ordering < new_to_old_.size(), "ordering index {} out of bounds, max {}", ordering, new_to_old_.size());
            return new_to_old_.at(ordering);
        }

        template <std::size_t NMaxTypes>
        friend constexpr auto static_type_hash(
            node_ordering const*, cista::hash_data<NMaxTypes> h) noexcept {
            using cista::static_type_hash;
            h = h.combine(cista::hash("node_ordering v1.0"));
            return h;
        }

        template <HasNodeImportance T>
        static node_ordering import(T const&);

        static node_ordering randomize(std::size_t num_nodes, std::uint32_t seed);

        vec_map<node_idx_t, node_idx_t> old_to_new_;
        vec_map<node_idx_t, node_idx_t> new_to_old_;
    };
} // namespace osr::cch_preprocessing