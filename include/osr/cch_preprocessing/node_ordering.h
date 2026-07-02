#pragma once

#include <filesystem>
#include <random>

#include "cista/memory_holder.h"
#include "cista/reflection/comparable.h"

#include "osr/types.h"

#include "utl/verify.h"

namespace osr::cch_preprocessing {
    template <typename T> 
    concept HasNodeImportance = requires (T t, node_idx_t idx) 
        {
            { t.node_important_[idx] } noexcept -> std::same_as<std::uint32_t&>;
        };

    struct node_ordering {

        constexpr std::size_t size() const {
            utl::verify(old_to_new_.size() == new_to_old_.size(),
                        "old_to_new and new_to_old size mismatch, expected {} but got {}",
                        old_to_new_.size(), new_to_old_.size());
            return old_to_new_.size();
        }

        constexpr node_idx_t const get_ordering(node_idx_t const node) const {
            return old_to_new_.at(node);
        }

        constexpr node_idx_t const get_node(node_idx_t const ordering) const {
            return new_to_old_.at(ordering);
        }

        template <std::size_t NMaxTypes>
        friend constexpr auto static_type_hash(
            node_ordering const*, cista::hash_data<NMaxTypes> h) noexcept {
            return h.combine(cista::hash("node_ordering v1.0"));
        }

        template <typename Ctx>
        friend void serialize(Ctx&, node_ordering const*, cista::offset_t) {}

        template <typename Ctx>
        friend void deserialize(Ctx const&, node_ordering*) {}

        cista::wrapped<node_ordering> read(std::filesystem::path const& path) {
            return cista::read<node_ordering>(path / "node_ordering.bin");
        }

        void write(std::filesystem::path const& path) const {
            return cista::write(path / "node_ordering.bin", *this);
        }

        template <HasNodeImportance T>
        static node_ordering import(T const&);

        static node_ordering randomize(std::size_t num_nodes, std::uint32_t seed);

        vec_map<node_idx_t, node_idx_t> old_to_new_;
        vec_map<node_idx_t, node_idx_t> new_to_old_;
    };
} // namespace osr::cch_preprocessing