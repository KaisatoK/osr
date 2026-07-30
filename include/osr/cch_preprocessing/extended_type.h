#pragma once

#include "osr/types.h"

namespace osr::cch_preprocessing {

    template <typename T> 
    concept HasNodeImportance = requires (T t, node_idx_t idx) 
        {
            { t.node_important_[idx] } noexcept -> std::same_as<std::uint32_t&>;
        };

    using ext_node_idx_t = pair<node_idx_t, std::uint16_t>;
    using ext_edge_idx_t = way_idx_t;

    constexpr std::string to_string(ext_node_idx_t idx) {
        return fmt::format("({}, {})", idx.first, idx.second);
    }

    constexpr std::string to_string(ext_edge_idx_t idx) {
        return fmt::format("({})", idx);
    }

    struct ext_edge {
        static constexpr std::size_t const kMaxTracebackSize = 4U;

        template <std::size_t NMaxTypes>
        friend constexpr auto static_type_hash(
            ext_edge const*, cista::hash_data<NMaxTypes> h) noexcept {
            using cista::static_type_hash;
            h = h.combine(cista::hash("ext_edge v1.0"));
            return h;
        }

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

} // namespace osr::cch_preprocessing