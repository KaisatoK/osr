#pragma once

#include <vector>

#include "utl/enumerate.h"
#include "utl/zip.h"
#include "utl/verify.h"
#include "utl/erase_duplicates.h"

#include "osr/types.h"
#include "osr/ways.h"
#include "osr/cch_preprocessing/node_ordering.h"

namespace osr::cch_preprocessing {

  using graph = std::vector<std::vector<std::uint32_t>>;

  struct elimination_tree {
    static constexpr auto const kDoFastContraction = true;

    static void normal_contraction(graph& adj, std::vector<std::uint32_t>& res);
    static void fast_contraction(graph const& adj, std::vector<std::uint32_t>& res);
    static vec_map<node_idx_t, node_idx_t> compute(node_ordering const& ordering, ways const& w);
  };

} // namespace osr::cch_preprocessing