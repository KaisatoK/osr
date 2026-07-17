#include "osr/cch_preprocessing/customized_cost.h"

namespace osr::cch_preprocessing {
    std::string to_string(ext_node_idx_t idx) {
        return fmt::format("({}, {})", idx.first, idx.second);
    }

    std::string to_string(ext_edge_idx_t idx) {
        return fmt::format("({})", idx);
    }
}