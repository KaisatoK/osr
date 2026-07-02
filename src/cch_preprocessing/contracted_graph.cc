#include "osr/cch_preprocessing/contracted_graph.h"

namespace osr::cch_preprocessing {
    contracted_graph::contracted_graph(node_ordering const& ordering, ways::routing const& route)
        : ordering_(ordering) {
            build_graph(route);
        }
        
    void contracted_graph::build_graph(ways::routing const& route) {
        utl::verify(route.node_properties_.size() == ordering_.size(),
        "node_properties and node_ordering size mismatch, expected {} but got {}",
        route.node_properties_.size(), ordering_.size());
        
        std::vector<std::vector<node_idx_t>> additional_edges;

        additional_edges.resize(ordering_.size());
        elimination_tree_.resize(ordering_.size());
        upward_edges_.resize(ordering_.size());
        downward_edges_.resize(ordering_.size());
        
        for (auto const& [new_idx, node] : utl::zip_unchecked(ordering_.old_to_new_, ordering_.new_to_old_)) {
            auto neighbors = std::move(additional_edges[static_cast<std::uint32_t>(node)]);
            auto const& add_node = [&](node_idx_t const node) {
                if (ordering_.get_ordering(node) > new_idx) {
                    neighbors.push_back(node);
                }
            };

            for (auto const [way, idx] : utl::zip_unchecked(route.node_ways_[node], route.node_in_way_idx_[node])) {
                if (idx > 0) {
                    add_node(route.way_nodes_[way][idx - 1]);
                }
                if (idx < route.way_nodes_[way].size() - 1U) {
                    add_node(route.way_nodes_[way][idx + 1]);
                }
            }

            {
                std::sort(neighbors.begin(), neighbors.end(), [&](node_idx_t const a, node_idx_t const b) {
                    return ordering_.get_ordering(a) < ordering_.get_ordering(b);
                });
                neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
            }
            auto const lowest_neighbor = static_cast<std::uint32_t>(neighbors.front());

            elimination_tree_[node] = static_cast<node_idx_t>(lowest_neighbor);
            for (auto const& neighbor : neighbors) {
                add_edge(new_idx, neighbor);
                additional_edges[lowest_neighbor].push_back(neighbor);
            }
        }

        edge_to_idx_.reserve(edges_.size());

        for (auto const& node_edges : upward_edges_) {
            std::sort(node_edges.begin(), node_edges.end(), [&](edge_idx_t const a, edge_idx_t const b) {
                auto const& edge_a = edges_[a];
                auto const& edge_b = edges_[b];
                return ordering_.get_ordering(edge_a.second) < ordering_.get_ordering(edge_b.second);
            });
        }

        for (auto const& node_edges : downward_edges_) {
            std::sort(node_edges.begin(), node_edges.end(), [&](edge_idx_t const a, edge_idx_t const b) {
                auto const& edge_a = edges_[a];
                auto const& edge_b = edges_[b];
                return ordering_.get_ordering(edge_a.second) < ordering_.get_ordering(edge_b.second);
            });
        }

    }

    void contracted_graph::add_edge(node_idx_t from, node_idx_t to) {
        if (ordering_.get_ordering(from) > ordering_.get_ordering(to)) {
            std::swap(from, to);
        }
        edges_.push_back({from, to});
        upward_edges_[from].push_back(static_cast<edge_idx_t>(edges_.size() - 1U));
        edge_to_idx_[{from, to}] = static_cast<edge_idx_t>(edges_.size() - 1U);
        edges_.push_back({to, from});
        downward_edges_[to].push_back(static_cast<edge_idx_t>(edges_.size() - 1U));
        edge_to_idx_[{to, from}] = static_cast<edge_idx_t>(edges_.size() - 1U);
    }

    constexpr edge_idx_t contracted_graph::get_edge_idx(node_idx_t const& from, node_idx_t const& to) const {
        auto const it = edge_to_idx_.find({from, to});
        utl::verify(it != edge_to_idx_.end(), "edge from {} to {} not found", from, to);
        return it->second;
    }

    constexpr simple_edge contracted_graph::get_edge(edge_idx_t idx) const {
        utl::verify(idx < edges_.size(), "edge index {} out of bounds", idx);
        return edges_[idx];
    }

    cista::wrapped<contracted_graph> contracted_graph::read(std::filesystem::path const& path) {
        return cista::read<contracted_graph>(path / "contracted_graph.bin");
    }

    void contracted_graph::write(std::filesystem::path const& path) const {
        return cista::write(path / "contracted_graph.bin", *this);
    }
}