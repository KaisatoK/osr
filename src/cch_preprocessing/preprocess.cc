#include "osr/cch_preprocessing/preprocess.h"

namespace osr::cch_preprocessing {

    constexpr std::uint32_t const kSeed = 0xdeadbeef;

    template <Profile P, typename T>
    auto build_cost_function(ways const& w, T&& ordering) {
        auto const pp = typename P::parameters{};
        auto cost_function = customized_cost<P>{std::forward<T>(ordering)};

        cost_function.customize(pp, w);
        return cost_function.store();
    }

    void preprocess(fs::path const& in, ways const& w) {
        
        auto ordering = node_ordering::randomize(w.n_nodes(), kSeed);
        utl::verify(ordering.size() == w.n_nodes(), "ordering size {} does not match number of nodes {}", ordering.size(), w.n_nodes());
        
        // auto build = [&](search_profile const& profile) {
        //     with_profile(profile, [&]<Profile P>(P&&) {
        //         auto cost_function = build_cost_function<P>(w, *ordering);
        //         cost_function.write(in);
        //     });
        // };

        // build(search_profile::kCar);
        auto cf = build_cost_function<car>(w, std::move(ordering));
        cf.write(in);
        
        // fmt::println("Finished CCH preprocessing!");
    }

} // namespace osr::cch_preprocessing