#include "osr/cch_preprocessing/preprocess.h"

#include "utl/progress_tracker.h"

#include "osr/ways.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profile.h"
#include "osr/routing/with_profile.h"
#include "osr/cch_preprocessing/node_ordering.h"
#include "osr/cch_preprocessing/customized_cost.h"

namespace osr::cch_preprocessing {

    template <Profile P>
    auto build_cost_function(ways const& w, node_ordering const& ordering) {
        auto const pp = typename P::parameters{};
        auto cost_function = customized_cost<P>{ordering};

        cost_function.customize(pp, *w.r_);
        return cost_function.store();
    }

    void preprocess(fs::path const& in) {
        auto pt = utl::get_active_progress_tracker_or_activate("osr-cch-preprocess");

        pt->status("Load Routing Data / Node Ordering").in_high(2).out_bounds(0, 10);

        auto const w = ways{in, cista::mmap::protection::READ};
        pt->update(1);

        auto const ordering = std::make_unique<node_ordering>(node_ordering::randomize(w.n_nodes(), 0xdeadbeef));
        pt->update(1);
        
        auto build = [&](search_profile const& profile) {
            with_profile(profile, [&]<Profile P>(P&&) {
                auto cost_function = build_cost_function<P>(w, *ordering);
                cost_function.write(in);
            });
        };

        build(search_profile::kCar);
        
        fmt::println("Finished CCH preprocessing!");
    }

} // namespace osr::cch_preprocessing