#include <iostream>

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "utl/progress_tracker.h"

#include "osr/cch_preprocessing/preprocess.h"

using namespace osr;
using namespace boost::program_options;

namespace fs = std::filesystem;
namespace prep = cch_preprocessing;

struct config : public conf::configuration {
    config(fs::path dir) : configuration{"Options"}, dir_{std::move(dir)} {
        param(dir_, "in,i", "Routing data directory");
    }

    fs::path dir_;
};

int main(int ac, char const** av) {
    auto opt = config{"."};
    auto parser = conf::options_parser({&opt});
    parser.read_command_line_args(ac, av);

    if (parser.help()) {
        parser.print_help(std::cout);
        return 0;
    } else if (parser.version()) {
        return 0;
    }

    parser.read_configuration_file();
    parser.print_unrecognized(std::cout);
    parser.print_used(std::cout);

    if (!fs::is_directory(opt.dir_)) {
        fmt::println("directory not found: {}", opt.dir_);
        return 1;
    }

    utl::activate_progress_tracker("osr-cch-preprocess");
    auto const silencer = utl::global_progress_bars{false};

    prep::preprocess(opt.dir_);
}