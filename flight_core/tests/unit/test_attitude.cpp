#include "test_support.hpp"

#include <cmath>

#include "core/context.hpp"
#include "navigation/attitude.hpp"

using namespace fsw_test;

int main() {
    fsw::internal::Context context(default_config());
    context.attitude_initialized = true;
    fsw::internal::integrate_attitude(
        context, {0.0, 0.0, 0.1}, 1.0
    );
    const auto relative = fsw::internal::relative_attitude(context);
    const double norm = std::sqrt(
        relative[0] * relative[0]
        + relative[1] * relative[1]
        + relative[2] * relative[2]
        + relative[3] * relative[3]
    );
    REQUIRE(std::abs(norm - 1.0) < 1e-12);
    REQUIRE(relative[3] > 0.0);
    return EXIT_SUCCESS;
}
