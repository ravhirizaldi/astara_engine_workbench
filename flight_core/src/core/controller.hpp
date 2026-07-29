#pragma once

#include "core/context.hpp"

namespace fsw::internal {

class Controller {
public:
    explicit Controller(const FswConfig& config);

    void reset();
    int32_t step(const FswInput* input, FswOutput* output);

private:
    Context context_;
};

}  // namespace fsw::internal
