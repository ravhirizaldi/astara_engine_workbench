#include "fsw/fsw.h"

#include <new>

#include "core/controller.hpp"
#include "output/output_builder.hpp"
#include "validation/config_validation.hpp"

using fsw::internal::Controller;
using fsw::internal::clear_output;

extern "C" {

uint32_t fsw_abi_version(void) {
    return FSW_ABI_VERSION;
}

FswHandle fsw_create(const FswConfig* config) {
    if (config == nullptr || !fsw::internal::valid_config(*config)) {
        return nullptr;
    }
    return new (std::nothrow) Controller(*config);
}

void fsw_reset(FswHandle handle) {
    if (handle != nullptr) {
        static_cast<Controller*>(handle)->reset();
    }
}

int32_t fsw_step(
    FswHandle handle,
    const FswInput* input,
    FswOutput* output
) {
    if (output == nullptr) {
        return FSW_STATUS_INVALID_ARGUMENT;
    }
    if (handle == nullptr || input == nullptr) {
        clear_output(*output);
        return FSW_STATUS_INVALID_ARGUMENT;
    }
    return static_cast<Controller*>(handle)->step(*input, *output);
}

void fsw_destroy(FswHandle handle) {
    delete static_cast<Controller*>(handle);
}

const char* fsw_version(void) {
    return FSW_VERSION_STRING;
}

}  // extern "C"
