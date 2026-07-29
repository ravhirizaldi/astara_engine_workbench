#include "fsw/fsw.h"

#include <new>

#include "core/controller.hpp"
#include "validation/config_validation.hpp"

using fsw::internal::Controller;

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
    if (handle == nullptr) {
        if (output != nullptr) {
            *output = {};
            output->abi_version = FSW_ABI_VERSION;
            output->struct_size = sizeof(FswOutput);
            output->output_valid = 0;
            output->step_status = FSW_STATUS_INVALID_ARGUMENT;
        }
        return FSW_STATUS_INVALID_ARGUMENT;
    }
    return static_cast<Controller*>(handle)->step(input, output);
}

void fsw_destroy(FswHandle handle) {
    delete static_cast<Controller*>(handle);
}

const char* fsw_version(void) {
    return FSW_VERSION_STRING;
}

}  // extern "C"
