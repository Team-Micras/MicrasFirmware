/**
 * @file
 */

#include <cstdint>
#include <span>

#include "micras/hal/fmac.hpp"

namespace micras::hal {
Fmac::Fmac(const Config& config) : handle{config.handle} {
    if (this->handle->State == HAL_FMAC_STATE_RESET) {
        config.init_function();
    }
}

bool Fmac::configure_iir(std::span<const int16_t> feed_forward, std::span<const int16_t> feedback) {
    this->initialized = false;

    if (feed_forward.empty() or feedback.empty() or feed_forward.size() + feedback.size() > buffer_size) {
        return false;
    }

    FMAC_FilterConfigTypeDef filter_config{
        .InputBaseAddress = input_base_address,
        .InputBufferSize = buffer_size,
        .InputThreshold = FMAC_THRESHOLD_1,
        .CoeffBaseAddress = coefficient_base_address,
        .CoeffBufferSize = buffer_size,
        .OutputBaseAddress = output_base_address,
        .OutputBufferSize = buffer_size,
        .OutputThreshold = FMAC_THRESHOLD_1,
        // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast) the HAL only reads the coefficients
        .pCoeffA = const_cast<int16_t*>(feedback.data()),
        .CoeffASize = static_cast<uint8_t>(feedback.size()),
        .pCoeffB = const_cast<int16_t*>(feed_forward.data()),
        .CoeffBSize = static_cast<uint8_t>(feed_forward.size()),
        // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
        .InputAccess = FMAC_BUFFER_ACCESS_POLLING,
        .OutputAccess = FMAC_BUFFER_ACCESS_POLLING,
        // Saturate instead of wrapping, so that a transient beyond the q1.15 range degrades the
        // output rather than inverting it
        .Clip = FMAC_CLIP_ENABLED,
        .Filter = FMAC_FUNC_IIR_DIRECT_FORM_1,
        .P = static_cast<uint8_t>(feed_forward.size()),
        .Q = static_cast<uint8_t>(feedback.size()),
        .R = 0,
    };

    if (HAL_FMAC_FilterConfig(this->handle, &filter_config) != HAL_OK) {
        return false;
    }

    if (HAL_FMAC_FilterStart(this->handle, this->output_buffer.data(), &this->output_buffer_size) != HAL_OK) {
        return false;
    }

    this->initialized = true;
    return true;
}

int16_t Fmac::update(int16_t sample) {
    this->handle->Instance->WDATA = static_cast<uint16_t>(sample);

    while ((this->handle->Instance->SR & FMAC_SR_YEMPTY) != 0) { }

    return static_cast<int16_t>(this->handle->Instance->RDATA & FMAC_WDATA_WDATA);
}

bool Fmac::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
