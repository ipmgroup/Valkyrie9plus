#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.tunnel.Protocol.h>

#define UAVCAN_TUNNEL_CALL_REQUEST_MAX_SIZE 63
#define UAVCAN_TUNNEL_CALL_REQUEST_SIGNATURE (0xDB11EDC510502658ULL)
#define UAVCAN_TUNNEL_CALL_REQUEST_ID 63

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class uavcan_tunnel_Call_cxx_iface;
#endif

struct uavcan_tunnel_CallRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = uavcan_tunnel_Call_cxx_iface;
#endif
    struct uavcan_tunnel_Protocol protocol;
    uint8_t                       channel_id;
    struct {
        uint8_t len;
        uint8_t data[60];
    } buffer;
};

#ifdef __cplusplus
extern "C" {
#endif

uint32_t uavcan_tunnel_CallRequest_encode(struct uavcan_tunnel_CallRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
                                          ,
                                          bool tao
#endif
);
bool uavcan_tunnel_CallRequest_decode(const CanardRxTransfer* transfer, struct uavcan_tunnel_CallRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_tunnel_CallRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_tunnel_CallRequest* msg, bool tao);
static inline bool _uavcan_tunnel_CallRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_tunnel_CallRequest* msg, bool tao);
void               _uavcan_tunnel_CallRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_tunnel_CallRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_tunnel_Protocol_encode(buffer, bit_ofs, &msg->protocol, false);
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->channel_id);
    *bit_ofs += 8;
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif
    const uint8_t buffer_len = msg->buffer.len > 60 ? 60 : msg->buffer.len;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 6, &buffer_len);
        *bit_ofs += 6;
    }
    for (size_t i = 0; i < buffer_len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->buffer.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode uavcan_tunnel_CallRequest, return true on failure, false on success
*/
bool _uavcan_tunnel_CallRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_tunnel_CallRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_uavcan_tunnel_Protocol_decode(transfer, bit_ofs, &msg->protocol, false)) {
        return true;
    }

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->channel_id);
    *bit_ofs += 8;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->buffer.len);
        *bit_ofs += 6;
    } else {
        msg->buffer.len = ((transfer->payload_len * 8) - *bit_ofs) / 8;
    }

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif
    if (msg->buffer.len > 60) {
        return true; /* invalid value */
    }
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
    for (size_t i = 0; i < msg->buffer.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->buffer.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_tunnel_CallRequest sample_uavcan_tunnel_CallRequest_msg(void);
#endif
#ifdef __cplusplus
}  // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
