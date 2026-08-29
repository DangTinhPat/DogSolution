#include "baby_alpha2_protocol.h"

uint16_t BA2_EncodePosDes(float q_rad)
{
    float raw_f = (q_rad / 16.384f) * 32768.0f + 32768.0f;
    if (raw_f < 0.0f) { raw_f = 0.0f; }
    if (raw_f > 65535.0f) { raw_f = 65535.0f; }
    return (uint16_t)(raw_f + 0.5f);
}

uint16_t BA2_EncodeVelDes(float velocity_rad_s)
{
    float raw_f = (velocity_rad_s / 45.0f) * 32768.0f + 32768.0f;
    if (raw_f < 0.0f) { raw_f = 0.0f; }
    if (raw_f > 65535.0f) { raw_f = 65535.0f; }
    return (uint16_t)(raw_f + 0.5f);
}

float BA2_DecodePosAct(uint16_t raw)
{
    return ((float)raw - 32768.0f) / 65535.0f * 32.768f;
}

float BA2_DecodeVelAct(uint16_t raw)
{
    return ((float)raw - 32768.0f) / 65535.0f * 90.0f;
}

float BA2_DecodeTauAct(uint16_t raw)
{
    return ((float)raw - 32768.0f) / 65535.0f * 48.0f;
}

uint16_t BA2_EncodePosAct(float q_rad)
{
    float raw_f = (q_rad / 32.768f) * 65535.0f + 32768.0f;
    if (raw_f < 0.0f) { raw_f = 0.0f; }
    if (raw_f > 65535.0f) { raw_f = 65535.0f; }
    return (uint16_t)(raw_f + 0.5f);
}

uint16_t BA2_EncodeGainX100(float gain)
{
    if (gain < 0.0f) { gain = 0.0f; }
    return (uint16_t)((gain * 100.0f) + 0.5f);
}

uint16_t BA2_EncodeTauFf(float tau_nm, float tau_abs_limit_nm)
{
    if (tau_nm >  tau_abs_limit_nm) { tau_nm =  tau_abs_limit_nm; }
    if (tau_nm < -tau_abs_limit_nm) { tau_nm = -tau_abs_limit_nm; }
    float raw = (tau_nm + 24.0f) / 48.0f * 65535.0f;
    if (raw < 0.0f)     { raw = 0.0f; }
    if (raw > 65535.0f) { raw = 65535.0f; }
    return (uint16_t)(raw + 0.5f);
}

CAN_Frame BA2_BuildSystemFrame(uint32_t id, uint8_t opcode)
{
    CAN_Frame f = {0};
    f.id = id;
    f.extended_id = false;
    f.fd_format = true;
    f.bit_rate_switch = false;
    f.data_len = BA2_SYS_FRAME_LEN;
    f.data[0] = 0xF0U;
    f.data[11] = opcode;
    return f;
}

CAN_Frame BA2_BuildHandshakeFrame(uint32_t id)
{
    CAN_Frame f = BA2_BuildSystemFrame(id, BA2_OPCODE_HANDSHAKE);
    f.data[1] = 0x09U;
    return f;
}

CAN_Frame BA2_BuildCalibEncoderFrame(uint32_t id, uint16_t offset_raw)
{
    CAN_Frame f = BA2_BuildSystemFrame(id, BA2_OPCODE_CALIB_ENCODER);
    f.data[1] = (uint8_t)(offset_raw >> 8);
    f.data[2] = (uint8_t)(offset_raw & 0xFFU);
    return f;
}

CAN_Frame BA2_BuildSetupLimitsFrame(uint32_t id, uint16_t max_raw,
                                    uint16_t min_raw, uint16_t vmax_raw)
{
    CAN_Frame f = BA2_BuildSystemFrame(id, BA2_OPCODE_SETUP_LIMITS);
    f.data[1] = (uint8_t)(max_raw >> 8);
    f.data[2] = (uint8_t)(max_raw & 0xFFU);
    f.data[3] = (uint8_t)(min_raw >> 8);
    f.data[4] = (uint8_t)(min_raw & 0xFFU);
    f.data[5] = (uint8_t)(vmax_raw >> 8);
    f.data[6] = (uint8_t)(vmax_raw & 0xFFU);
    return f;
}

CAN_Frame BA2_BuildPdFrame(uint32_t id, float pos_rad, float velocity_rad_s,
                           float kp, float kd,
                           float tau_nm, float tau_abs_limit_nm)
{
    CAN_Frame f = {0};
    f.id = id;
    f.extended_id = false;
    f.fd_format = true;
    f.bit_rate_switch = false;
    f.data_len = BA2_SYS_FRAME_LEN;
    const uint16_t pos_raw = BA2_EncodePosDes(pos_rad);
    const uint16_t velocity_raw = BA2_EncodeVelDes(velocity_rad_s);
    const uint16_t kp_raw = BA2_EncodeGainX100(kp);
    const uint16_t kd_raw = BA2_EncodeGainX100(kd);
    f.data[0] = 0U;
    f.data[1] = (uint8_t)(pos_raw >> 8);
    f.data[2] = (uint8_t)(pos_raw & 0xFFU);
    f.data[3] = (uint8_t)(velocity_raw >> 8);
    f.data[4] = (uint8_t)(velocity_raw & 0xFFU);
    f.data[5] = (uint8_t)(kp_raw >> 8);
    f.data[6] = (uint8_t)(kp_raw & 0xFFU);
    f.data[7] = (uint8_t)(kd_raw >> 8);
    f.data[8] = (uint8_t)(kd_raw & 0xFFU);
    const uint16_t tau_raw = BA2_EncodeTauFf(tau_nm, tau_abs_limit_nm);
    f.data[9]  = (uint8_t)(tau_raw >> 8);
    f.data[10] = (uint8_t)(tau_raw & 0xFFU);
    f.data[11] = 0U;
    return f;
}
