/**
 ******************************************************************************
 * @file    baby_alpha2_protocol.h
 * @brief   Tang MA HOA thuan tuy cho giao thuc BabyAlpha2 (12-byte CAN-FD).
 *          KHONG phu thuoc trang thai, KHONG dung Tick/CAN_Transmit - chi
 *          bien doi so <-> byte va dung CAN_Frame lam kieu du lieu output.
 *          Ke thua tu du an oneLeg (/home/dvt/OUT_SAVE/babyDog_test/oneLeg) - da chay
 *          that tren board FK743M5-XIH6 + driver dong co BabyAlpha2 that.
 *          Tach rieng khoi tang giao tiep (actuator_if.c) de co the kiem thu
 *          doc lap (vi du: ma hoa mot gia tri, so sanh byte mong doi) va tai
 *          su dung cho khop khac neu can.
 ******************************************************************************
 */
#ifndef BABY_ALPHA2_PROTOCOL_H
#define BABY_ALPHA2_PROTOCOL_H

#include <stdint.h>
#include "can.h"

#define BA2_SYS_FRAME_LEN  12U

#define BA2_OPCODE_PING            0xF0U
#define BA2_OPCODE_HANDSHAKE       0xF2U
#define BA2_OPCODE_CALIB_ENCODER   0xF4U
#define BA2_OPCODE_SETUP_LIMITS    0xF5U
#define BA2_OPCODE_MOTOR_ENABLE    0xFCU
#define BA2_OPCODE_MOTOR_DISABLE   0xFDU

/* Ma tra loi (byte 0 cua khung phan hoi, ID phan hoi luon = 0):
 *   PING      -> 0x08 + id
 *   HANDSHAKE -> 0x18 + id
 *   CALIB     -> 0x28 + id
 *   SETUP     -> 0x38 + id
 *   Page0 (telemetry vi tri)  -> 0x00 + id */
#define BA2_REPLY_PING(id)   ((uint8_t)(0x08U + (id)))
#define BA2_REPLY_HS(id)     ((uint8_t)(0x18U + (id)))
#define BA2_REPLY_CALIB(id)  ((uint8_t)(0x28U + (id)))
#define BA2_REPLY_SETUP(id)  ((uint8_t)(0x38U + (id)))
#define BA2_REPLY_PAGE0(id)  ((uint8_t)(0x00U + (id)))

/* ===== Ma hoa / giai ma so hoc =====
 * p_des:  HR=16.384 rad, raw = q/16.384*32768+32768
 * p_act:  HR=32.768 rad, raw = (raw-32768)/65535*32.768
 * v_des:  HR=45 rad/s, raw = v/45*32768+32768
 * v_act:  raw = (v+45)/90*65535
 * Kp/Kd:  raw = gain*100
 * tau_ff/tau_act: dai +-24 N.m, raw = (tau+24)/48*65535,
 *                  0x8000 ~= 0 N.m */
uint16_t BA2_EncodePosDes(float q_rad);
uint16_t BA2_EncodeVelDes(float velocity_rad_s);
float    BA2_DecodePosAct(uint16_t raw);
float    BA2_DecodeVelAct(uint16_t raw);
float    BA2_DecodeTauAct(uint16_t raw);
/* Chieu nguoc cua BA2_DecodePosAct() - KHONG co trong ban goc oneLeg (noi do
 * dung bang raw co dinh tinh san cho 1 chan cu the), can them de tinh
 * SETUP_LIMITS tu HOME do duoc luc chay (hieu chuan thich ung, xem
 * motor_calib.c) - cung thang p_act=32.768 da ghi trong comment o tren,
 * chi la chieu ma hoa thay vi giai ma. */
uint16_t BA2_EncodePosAct(float q_rad);
uint16_t BA2_EncodeGainX100(float gain);

/* Kep cung trong [-tau_abs_limit_nm, +tau_abs_limit_nm] TRUOC khi ma hoa -
 * day la chot chan cuoi cung giua moi loi tinh toan phia tren va cuon day
 * dong co. Goi rieng (khong hard-code hang so) de moi noi goi tu quyet dinh
 * tran phu hop boi canh (vi du: chan dang treo khong tai can tran thap hon
 * luc da xac nhan co tai that). */
uint16_t BA2_EncodeTauFf(float tau_nm, float tau_abs_limit_nm);

/* ===== Dung khung CAN-FD 12 byte ===== */
CAN_Frame BA2_BuildSystemFrame(uint32_t id, uint8_t opcode);
CAN_Frame BA2_BuildHandshakeFrame(uint32_t id);

/* CALIB_ENCODER (0xF4) - can lai diem-khong tuyet doi cua encoder trong bo
 * nho driver (khac voi hieu chuan thich ung dang dung o actuator_if.c: cai
 * nay GHI VINH VIEN vao driver, con hieu chuan thich ung chi TINH TRONG RAM
 * cua VDK moi lan boot). offset_raw la gia tri THO 16-bit ghi thang vao
 * Byte1-2 - tai lieu chinh thuc chi cho 1 vi du (0x044C) ma KHONG neu cong
 * thuc suy offset nay tu vi tri vat ly nao, nen CHUA THE tinh dung offset
 * can dung cho robot cu the cua du an nay. Ham nay CHI la lop ma hoa co san -
 * CHUA duoc goi o dau ca (khong nam trong chuoi Actuator_Init()) cho toi khi
 * xac dinh duoc ro rang tren phan cung that (do that + doi chieu phan hoi
 * xac nhan Byte2-3, xem tai lieu chinh thuc muc 2.4 va 7.1: phai can chinh
 * luc robot nam phang). KHONG tu y goi ham nay len robot that. */
CAN_Frame BA2_BuildCalibEncoderFrame(uint32_t id, uint16_t offset_raw);

/* max_raw/min_raw: gioi han hanh trinh da ma hoa theo thang p_act (HR=32.768),
 * vmax_raw: toc do toi da ma hoa - ca ba lay tu bang hieu chuan cua tung khop
 * (xem motor_calib.h), KHONG hard-code trong tang giao thuc nay. */
CAN_Frame BA2_BuildSetupLimitsFrame(uint32_t id, uint16_t max_raw,
                                    uint16_t min_raw, uint16_t vmax_raw);

/* Khung PD day du. velocity_rad_s duoc ma hoa offset-binary tren dai
 * [-45,+45] rad/s; 0 rad/s = 0x8000. Caller van phai kep theo gioi han van
 * hanh cua tung khop. tau_nm duoc kep boi tau_abs_limit_nm khi ma hoa. */
CAN_Frame BA2_BuildPdFrame(uint32_t id, float pos_rad, float velocity_rad_s,
                           float kp, float kd,
                           float tau_nm, float tau_abs_limit_nm);

#endif /* BABY_ALPHA2_PROTOCOL_H */
