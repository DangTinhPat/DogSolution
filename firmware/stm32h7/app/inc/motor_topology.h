/**
 ******************************************************************************
 * @file    motor_topology.h
 * @brief   Dinh danh 12 khop + tra cuu bus/ID/loai-khop/nhom-chan cho giao
 *          thuc BabyAlpha2 (CAN-FD, xem baby_alpha2_protocol.h) giua STM32H7
 *          (main board) va 12 board driver dong co that o moi khop. Ke thua
 *          topology tu du an oneLeg (/home/dvt/OUT_SAVE/babyDog_test/oneLeg,
 *          main_12joint_hold_proven.c.bak) - da chay that tren phan cung.
 *
 *          Topology BUS: 2 bus CAN-FD, moi bus 6 dong co, ID 1-6 TRONG bus
 *          (khong phai ID toan cuc nhu ban cu). CAN_INSTANCE_1 (FDCAN1,
 *          PA11/12) = 2 chan TRUOC (front_right id1-3, front_left id4-6).
 *          CAN_INSTANCE_2 (FDCAN2, PB5/6) = 2 chan SAU (hind_right id1-3,
 *          hind_left id4-6). Trong 1 chan, id 1=Hang(abad, roll), 2=Dui(hip,
 *          pitch), 3=Goi(knee, pitch) - LAP LAI cho chan thu 2 cua bus (id
 *          4=Hang,5=Dui,6=Goi).
 *
 *          QUAN TRONG: JointIndex_t ben duoi dung thu tu ROS/OCS2 cua megaDog
 *          (LF, LH, RF, RH), KHONG dung thu tu vat ly tren bus CAN. Vi vay
 *          Motor_BusForJoint()/Motor_IdForJoint()/Motor_LegGroupForJoint()
 *          phai tra bang ro rang, khong duoc suy bang joint < 6 hay joint/3.
 *
 *          QUAN TRONG: phan hoi (PING/HANDSHAKE/SETUP/telemetry) tu MOI dong
 *          co TRONG 1 bus deu ve CHUNG 1 CAN ID = 0 - phan biet dong co nao
 *          bang byte data[0] (gia tri 1-6 cho telemetry Page0, hoac
 *          0x08/0x18/0x38 + id cho cac phan hoi he thong), KHONG phai bang
 *          CAN ID nhu ban giao thuc tu bia truoc day. Xem actuator_if.c.
 ******************************************************************************
 */
#ifndef MOTOR_TOPOLOGY_H
#define MOTOR_TOPOLOGY_H

#include <stdint.h>
#include "can.h" /* CAN_INSTANCE_1/2 */

/* Thu tu khop 0-11, khop voi controllers_real.yaml, real_hardware.xacro va
 * MegadogController::jointNames(): LF, LH, RF, RH. */
typedef enum
{
    JOINT_LF_ABAD = 0,
    JOINT_LF_HIP = 1,
    JOINT_LF_KNEE = 2,
    JOINT_LH_ABAD = 3,
    JOINT_LH_HIP = 4,
    JOINT_LH_KNEE = 5,
    JOINT_RF_ABAD = 6,
    JOINT_RF_HIP = 7,
    JOINT_RF_KNEE = 8,
    JOINT_RH_ABAD = 9,
    JOINT_RH_HIP = 10,
    JOINT_RH_KNEE = 11,
    JOINT_COUNT = 12
} JointIndex_t;

/* Loai khop (dung tra bang gioi han hanh trinh theo LOAI, KHONG theo ID CAN
 * 1-6 - bay da tung sap: id 4,5,6 doc nham vao mang chi co 4 phan tu neu
 * dung id truc tiep, xem oneLeg/HW.md muc 4). Gia tri 1-indexed (index 0
 * khong dung) de khop dung quy uoc bang gioi han ke thua tu oneLeg. */
typedef enum
{
    JOINT_TYPE_HANG = 1,   /* Hang/abad - roll */
    JOINT_TYPE_DUI  = 2,   /* Dui/hip - pitch */
    JOINT_TYPE_GOI  = 3,   /* Goi/knee - pitch */
} JointType_t;

/* Nhom chan - dung cho logic mirror dau motor (xem motor_calib.h). Thu tu
 * TRUNG KHOP voi JointIndex_t: LF, LH, RF, RH. */
typedef enum
{
    LEG_LF = 0,
    LEG_LH = 1,
    LEG_RF = 2,
    LEG_RH = 3,
} LegGroup_t;

/* Bus CAN vat ly cho khop: LF/RF tren CAN1, LH/RH tren CAN2. */
static inline uint32_t Motor_BusForJoint(JointIndex_t joint)
{
    static const uint32_t kBusByJoint[JOINT_COUNT] = {
        CAN_INSTANCE_1, CAN_INSTANCE_1, CAN_INSTANCE_1, /* LF: front-left  id4-6 */
        CAN_INSTANCE_2, CAN_INSTANCE_2, CAN_INSTANCE_2, /* LH: hind-left   id4-6 */
        CAN_INSTANCE_1, CAN_INSTANCE_1, CAN_INSTANCE_1, /* RF: front-right id1-3 */
        CAN_INSTANCE_2, CAN_INSTANCE_2, CAN_INSTANCE_2, /* RH: hind-right  id1-3 */
    };
    return kBusByJoint[(uint32_t)joint];
}

/* ID dong co TRONG bus (1-6) - dung de dinh dia chi khung CAN gui DI (lenh
 * PD/PING/HANDSHAKE/SETUP_LIMITS). Phan hoi KHONG dung ID nay (xem module
 * header o tren - phan hoi luon ve ID=0, phan biet bang data[0]). */
static inline uint32_t Motor_IdForJoint(JointIndex_t joint)
{
    static const uint32_t kIdByJoint[JOINT_COUNT] = {
        4U, 5U, 6U, /* LF */
        4U, 5U, 6U, /* LH */
        1U, 2U, 3U, /* RF */
        1U, 2U, 3U, /* RH */
    };
    return kIdByJoint[(uint32_t)joint];
}

/* Loai khop (1=Hang,2=Dui,3=Goi) - dung tra bang gioi han hanh trinh theo
 * LOAI (motor_calib.h), KHONG dung Motor_IdForJoint() cho viec nay. */
static inline JointType_t Motor_JointTypeForJoint(JointIndex_t joint)
{
    return (JointType_t)(((uint32_t)joint % 3U) + 1U);
}

/* Nhom chan (0-3, xem LegGroup_t) - dung cho logic mirror gioi han. */
static inline LegGroup_t Motor_LegGroupForJoint(JointIndex_t joint)
{
    static const LegGroup_t kLegByJoint[JOINT_COUNT] = {
        LEG_LF, LEG_LF, LEG_LF,
        LEG_LH, LEG_LH, LEG_LH,
        LEG_RF, LEG_RF, LEG_RF,
        LEG_RH, LEG_RH, LEG_RH,
    };
    return kLegByJoint[(uint32_t)joint];
}

#endif /* MOTOR_TOPOLOGY_H */
