/**
 ******************************************************************************
 * @file    motor_calib.h
 * @brief   Tham so hieu chuan + do cung PD cho ca 12 khop, dung chung boi
 *          actuator_if.c. Ke thua tu 2 nguon: du an oneLeg
 *          (/home/dvt/OUT_SAVE/babyDog_test/oneLeg, da chay that tren phan cung) va
 *          tai lieu chinh thuc BabyAlpha2
 *          (github.com/DungTranBK/BabyAlpha2_Docs/can_fd_developer_integration_guide.md).
 *
 *          KIEN TRUC "khong gian LOGIC vs khong gian RAW":
 *          - Khong gian LOGIC: gia tri goc khop nhu controller
 *            (ROS2) hieu - CUNG 1 con so cho moi 4 chan (vd stand_pos giong
 *            het nhau ca 4 chan trong controllers.yaml) vi URDF/xacro da tu
 *            quy uoc dau truc theo tung chan roi.
 *          - Khong gian RAW: gia tri gui thang xuong dong co BabyAlpha2 that
 *            - motor lap GUONG giua cac chan nen CUNG 1 chuyen dong vat ly
 *            can dau lenh RAW khac nhau tuy chan (da do that qua test dung
 *            len tren oneLeg, xem MOTOR_JOINT_SIGN duoi day).
 *          Actuator_SetTarget()/telemetry decode (actuator_if.c) la noi DUY
 *          NHAT chuyen doi giua 2 khong gian nay (nhan logic, gui raw; nhan
 *          raw, tra logic) - MOI tinh toan khac (gioi han, HOME) deu lam
 *          trong khong gian LOGIC, khong can mirror/dao dau rieng le nao nua.
 *
 *          QUAN TRONG - doc truoc khi tin dung:
 *          Bang gioi han + MOTOR_JOINT_SIGN la SO DO THAT tren phan cung CU
 *          THE cua nguoi dung (khong phai suy doan) nhung van CAN xac nhan
 *          lai bang do that/treo chan khong tai truoc khi tin dung dieu
 *          khien co luc that - xem plan Verification.
 ******************************************************************************
 */
#ifndef MOTOR_CALIB_H
#define MOTOR_CALIB_H

#include <stdint.h>
#include "motor_topology.h"

/* Toc do toi da ma hoa (dung chung 12 khop) - tu oneLeg/leg1_config.h
 * (LEG1_VMAX_RAW) VA tai lieu chinh thuc BabyAlpha2 (muc 3.2, vi du
 * 32.94 rad/s -> 0x80AC) - ca 2 nguon khop nhau. */
#define MOTOR_VMAX_RAW   0x80ACU

/* Tran v_des LOP 2 theo tung khop, doc lap voi velocity_max_rad_s tren EC.
 * Giao thuc cho phep +/-45 rad/s va SETUP_LIMITS dang la 32.94 rad/s, nhung
 * Stand/Sit hien tai chi can toc do thap hon nhieu. Khoi dau bao thu 5 rad/s;
 * tach mang 12 phan tu de co the ha rieng abad/hip/knee sau khi log robot that. */
extern const float MOTOR_VELOCITY_ABS_LIMIT_RAD_S[JOINT_COUNT];

/* Mo-men feedforward: duong truyen 12-khop da co; ROS2 tinh Tff luc Stand va
 * scale robot that duoc tune tren EC trong controllers_real.yaml sau khi baseline
 * Tff=0 xac nhan dau -J^T*F. Day la tran firmware doc lap,
 * bat buoc kep lai moi lenh truoc CAN-FD. Giao thuc ma hoa duoc +/-24 N.m;
 * tran du an hien tai dang tune la +/-10 N.m. */
#define MOTOR_TAU_ABS_LIMIT_NM   10.0f

/* Dai hop le Kp/Kd cho tung khop da xac nhan LAI voi
 * hang (2026-08-14): Kp toi da = 60.0 (tai lieu ghi 5.0 la loi danh may), Kd=[0,40.0]
 * N.m(.s)/rad. GIA TRI KHOI DIEM o day van thap (an toan luc chua kiem chung) -
 * TANG DAN theo Verification cua plan (treo chan khong tai, quan sat tung khop)
 * truoc khi tin dung, du tran hop le thuc te cao hon nhieu. */
#define MOTOR_MOVE_KP   1.5f
#define MOTOR_MOVE_KD   3.0f

/* Tran KP - da hoi lai nguoi viet tai lieu cua hang nhieu lan (2026-08-14): tai lieu
 * goc ghi 5.0 (loi danh may theo nguoi dung), lan hoi dau tra loi 60.0, lan hoi lai
 * tra loi 100.0 - dung con so xac nhan GAN NHAT (100.0). Neu sau nay lai doi them,
 * can xac minh chac chan truoc khi sua tiep (da doi 3 lan lien tiep trong 1 phien).
 * KD GIU NGUYEN tran 40.0 (chua co thong tin dinh chinh nao cho Kd, va kd dang dung
 * (1.5) chua bao gio cham tran nay). */
#define MOTOR_KP_ABS_LIMIT   100.0f
#define MOTOR_KD_ABS_LIMIT   40.0f

/* Bang gioi han hanh trinh TUONG DOI (do, khong gian LOGIC, tinh tu HOME do
 * duoc luc khoi dong - xem Actuator_Init()) - chi so theo JointType_t
 * (1=Hang,2=Dui,3=Goi), phan tu [0] khong dung. Nguoi dung xac nhan truc
 * tiep (khong phai suy tu tai lieu): HOME = robot nam xap (tu the nghi tu
 * nhien) luc cap nguon, cung 1 bo gioi han logic nay dung cho CA 4 chan
 * (khong can mirror rieng o day - xem MOTOR_JOINT_SIGN, mirror da chuyen
 * het sang lop dau lenh RAW). */
extern const float MOTOR_LIMIT_MAX_REL_DEG[4];
extern const float MOTOR_LIMIT_MIN_REL_DEG[4];

/* Dau RAW tuyet doi cho tung chan/loai khop - do that qua test dung robot
 * len (oneLeg/HW.md muc 6, bang "Vector don vi"). +1 = lenh logic duong ->
 * gui thang raw duong (khong doi dau). -1 = phai DAO DAU truoc khi ma hoa
 * gui xuong dong co (va dao dau NGUOC LAI khi giai ma telemetry ve logic) -
 * dong co lap GUONG o chan do nen cung 1 chuyen dong vat ly can lenh RAW
 * nguoc dau. Chi so [LegGroup_t][JointType_t], JointType_t 1-indexed (index
 * 0 khong dung). */
extern const int MOTOR_JOINT_SIGN[4][4];

#endif /* MOTOR_CALIB_H */
