#include "motor_calib.h"

const float MOTOR_VELOCITY_ABS_LIMIT_RAD_S[JOINT_COUNT] = {
    5.0f, 5.0f, 5.0f,
    5.0f, 5.0f, 5.0f,
    5.0f, 5.0f, 5.0f,
    5.0f, 5.0f, 5.0f
};

/* [0] khong dung (JointType_t bat dau tu 1). Nguoi dung xac nhan truc tiep
 * (khong gian LOGIC, tuong doi so HOME = tu the nam xap luc cap nguon).
 *
 * DA DAO DAU (2026-08-13) so voi ban goc ke thua tu oneLeg ({45,-85}/{110,-230}/
 * {130,0}) - ban goc do bang quy uoc dau RIENG cua oneLeg (du an khac, khong
 * dung chung "khong gian LOGIC" voi babyDog). Sau khi babyDog hieu chinh lai
 * origin/limit trong babydog.xacro (xem NOTE.md muc 2.6+2.8), quy uoc dau
 * LOGIC chuan cua du an nay la NGUOC LAI ban oneLeg cho ca 3 loai khop - xac
 * nhan bang cach doi chieu voi <limit> that trong babydog.xacro (rad->do):
 *   Hang  URDF=[-44.4,+85.6]  ~  dao dau ban oneLeg [-45,+85]  (khop)
 *   Dui   URDF=[-109,+231]    ~  dao dau ban oneLeg [-110,+230] (khop)
 *   Goi   URDF=[-130,0]       ~  dao dau ban oneLeg [-130,0]   (khop chinh xac)
 * Truoc khi sua, bang nay khien firmware kep target Stand/Sit theo huong
 * NGUOC voi URDF/RViz - khop quay dung bien do nhung sai chieu vat ly khi
 * dung/ngoi that (khong lien quan MOTOR_JOINT_SIGN, cai do van dung - day la
 * loi khac, o tang gioi han). */
const float MOTOR_LIMIT_MAX_REL_DEG[4] = { 0.0f, 85.0f, 230.0f, 0.0f };
const float MOTOR_LIMIT_MIN_REL_DEG[4] = { 0.0f, -45.0f, -110.0f, -130.0f };

/* Vector dau tuyet doi - oneLeg/HW.md muc 6, do that qua test dung robot
 * len:
 *   FL: Hang=-1, Dui=-1, Goi=+1
 *   FR: Hang=+1, Dui=+1, Goi=-1
 *   RB: Hang=+1, Dui=-1, Goi=+1
 *   LB: Hang=-1, Dui=+1, Goi=-1
 * Anh xa "FL/FR/RB/LB" (ten oneLeg) -> LegGroup_t (ten babyDog): FR=FRONT_
 * RIGHT, FL=FRONT_LEFT, RB=HIND_RIGHT, LB=HIND_LEFT (gia dinh "RB"="hind
 * right", "LB"="hind left" - CAN XAC NHAN LAI bang do tay/test co lap that
 * neu nghi ngo, khong suy duoc tu ten goi don thuan). [x][0] khong dung.
 * Bang ben duoi da duoc SAP LAI theo thu tu JointIndex_t/ROS cua megaDog:
 * LF, LH, RF, RH. */
/* Da dao dau lai theo phan hoi thuc te tren robot that (nguoi dung xoay tay
 * tung khop, doi chieu voi RViz - lay RViz/URDF lam chuan): chan 1
 * (FRONT_RIGHT) Hang+Dui nguoc; chan 2 (FRONT_LEFT) Dui nguoc; chan 3
 * (HIND_RIGHT) Goi nguoc; chan 4 (HIND_LEFT) Hang+Goi nguoc - da dao dau
 * dung 6 o do so voi ban goc (oneLeg) ben duoi. */
const int MOTOR_JOINT_SIGN[4][4] = {
    /*                [0]  Hang Dui  Goi */
    /* LF */          { 0,  -1,  +1,  +1 },
    /* LH */          { 0,  +1,  +1,  +1 },
    /* RF */          { 0,  -1,  -1,  -1 },
    /* RH */          { 0,  +1,  -1,  -1 },
};
