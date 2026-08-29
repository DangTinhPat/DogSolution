#include "baby_alpha2_protocol.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static void expect_near(float actual, float expected, float tolerance, const char *name)
{
    if (fabsf(actual - expected) > tolerance)
    {
        fprintf(stderr, "%s: got %.7f, expected %.7f\n", name, actual, expected);
        exit(EXIT_FAILURE);
    }
}

static void expect_u16(uint16_t actual, uint16_t expected, const char *name)
{
    if (actual != expected)
    {
        fprintf(stderr, "%s: got 0x%04x, expected 0x%04x\n",
                name, (unsigned int)actual, (unsigned int)expected);
        exit(EXIT_FAILURE);
    }
}

int main(void)
{
    expect_near(BA2_DecodePosAct(0x8000U), 0.0f, 0.00001f, "position zero");
    expect_near(BA2_DecodeVelAct(0x8000U), 0.0f, 0.00001f, "velocity zero");
    expect_near(BA2_DecodeVelAct(0xFFFFU), 44.9993f, 0.001f, "velocity max");
    expect_near(BA2_DecodeTauAct(0x8000U), 0.0f, 0.00001f, "torque zero");
    expect_near(BA2_DecodeTauAct(0x0000U), -24.0004f, 0.001f, "torque min");
    expect_u16(BA2_EncodeVelDes(0.0f), 0x8000U, "encode velocity zero");
    expect_u16(BA2_EncodeTauFf(0.0f, 10.0f), 0x8000U, "encode torque zero");

    const CAN_Frame setup = BA2_BuildSetupLimitsFrame(3U, 0x91B3U, 0x8014U, 0x80ACU);
    expect_u16((uint16_t)(((uint16_t)setup.data[1] << 8) | setup.data[2]),
               0x91B3U, "setup pmax");
    expect_u16((uint16_t)(((uint16_t)setup.data[5] << 8) | setup.data[6]),
               0x80ACU, "setup vmax");
    if (!setup.fd_format || setup.data_len != 12U || setup.data[11] != BA2_OPCODE_SETUP_LIMITS)
    {
        fprintf(stderr, "setup frame metadata mismatch\n");
        return EXIT_FAILURE;
    }

    puts("BabyAlpha2 protocol tests passed");
    return EXIT_SUCCESS;
}
