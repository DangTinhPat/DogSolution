#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void testCppADCG_dynamics_flow_map_forward_zero(double const *const * in,
                                                double*const * out,
                                                struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables

   y[0] = -0.839231927804291 * x[2] + -0.465731248010756 * x[1] + -0.344747958399704 * x[3] + -0.74451459839219 * x[4] + -0.83484605319558 * x[5] + -0.487022963113628 * x[6];
   y[1] = -0.0636307592800031 * x[2] + -0.130520120789539 * x[1] + 0.971892645569468 * x[3] + -0.79775256654143 * x[4] + -0.715421306768163 * x[5] + 0.538242727303059 * x[6];
   y[2] = 0.326503132156331 * x[2] + -0.371913673995023 * x[1] + -0.507048685805429 * x[3] + 0.169995757364666 * x[4] + 0.538148971990752 * x[5] + -0.711064073122602 * x[6];
   y[3] = 0.729746183254638 * x[2] + 0.146243726437093 * x[1] + -0.610104990010199 * x[3] + -0.879082248489876 * x[4] + 0.979082625349556 * x[5] + 0.128503481451656 * x[6];
}

