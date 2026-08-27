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

void testCppADCG_dynamics_flow_map_sparse_jacobian(double const *const * in,
                                                   double*const * out,
                                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   // dependent variables without operations
   jac[0] = -0.465731248010756;
   jac[1] = -0.839231927804291;
   jac[2] = -0.344747958399704;
   jac[3] = -0.74451459839219;
   jac[4] = -0.83484605319558;
   jac[5] = -0.487022963113628;
   jac[6] = -0.130520120789539;
   jac[7] = -0.0636307592800031;
   jac[8] = 0.971892645569468;
   jac[9] = -0.79775256654143;
   jac[10] = -0.715421306768163;
   jac[11] = 0.538242727303059;
   jac[12] = -0.371913673995023;
   jac[13] = 0.326503132156331;
   jac[14] = -0.507048685805429;
   jac[15] = 0.169995757364666;
   jac[16] = 0.538148971990752;
   jac[17] = -0.711064073122602;
   jac[18] = 0.146243726437093;
   jac[19] = 0.729746183254638;
   jac[20] = -0.610104990010199;
   jac[21] = -0.879082248489876;
   jac[22] = 0.979082625349556;
   jac[23] = 0.128503481451656;
}

