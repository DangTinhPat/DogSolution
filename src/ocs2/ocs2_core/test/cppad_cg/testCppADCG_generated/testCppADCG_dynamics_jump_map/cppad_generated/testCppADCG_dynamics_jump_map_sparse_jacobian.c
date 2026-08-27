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

void testCppADCG_dynamics_jump_map_sparse_jacobian(double const *const * in,
                                                   double*const * out,
                                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   // dependent variables without operations
   jac[0] = 0.601549194940156;
   jac[1] = 0.780840602601338;
   jac[2] = 0.494461128252773;
   jac[3] = 0.635494158433515;
   jac[4] = -0.176897642750711;
   jac[5] = -0.375018750957688;
   jac[6] = -0.300875082286482;
   jac[7] = -0.212143787747316;
   jac[8] = 0.198581372014518;
   jac[9] = -0.92896140829146;
   jac[10] = 0.461353080096353;
   jac[11] = 0.38497538370312;
   jac[12] = -0.103355412885247;
   jac[13] = -0.68489064634074;
   jac[14] = 0.655229200448482;
   jac[15] = -0.709253800897512;
}

