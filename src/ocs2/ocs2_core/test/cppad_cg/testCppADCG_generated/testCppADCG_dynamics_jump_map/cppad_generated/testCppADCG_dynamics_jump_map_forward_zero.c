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

void testCppADCG_dynamics_jump_map_forward_zero(double const *const * in,
                                                double*const * out,
                                                struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables

   y[0] = 0.780840602601338 * x[2] + 0.601549194940156 * x[1] + 0.494461128252773 * x[3] + 0.635494158433515 * x[4];
   y[1] = -0.375018750957688 * x[2] + -0.176897642750711 * x[1] + -0.300875082286482 * x[3] + -0.212143787747316 * x[4];
   y[2] = -0.92896140829146 * x[2] + 0.198581372014518 * x[1] + 0.461353080096353 * x[3] + 0.38497538370312 * x[4];
   y[3] = -0.68489064634074 * x[2] + -0.103355412885247 * x[1] + 0.655229200448482 * x[3] + -0.709253800897512 * x[4];
}

