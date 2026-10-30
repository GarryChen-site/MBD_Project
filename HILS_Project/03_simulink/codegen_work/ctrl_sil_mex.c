#include "mex.h"
#include "app_wrapper.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs < 3) mexErrMsgTxt("Usage: [duty,req,fb,mode,s1,p2,e,pcmd] = ctrl_sil_mex(x, y, w)");
    size_t N = mxGetNumberOfElements(prhs[0]);
    const double *x = mxGetPr(prhs[0]);
    const double *y = mxGetPr(prhs[1]);
    const double *w = mxGetPr(prhs[2]);
    for (int i = 0; i < 8; i++) plhs[i] = mxCreateDoubleMatrix(N, 1, mxREAL);
    double *duty = mxGetPr(plhs[0]);
    double *req  = mxGetPr(plhs[1]);
    double *fb   = mxGetPr(plhs[2]);
    double *mode = mxGetPr(plhs[3]);
    double *s1   = mxGetPr(plhs[4]);
    double *p2   = mxGetPr(plhs[5]);
    double *e    = mxGetPr(plhs[6]);
    double *pcmd = mxGetPr(plhs[7]);
    App_Init();
    for (size_t k = 0; k < N; k++) {
        Rte_AppIn_t  in  = { (float)x[k], (float)y[k], (float)w[k] };
        Rte_AppOut_t out = { 0 };
        App_Step10ms(&in, &out);
        duty[k] = (double)out.duty_cmd_pct;
        req[k]  = (double)out.speed_req_rpm;
        fb[k]   = (double)out.fault_bitmap;
        mode[k] = (double)out.mode;
        s1[k]   = (double)out.s1_pct;
        p2[k]   = (double)out.p2_pct;
        e[k]    = (double)out.e_pct;
        pcmd[k] = (double)out.p_cmd_pct;
    }
}
