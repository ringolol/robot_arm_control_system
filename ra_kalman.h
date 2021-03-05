#ifndef RA_KALMAN_H
#define RA_KALMAN_H

#include "ra_matrix.h"

class RaKalman {
private:
    RaMatrix F, B, Q, R, X, P, K;
    
public:
    RaKalman(RaMatrix F_, RaMatrix B_, RaMatrix Q_, RaMatrix R_, RaMatrix X0, RaMatrix P0) {
        F = F_;
        B = B_;
        Q = Q_;
        R = R_;
        X = X0;
        P = P0;
        K = RaMatrix(F.row_n, F.col_n);
    }

    void pred_upd(RaMatrix Z, float u) {
        // predict
        X = F * X + B * u;
        P = F * P * F.tr() + Q;
        // update
        K = P * (P + R).inv_diag();
        X = X + K * (Z - X);
        P = P - K * P;
    }

    float getSpeed() {
        return X.data[1][0];
    }

    float getPos() {
        return X.data[0][0];
    }
};

#endif