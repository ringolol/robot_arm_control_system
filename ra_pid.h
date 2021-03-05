#ifndef RA_PID_H
#define RA_PID_H

#include <Arduino.h>

class RaPID {
private:
    float P=1.f, I=.0f, D=.0f;
    float sI=.0f;
    float error=.0f;
    float *Dbuff;
    short N, dinx=0;
    unsigned long tnew;

    float getDiff(float diff_new) {
        if(!N) return .0f;
        if(dinx >= N) dinx = 0;
        Dbuff[dinx] = diff_new;

        float sum = 0.f;
        for(short i=0; i<N; i++) {
            sum += Dbuff[i];
        }
        dinx++;
        return sum / N;
    }
    
public:
    RaPID(float P_, float I_, float D_, float sI0=.0f, short N_=1) {
        set_pars(P_, I_, D_, sI0, N_);
    }

    RaPID() {
        P = 1.f;
        I = 0.f;
        D = 0.f;
        sI = 0.f;
        N = 0;
        Dbuff = new float[N];
        reset();
    }

    void set_pars(float P_, float I_, float D_, float sI0=.0f, short N_=1) {
        P = P_;
        I = I_;
        D = D_;
        sI = sI0;
        N = N_;
        delete[] Dbuff;
        Dbuff = new float[N];
        reset();
    }

    float calc(float ref, float mes) {
        float error_old = error;
        error = ref - mes;

        unsigned long told = tnew;
        tnew = millis();

        float dt = (tnew - told) / 1000.f;
        
        sI += error * dt;
        float de = (error - error_old) / dt;
        float diff = getDiff(de);

        return P*error + I*sI + D*diff;
    }

    void reset() {
        sI = 0.f;
        for(short i=0; i<N; i++) {
            Dbuff[i] = 0;
        }
        tnew = millis() - 15;
    }
};

#endif