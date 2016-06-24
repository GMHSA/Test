#ifndef _Kalman_h
#define _Kalman_h

double getAngle(double newAngle, double newRate, double dt);
void setAngle(double newAngle);
double getRate(void);
void setQangle(double newQ_angle);
void setQbias(double newQ_bias);
void setRmeasure(double newR_measure);
double getQangle(void);
double getQbias(void);
double getRmeasure(void);

#endif
