#ifndef _LPF_H
#define _LPF_H

class lowpassFilter
{
  public:
    lowpassFilter(double w_c, double T);
    lowpassFilter()
    {
    }

    double w_c_;  // bandwidth frequency
    double dT;    // sampling period

    double u_k_1;
    double y_k_1;
    double y_k;
    void init(double w_c, double T);
    double update(double u_k);
    void reset();
};

#endif