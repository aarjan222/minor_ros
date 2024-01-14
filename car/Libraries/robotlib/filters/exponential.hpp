#ifndef EXPONENTIAL_H_
#define EXPONENTIAL_H_

template <typename T> class ExponentialFilter {
public:
  T alpha, x_last;
  bool first_sample = true;
  ExponentialFilter(){}
  ExponentialFilter(T _alpha) : alpha(_alpha) {}
  T operator()(const T x) {
    if (first_sample) {
      x_last = x;
      first_sample = false;
    } else
      x_last = alpha * x + (1 - alpha) * x_last;
    return x_last;
  }
};

#endif // EXPONENTIAL_H_
