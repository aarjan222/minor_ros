#ifndef COMPLEMENTARY_H_
#define COMPLEMENTARY_H_

template <typename T> class ComplementaryFilter {
public:
  T alpha;
  ComplementaryFilter(T _alpha) : alpha(_alpha) {}
  T operator()(const T x_a, const T x_b) {
    return alpha * x_a + (1 - alpha) * x_b;
  }
};

#endif // COMPLEMENTARY_H_
