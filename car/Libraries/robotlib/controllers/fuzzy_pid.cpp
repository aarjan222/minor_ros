#include "fuzzy_pid.hpp"

// clang-format off

// set of output rules defined based upon the relation between input
// (error & d_error) and output parameters(kp,ki,kd)
int kp_rules[no_of_fuzzy_sets][no_of_fuzzy_sets] = {
  {HB, HB, HM, HM, HS, ZO, ZO},
  {HB, HB, HM, HS, HS, ZO, LS},
  {HM, HM, HM, HS, ZO, LS, LS},
  {HM, HM, HS, ZO, LS, LM, LM},
  {HS, HS, ZO, LS, LS, LM, LM},
  {HS, ZO, LS, LM, LM, LM, LB},
  {ZO, ZO, LM, LM, LM, LB, LB}};

int ki_rules[no_of_fuzzy_sets][no_of_fuzzy_sets] = {
  {LB, LB, LM, LM, LS, ZO, ZO},
  {LB, LB, LM, LS, LS, ZO, ZO},
  {LB, LM, LS, LS, ZO, HS, HS},
  {LM, LM, LS, ZO, HS, HM, HM},
  {LM, LS, ZO, HS, HS, HM, HB},
  {ZO, ZO, HS, HS, HM, HB, HB},
  {ZO, ZO, HS, HM, HM, HB, HB}};

int kd_rules[no_of_fuzzy_sets][no_of_fuzzy_sets] = {
  {HS, LS, LB, LB, LB, LM, HS},
  {HS, LS, LB, LM, LM, LS, ZO},
  {ZO, LS, LM, LM, LS, LS, ZO},
  {ZO, LS, LS, LS, LS, LS, ZO},
  {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
  {HB, LS, HS, HS, HS, HS, HB},
  {HB, HM, HM, HM, HS, HS, HB}};

// set of membership function used to define each input and output
// parameters,with in range -3 to 3
//  where each 3 consecutive term represents the triangular membershipfunction
//  whch is used to find the equation of triangular geometry


//fuzzification tringular membership functions for error and change in error already defined by researchers
float e_mf[no_of_fuzzy_sets * 3] =
  {-3, -3, -2,
   -3, -2,  0,
   -3, -1,  1,
   -2,  0,  2,
   -1,  1,  3,
    0,  2,  3,
    2,  3,  3};

float de_mf[no_of_fuzzy_sets * 3] =
  {-3, -3, -2,
   -3, -2,  0,
   -3, -1,  1,
   -2,  0,  2,
   -1,  1,  3,
    0,  2,  3,
    2,  3,  3};


//defuzzification  tringular membership functions
float Kp_mf[no_of_fuzzy_sets * 3] =
  {-0.03, -0.03, -0.02,
   -0.03, -0.02,  0,
   -0.03, -0.01,  0.01,
   -0.02,  0,     0.02,
   -0.01,  0.01,  0.03,
    0,     0.02,  0.03,
    0.02,  0.03,  0.03};

float Ki_mf[no_of_fuzzy_sets * 3] =
  {-0.03, -0.03, -0.02,
   -0.03, -0.02,  0,
   -0.03, -0.01,  0.01,
   -0.02,  0,     0.02,
   -0.01,  0.01,  0.03,
    0,     0.02,  0.03,
    0.02,  0.03,  0.03};

float Kd_mf[no_of_fuzzy_sets * 3] =
  {-0.03, -0.03, -0.02,
   -0.03, -0.02,  0,
   -0.03, -0.01,  0.01,
   -0.02,  0,     0.02,
   -0.01,  0.01,  0.03,
    0,     0.02,  0.03,
    0.02,  0.03,  0.03};

// clang-format on


/*calculated error and de_error lai tringular membership function ko harek side ma 
  linearly map garera corresponding y-value return garxa. if side does not contain the 
  data(e, de_e sets) then returns zero. */
float fuzzy_pid::trimf(float x, float a, float b, float c)
{
  float out;
  if (x > a && x <= b)
    out = (x - a) / (b - a);
  else if (x > b && x <= c)
    out = (c - x) / (c - b);
  else
    out = 0;
  return out;
}


/* again apply unitary method (or linearize the y axis data to get average of x-values)

        /\
--------------------------
      /|  |\
     / |  | \
        -.-

*/
float fuzzy_pid::invTrimf(float x, float a, float b, float c)
{
  float out1, out2;
  out1 = (b - a) * x + a;
  out2 = c - (c - b) * x;
  return (out1 + out2) / 2;
}


//initialize the data members
fuzzy_pid::fuzzy_pid()
{
  this->outMax = 0;
  this->outMin = 0;
  this->output = 0;
  this->error = 0;
  this->d_error = 0;
  this->error_l = 0;
  this->error_ll = 0;
  this->kp = 0;
  this->ki = 0;
  this->kd = 0;
  this->maxValue = 0;
}

void fuzzy_pid::set_parameter(float kp, float ki, float kd, float outMin,
                              float outMax)
{
  this->outMax = outMax;
  this->outMin = outMin;
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  calculate_del_output(this->kp, this->ki, this->kd);
}

// based on the descretization of pid algorithm keeping sampling timeperiod to
// be constant
void fuzzy_pid::calculate_del_output(float kp_, float ki_, float kd_)
{

  A = kp_ + ki_ + kd_;
  B = kp_ - ki_ - 2 * kd_;
  C = kd_;
  del_out = A * error + B * error_l + C * error_ll;
  output += del_out;
}

float fuzzy_pid::calculate_del_tuningGain(
    int rules[no_of_fuzzy_sets][no_of_fuzzy_sets],
    float mf[no_of_fuzzy_sets * 7])
{

  //e_set ko ith and de_sets ko jth item bata min value xanera i*j ko matrix banaune
  for (int i = 0; i < 7; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      maxIndex[i][j] = e_sets[i] > de_sets[j] ? de_sets[j] : e_sets[i];
    }
  }

  //  min value ko matrix bata maximum value ra tesko position note garne
  for (int i = 0; i < 7; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      if (maxIndex[i][j] > maxValue)
      {
        maxValue = maxIndex[i][j];
        maxRow = i;
        maxColumn = j;
      }
    }
  }

  //then apply inverse tringular member function 
  float k = invTrimf(maxValue, mf[rules[maxRow][maxColumn] * 3],
                     mf[rules[maxRow][maxColumn] * 3 + 1],
                     mf[rules[maxRow][maxColumn] * 3 + 2]);
  return k;
}

float fuzzy_pid::map(float x, float input, float output)
{
  return (x * output) / input;
}

float fuzzy_pid::clamp(float x, float outmin, float outmax)
{
  if (x > outmax)
    return outmax;
  else if (x < outmin)
    return outmin;
  else
    return x;
}

float fuzzy_pid::compute_fuzzy_selfTuning_PID(float setpoint, float input)
{
  setpoint = map(setpoint, 50, 1);
  input = map(input, 50, 1);
  error = setpoint - input;
  d_error = (error - error_l);

  //generate sets of error and d_error using membership functions defined above
  //(error in terms of membership function(lineraly mapped))
  
  for (int i = 0; i < no_of_fuzzy_sets; i++)
  {
    e_sets[i] = trimf(error, e_mf[i * 3], e_mf[i * 3 + 1], e_mf[i * 3 + 2]);
    de_sets[i] =
        trimf(d_error, de_mf[i * 3], de_mf[i * 3 + 1], de_mf[i * 3 + 2]);
  }

  kp += calculate_del_tuningGain(kp_rules, Kp_mf);
  if (kp < 0)
    kp = 0;
  ki += calculate_del_tuningGain(ki_rules, Ki_mf);
  if (ki < 0)
    ki = 0;
  kd += calculate_del_tuningGain(kd_rules, Kd_mf);
  if (kd < 0)
    kd = 0;
    
  calculate_del_output(this->kp, this->ki, 0);
  error_ll = error_l;
  error_l = error;
  output = clamp(output, outMin, outMax);
  return output;
}

void fuzzy_pid::get_parameter(float k[3])
{
  k[0] = kp;
  k[1] = ki;
  k[2] = kd;
}
