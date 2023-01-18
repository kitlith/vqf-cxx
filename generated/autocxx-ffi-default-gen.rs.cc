#include "vqf.hpp"
#include "autocxxgen_ffi.h"
#include <cstddef>

extern "C" {
void cxxbridge1$new_autocxx_autocxx_wrapper(::VQFParams *autocxx_gen_this) noexcept {
  void (*new_autocxx_autocxx_wrapper$)(::VQFParams *) = ::new_autocxx_autocxx_wrapper;
  new_autocxx_autocxx_wrapper$(autocxx_gen_this);
}

void cxxbridge1$VQF$updateGyr(::VQF &self, double const *gyr) noexcept {
  void (::VQF::*updateGyr$)(double const *) = &::VQF::updateGyr;
  (self.*updateGyr$)(gyr);
}

void cxxbridge1$VQF$updateAcc(::VQF &self, double const *acc) noexcept {
  void (::VQF::*updateAcc$)(double const *) = &::VQF::updateAcc;
  (self.*updateAcc$)(acc);
}

void cxxbridge1$VQF$updateMag(::VQF &self, double const *mag) noexcept {
  void (::VQF::*updateMag$)(double const *) = &::VQF::updateMag;
  (self.*updateMag$)(mag);
}

void cxxbridge1$VQF$update(::VQF &self, double const *gyr, double const *acc) noexcept {
  void (::VQF::*update$)(double const *, double const *) = &::VQF::update;
  (self.*update$)(gyr, acc);
}

void cxxbridge1$VQF$update1(::VQF &self, double const *gyr, double const *acc, double const *mag) noexcept {
  void (::VQF::*update1$)(double const *, double const *, double const *) = &::VQF::update;
  (self.*update1$)(gyr, acc, mag);
}

void cxxbridge1$VQF$updateBatch(::VQF &self, double const *gyr, double const *acc, double const *mag, ::std::size_t N, double *out6D, double *out9D, double *outDelta, double *outBias, double *outBiasSigma, bool *outRest, bool *outMagDist) noexcept {
  void (::VQF::*updateBatch$)(double const *, double const *, double const *, ::std::size_t, double *, double *, double *, double *, double *, bool *, bool *) = &::VQF::updateBatch;
  (self.*updateBatch$)(gyr, acc, mag, N, out6D, out9D, outDelta, outBias, outBiasSigma, outRest, outMagDist);
}

void cxxbridge1$VQF$getQuat3D(::VQF const &self, double *out) noexcept {
  void (::VQF::*getQuat3D$)(double *) const = &::VQF::getQuat3D;
  (self.*getQuat3D$)(out);
}

void cxxbridge1$VQF$getQuat6D(::VQF const &self, double *out) noexcept {
  void (::VQF::*getQuat6D$)(double *) const = &::VQF::getQuat6D;
  (self.*getQuat6D$)(out);
}

void cxxbridge1$VQF$getQuat9D(::VQF const &self, double *out) noexcept {
  void (::VQF::*getQuat9D$)(double *) const = &::VQF::getQuat9D;
  (self.*getQuat9D$)(out);
}

double cxxbridge1$VQF$getDelta(::VQF const &self) noexcept {
  double (::VQF::*getDelta$)() const = &::VQF::getDelta;
  return (self.*getDelta$)();
}

double cxxbridge1$VQF$getBiasEstimate(::VQF const &self, double *out) noexcept {
  double (::VQF::*getBiasEstimate$)(double *) const = &::VQF::getBiasEstimate;
  return (self.*getBiasEstimate$)(out);
}

void cxxbridge1$VQF$setBiasEstimate(::VQF &self, double *bias, double sigma) noexcept {
  void (::VQF::*setBiasEstimate$)(double *, double) = &::VQF::setBiasEstimate;
  (self.*setBiasEstimate$)(bias, sigma);
}

bool cxxbridge1$VQF$getRestDetected(::VQF const &self) noexcept {
  bool (::VQF::*getRestDetected$)() const = &::VQF::getRestDetected;
  return (self.*getRestDetected$)();
}

bool cxxbridge1$VQF$getMagDistDetected(::VQF const &self) noexcept {
  bool (::VQF::*getMagDistDetected$)() const = &::VQF::getMagDistDetected;
  return (self.*getMagDistDetected$)();
}

void cxxbridge1$VQF$getRelativeRestDeviations(::VQF const &self, double *out) noexcept {
  void (::VQF::*getRelativeRestDeviations$)(double *) const = &::VQF::getRelativeRestDeviations;
  (self.*getRelativeRestDeviations$)(out);
}

double cxxbridge1$VQF$getMagRefNorm(::VQF const &self) noexcept {
  double (::VQF::*getMagRefNorm$)() const = &::VQF::getMagRefNorm;
  return (self.*getMagRefNorm$)();
}

double cxxbridge1$VQF$getMagRefDip(::VQF const &self) noexcept {
  double (::VQF::*getMagRefDip$)() const = &::VQF::getMagRefDip;
  return (self.*getMagRefDip$)();
}

void cxxbridge1$VQF$setMagRef(::VQF &self, double norm, double dip) noexcept {
  void (::VQF::*setMagRef$)(double, double) = &::VQF::setMagRef;
  (self.*setMagRef$)(norm, dip);
}

void cxxbridge1$VQF$setTauAcc(::VQF &self, double tauAcc) noexcept {
  void (::VQF::*setTauAcc$)(double) = &::VQF::setTauAcc;
  (self.*setTauAcc$)(tauAcc);
}

void cxxbridge1$VQF$setTauMag(::VQF &self, double tauMag) noexcept {
  void (::VQF::*setTauMag$)(double) = &::VQF::setTauMag;
  (self.*setTauMag$)(tauMag);
}

void cxxbridge1$VQF$setMotionBiasEstEnabled(::VQF &self, bool enabled) noexcept {
  void (::VQF::*setMotionBiasEstEnabled$)(bool) = &::VQF::setMotionBiasEstEnabled;
  (self.*setMotionBiasEstEnabled$)(enabled);
}

void cxxbridge1$VQF$setRestBiasEstEnabled(::VQF &self, bool enabled) noexcept {
  void (::VQF::*setRestBiasEstEnabled$)(bool) = &::VQF::setRestBiasEstEnabled;
  (self.*setRestBiasEstEnabled$)(enabled);
}

void cxxbridge1$VQF$setMagDistRejectionEnabled(::VQF &self, bool enabled) noexcept {
  void (::VQF::*setMagDistRejectionEnabled$)(bool) = &::VQF::setMagDistRejectionEnabled;
  (self.*setMagDistRejectionEnabled$)(enabled);
}

void cxxbridge1$VQF$setRestDetectionThresholds(::VQF &self, double thGyr, double thAcc) noexcept {
  void (::VQF::*setRestDetectionThresholds$)(double, double) = &::VQF::setRestDetectionThresholds;
  (self.*setRestDetectionThresholds$)(thGyr, thAcc);
}

::VQFParams const *cxxbridge1$VQF$getParams(::VQF const &self) noexcept {
  ::VQFParams const &(::VQF::*getParams$)() const = &::VQF::getParams;
  return &(self.*getParams$)();
}

::VQFCoefficients const *cxxbridge1$VQF$getCoeffs(::VQF const &self) noexcept {
  ::VQFCoefficients const &(::VQF::*getCoeffs$)() const = &::VQF::getCoeffs;
  return &(self.*getCoeffs$)();
}

::VQFState const *cxxbridge1$VQF$getState(::VQF const &self) noexcept {
  ::VQFState const &(::VQF::*getState$)() const = &::VQF::getState;
  return &(self.*getState$)();
}

void cxxbridge1$VQF$setState(::VQF &self, ::VQFState const &state) noexcept {
  void (::VQF::*setState$)(::VQFState const &) = &::VQF::setState;
  (self.*setState$)(state);
}

void cxxbridge1$VQF$resetState(::VQF &self) noexcept {
  void (::VQF::*resetState$)() = &::VQF::resetState;
  (self.*resetState$)();
}

void cxxbridge1$quatMultiply_autocxx_wrapper(double const *q1, double const *q2, double *out) noexcept {
  void (*quatMultiply_autocxx_wrapper$)(double const *, double const *, double *) = ::quatMultiply_autocxx_wrapper;
  quatMultiply_autocxx_wrapper$(q1, q2, out);
}

void cxxbridge1$quatConj_autocxx_wrapper(double const *q, double *out) noexcept {
  void (*quatConj_autocxx_wrapper$)(double const *, double *) = ::quatConj_autocxx_wrapper;
  quatConj_autocxx_wrapper$(q, out);
}

void cxxbridge1$quatSetToIdentity_autocxx_wrapper(double *out) noexcept {
  void (*quatSetToIdentity_autocxx_wrapper$)(double *) = ::quatSetToIdentity_autocxx_wrapper;
  quatSetToIdentity_autocxx_wrapper$(out);
}

void cxxbridge1$quatApplyDelta_autocxx_wrapper(double *q, double delta, double *out) noexcept {
  void (*quatApplyDelta_autocxx_wrapper$)(double *, double, double *) = ::quatApplyDelta_autocxx_wrapper;
  quatApplyDelta_autocxx_wrapper$(q, delta, out);
}

void cxxbridge1$quatRotate_autocxx_wrapper(double const *q, double const *v, double *out) noexcept {
  void (*quatRotate_autocxx_wrapper$)(double const *, double const *, double *) = ::quatRotate_autocxx_wrapper;
  quatRotate_autocxx_wrapper$(q, v, out);
}

double cxxbridge1$norm_autocxx_wrapper(double const *vec, ::std::size_t N) noexcept {
  double (*norm_autocxx_wrapper$)(double const *, ::std::size_t) = ::norm_autocxx_wrapper;
  return norm_autocxx_wrapper$(vec, N);
}

void cxxbridge1$normalize_autocxx_wrapper(double *vec, ::std::size_t N) noexcept {
  void (*normalize_autocxx_wrapper$)(double *, ::std::size_t) = ::normalize_autocxx_wrapper;
  normalize_autocxx_wrapper$(vec, N);
}

void cxxbridge1$clip_autocxx_wrapper(double *vec, ::std::size_t N, double min, double max) noexcept {
  void (*clip_autocxx_wrapper$)(double *, ::std::size_t, double, double) = ::clip_autocxx_wrapper;
  clip_autocxx_wrapper$(vec, N, min, max);
}

double cxxbridge1$gainFromTau_autocxx_wrapper(double tau, double Ts) noexcept {
  double (*gainFromTau_autocxx_wrapper$)(double, double) = ::gainFromTau_autocxx_wrapper;
  return gainFromTau_autocxx_wrapper$(tau, Ts);
}

void cxxbridge1$filterCoeffs_autocxx_wrapper(double tau, double Ts, double *outB, double *outA) noexcept {
  void (*filterCoeffs_autocxx_wrapper$)(double, double, double *, double *) = ::filterCoeffs_autocxx_wrapper;
  filterCoeffs_autocxx_wrapper$(tau, Ts, outB, outA);
}

void cxxbridge1$filterInitialState_autocxx_wrapper(double x0, double const *b, double const *a, double *out) noexcept {
  void (*filterInitialState_autocxx_wrapper$)(double, double const *, double const *, double *) = ::filterInitialState_autocxx_wrapper;
  filterInitialState_autocxx_wrapper$(x0, b, a, out);
}

void cxxbridge1$filterAdaptStateForCoeffChange_autocxx_wrapper(double *last_y, ::std::size_t N, double const *b_old, double const *a_old, double const *b_new, double const *a_new, double *state) noexcept {
  void (*filterAdaptStateForCoeffChange_autocxx_wrapper$)(double *, ::std::size_t, double const *, double const *, double const *, double const *, double *) = ::filterAdaptStateForCoeffChange_autocxx_wrapper;
  filterAdaptStateForCoeffChange_autocxx_wrapper$(last_y, N, b_old, a_old, b_new, a_new, state);
}

double cxxbridge1$filterStep_autocxx_wrapper(double x, double const *b, double const *a, double *state) noexcept {
  double (*filterStep_autocxx_wrapper$)(double, double const *, double const *, double *) = ::filterStep_autocxx_wrapper;
  return filterStep_autocxx_wrapper$(x, b, a, state);
}

void cxxbridge1$filterVec_autocxx_wrapper(double const *x, ::std::size_t N, double tau, double Ts, double const *b, double const *a, double *state, double *out) noexcept {
  void (*filterVec_autocxx_wrapper$)(double const *, ::std::size_t, double, double, double const *, double const *, double *, double *) = ::filterVec_autocxx_wrapper;
  filterVec_autocxx_wrapper$(x, N, tau, Ts, b, a, state, out);
}

void cxxbridge1$matrix3SetToScaledIdentity_autocxx_wrapper(double scale, double *out) noexcept {
  void (*matrix3SetToScaledIdentity_autocxx_wrapper$)(double, double *) = ::matrix3SetToScaledIdentity_autocxx_wrapper;
  matrix3SetToScaledIdentity_autocxx_wrapper$(scale, out);
}

void cxxbridge1$matrix3Multiply_autocxx_wrapper(double const *in1, double const *in2, double *out) noexcept {
  void (*matrix3Multiply_autocxx_wrapper$)(double const *, double const *, double *) = ::matrix3Multiply_autocxx_wrapper;
  matrix3Multiply_autocxx_wrapper$(in1, in2, out);
}

void cxxbridge1$matrix3MultiplyTpsFirst_autocxx_wrapper(double const *in1, double const *in2, double *out) noexcept {
  void (*matrix3MultiplyTpsFirst_autocxx_wrapper$)(double const *, double const *, double *) = ::matrix3MultiplyTpsFirst_autocxx_wrapper;
  matrix3MultiplyTpsFirst_autocxx_wrapper$(in1, in2, out);
}

void cxxbridge1$matrix3MultiplyTpsSecond_autocxx_wrapper(double const *in1, double const *in2, double *out) noexcept {
  void (*matrix3MultiplyTpsSecond_autocxx_wrapper$)(double const *, double const *, double *) = ::matrix3MultiplyTpsSecond_autocxx_wrapper;
  matrix3MultiplyTpsSecond_autocxx_wrapper$(in1, in2, out);
}

bool cxxbridge1$matrix3Inv_autocxx_wrapper(double const *in_, double *out) noexcept {
  bool (*matrix3Inv_autocxx_wrapper$)(double const *, double *) = ::matrix3Inv_autocxx_wrapper;
  return matrix3Inv_autocxx_wrapper$(in_, out);
}

void cxxbridge1$VQF_new_autocxx_autocxx_wrapper(::VQF *autocxx_gen_this, double gyrTs, double accTs, double magTs) noexcept {
  void (*VQF_new_autocxx_autocxx_wrapper$)(::VQF *, double, double, double) = ::VQF_new_autocxx_autocxx_wrapper;
  VQF_new_autocxx_autocxx_wrapper$(autocxx_gen_this, gyrTs, accTs, magTs);
}

void cxxbridge1$new1_autocxx_wrapper(::VQF *autocxx_gen_this, ::VQFParams const &params, double gyrTs, double accTs, double magTs) noexcept {
  void (*new1_autocxx_wrapper$)(::VQF *, ::VQFParams const &, double, double, double) = ::new1_autocxx_wrapper;
  new1_autocxx_wrapper$(autocxx_gen_this, params, gyrTs, accTs, magTs);
}

void cxxbridge1$new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFParams *autocxx_gen_this, ::VQFParams *other) noexcept {
  void (*new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFParams *, ::VQFParams *) = ::new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper;
  new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this, other);
}

void cxxbridge1$new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFParams *autocxx_gen_this, ::VQFParams const &other) noexcept {
  void (*new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFParams *, ::VQFParams const &) = ::new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper;
  new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this, other);
}

void cxxbridge1$VQFParams_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFParams *autocxx_gen_this) noexcept {
  void (*VQFParams_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFParams *) = ::VQFParams_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQFParams_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this);
}

void cxxbridge1$VQFState_new_autocxx_autocxx_wrapper(::VQFState *autocxx_gen_this) noexcept {
  void (*VQFState_new_autocxx_autocxx_wrapper$)(::VQFState *) = ::VQFState_new_autocxx_autocxx_wrapper;
  VQFState_new_autocxx_autocxx_wrapper$(autocxx_gen_this);
}

void cxxbridge1$VQFState_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFState *autocxx_gen_this, ::VQFState *other) noexcept {
  void (*VQFState_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFState *, ::VQFState *) = ::VQFState_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQFState_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this, other);
}

void cxxbridge1$VQFState_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFState *autocxx_gen_this, ::VQFState const &other) noexcept {
  void (*VQFState_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFState *, ::VQFState const &) = ::VQFState_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQFState_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this, other);
}

void cxxbridge1$VQFState_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFState *autocxx_gen_this) noexcept {
  void (*VQFState_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFState *) = ::VQFState_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQFState_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this);
}

void cxxbridge1$VQFCoefficients_new_autocxx_autocxx_wrapper(::VQFCoefficients *autocxx_gen_this) noexcept {
  void (*VQFCoefficients_new_autocxx_autocxx_wrapper$)(::VQFCoefficients *) = ::VQFCoefficients_new_autocxx_autocxx_wrapper;
  VQFCoefficients_new_autocxx_autocxx_wrapper$(autocxx_gen_this);
}

void cxxbridge1$VQFCoefficients_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFCoefficients *autocxx_gen_this, ::VQFCoefficients *other) noexcept {
  void (*VQFCoefficients_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFCoefficients *, ::VQFCoefficients *) = ::VQFCoefficients_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQFCoefficients_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this, other);
}

void cxxbridge1$VQFCoefficients_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFCoefficients *autocxx_gen_this, ::VQFCoefficients const &other) noexcept {
  void (*VQFCoefficients_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFCoefficients *, ::VQFCoefficients const &) = ::VQFCoefficients_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQFCoefficients_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this, other);
}

void cxxbridge1$VQFCoefficients_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(::VQFCoefficients *autocxx_gen_this) noexcept {
  void (*VQFCoefficients_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQFCoefficients *) = ::VQFCoefficients_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQFCoefficients_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this);
}

void cxxbridge1$VQF_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(::VQF *autocxx_gen_this, ::VQF *other) noexcept {
  void (*VQF_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQF *, ::VQF *) = ::VQF_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQF_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this, other);
}

void cxxbridge1$VQF_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(::VQF *autocxx_gen_this, ::VQF const &other) noexcept {
  void (*VQF_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQF *, ::VQF const &) = ::VQF_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQF_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this, other);
}

void cxxbridge1$VQF_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(::VQF *autocxx_gen_this) noexcept {
  void (*VQF_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper$)(::VQF *) = ::VQF_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper;
  VQF_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper$(autocxx_gen_this);
}
} // extern "C"
