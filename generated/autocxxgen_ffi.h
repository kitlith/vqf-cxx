#ifndef __AUTOCXXGEN_H__
#define __AUTOCXXGEN_H__

#include "cxx.h"
#include "vqf.hpp"

static_assert(::rust::IsRelocatable<VQFParams>::value, "type VQFParams should be trivially move constructible and trivially destructible to be used with generate_pod! in autocxx");
static_assert(::rust::IsRelocatable<VQFState>::value, "type VQFState should be trivially move constructible and trivially destructible to be used with generate_pod! in autocxx");
static_assert(::rust::IsRelocatable<VQFCoefficients>::value, "type VQFCoefficients should be trivially move constructible and trivially destructible to be used with generate_pod! in autocxx");
static_assert(::rust::IsRelocatable<VQF>::value, "type VQF should be trivially move constructible and trivially destructible to be used with generate_pod! in autocxx");
inline void new_autocxx_autocxx_wrapper(VQFParams* autocxx_gen_this)  { new (autocxx_gen_this) VQFParams(); }
inline void quatMultiply_autocxx_wrapper(const double* arg0, const double* arg1, double* arg2)  { VQF::quatMultiply(arg0, arg1, arg2); }
inline void quatConj_autocxx_wrapper(const double* arg0, double* arg1)  { VQF::quatConj(arg0, arg1); }
inline void quatSetToIdentity_autocxx_wrapper(double* arg0)  { VQF::quatSetToIdentity(arg0); }
inline void quatApplyDelta_autocxx_wrapper(double* arg0, double arg1, double* arg2)  { VQF::quatApplyDelta(arg0, arg1, arg2); }
inline void quatRotate_autocxx_wrapper(const double* arg0, const double* arg1, double* arg2)  { VQF::quatRotate(arg0, arg1, arg2); }
inline double norm_autocxx_wrapper(const double* arg0, size_t arg1)  { return VQF::norm(arg0, arg1); }
inline void normalize_autocxx_wrapper(double* arg0, size_t arg1)  { VQF::normalize(arg0, arg1); }
inline void clip_autocxx_wrapper(double* arg0, size_t arg1, double arg2, double arg3)  { VQF::clip(arg0, arg1, arg2, arg3); }
inline double gainFromTau_autocxx_wrapper(double arg0, double arg1)  { return VQF::gainFromTau(arg0, arg1); }
inline void filterCoeffs_autocxx_wrapper(double arg0, double arg1, double* arg2, double* arg3)  { VQF::filterCoeffs(arg0, arg1, arg2, arg3); }
inline void filterInitialState_autocxx_wrapper(double arg0, const double* arg1, const double* arg2, double* arg3)  { VQF::filterInitialState(arg0, arg1, arg2, arg3); }
inline void filterAdaptStateForCoeffChange_autocxx_wrapper(double* arg0, size_t arg1, const double* arg2, const double* arg3, const double* arg4, const double* arg5, double* arg6)  { VQF::filterAdaptStateForCoeffChange(arg0, arg1, arg2, arg3, arg4, arg5, arg6); }
inline double filterStep_autocxx_wrapper(double arg0, const double* arg1, const double* arg2, double* arg3)  { return VQF::filterStep(arg0, arg1, arg2, arg3); }
inline void filterVec_autocxx_wrapper(const double* arg0, size_t arg1, double arg2, double arg3, const double* arg4, const double* arg5, double* arg6, double* arg7)  { VQF::filterVec(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7); }
inline void matrix3SetToScaledIdentity_autocxx_wrapper(double arg0, double* arg1)  { VQF::matrix3SetToScaledIdentity(arg0, arg1); }
inline void matrix3Multiply_autocxx_wrapper(const double* arg0, const double* arg1, double* arg2)  { VQF::matrix3Multiply(arg0, arg1, arg2); }
inline void matrix3MultiplyTpsFirst_autocxx_wrapper(const double* arg0, const double* arg1, double* arg2)  { VQF::matrix3MultiplyTpsFirst(arg0, arg1, arg2); }
inline void matrix3MultiplyTpsSecond_autocxx_wrapper(const double* arg0, const double* arg1, double* arg2)  { VQF::matrix3MultiplyTpsSecond(arg0, arg1, arg2); }
inline bool matrix3Inv_autocxx_wrapper(const double* arg0, double* arg1)  { return VQF::matrix3Inv(arg0, arg1); }
inline void VQF_new_autocxx_autocxx_wrapper(VQF* autocxx_gen_this, double arg1, double arg2, double arg3)  { new (autocxx_gen_this) VQF(arg1, arg2, arg3); }
inline void new1_autocxx_wrapper(VQF* autocxx_gen_this, const VQFParams& arg1, double arg2, double arg3, double arg4)  { new (autocxx_gen_this) VQF(arg1, arg2, arg3, arg4); }
inline void new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(VQFParams* autocxx_gen_this, VQFParams* arg1)  { new (autocxx_gen_this) VQFParams(std::move(*arg1)); }
inline void new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(VQFParams* autocxx_gen_this, const VQFParams& arg1)  { new (autocxx_gen_this) VQFParams(arg1); }
inline void VQFParams_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(VQFParams* arg0)  { arg0->~VQFParams(); }
inline void VQFState_new_autocxx_autocxx_wrapper(VQFState* autocxx_gen_this)  { new (autocxx_gen_this) VQFState(); }
inline void VQFState_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(VQFState* autocxx_gen_this, VQFState* arg1)  { new (autocxx_gen_this) VQFState(std::move(*arg1)); }
inline void VQFState_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(VQFState* autocxx_gen_this, const VQFState& arg1)  { new (autocxx_gen_this) VQFState(arg1); }
inline void VQFState_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(VQFState* arg0)  { arg0->~VQFState(); }
inline void VQFCoefficients_new_autocxx_autocxx_wrapper(VQFCoefficients* autocxx_gen_this)  { new (autocxx_gen_this) VQFCoefficients(); }
inline void VQFCoefficients_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(VQFCoefficients* autocxx_gen_this, VQFCoefficients* arg1)  { new (autocxx_gen_this) VQFCoefficients(std::move(*arg1)); }
inline void VQFCoefficients_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(VQFCoefficients* autocxx_gen_this, const VQFCoefficients& arg1)  { new (autocxx_gen_this) VQFCoefficients(arg1); }
inline void VQFCoefficients_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(VQFCoefficients* arg0)  { arg0->~VQFCoefficients(); }
inline void VQF_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(VQF* autocxx_gen_this, VQF* arg1)  { new (autocxx_gen_this) VQF(std::move(*arg1)); }
inline void VQF_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(VQF* autocxx_gen_this, const VQF& arg1)  { new (autocxx_gen_this) VQF(arg1); }
inline void VQF_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(VQF* arg0)  { arg0->~VQF(); }
#endif // __AUTOCXXGEN_H__
