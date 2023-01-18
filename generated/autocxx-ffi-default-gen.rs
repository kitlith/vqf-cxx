#[allow(non_snake_case)]
#[allow(dead_code)]
#[allow(non_upper_case_globals)]
#[allow(non_camel_case_types)]
mod ffi {
    unsafe impl cxx::ExternType for bindgen::root::VQFParams {
        type Id = cxx::type_id!("VQFParams");
        type Kind = cxx::kind::Trivial;
    }
    unsafe impl cxx::ExternType for bindgen::root::VQFState {
        type Id = cxx::type_id!("VQFState");
        type Kind = cxx::kind::Trivial;
    }
    unsafe impl cxx::ExternType for bindgen::root::VQFCoefficients {
        type Id = cxx::type_id!("VQFCoefficients");
        type Kind = cxx::kind::Trivial;
    }
    unsafe impl cxx::ExternType for bindgen::root::VQF {
        type Id = cxx::type_id!("VQF");
        type Kind = cxx::kind::Trivial;
    }
    mod bindgen {
        pub(super) mod root {
            #[doc = " @brief Struct containing all tuning parameters used by the VQF class."]
            #[doc = ""]
            #[doc = " The parameters influence the behavior of the algorithm and are independent of the sampling rate of the IMU data. The"]
            #[doc = " constructor sets all parameters to the default values."]
            #[doc = ""]
            #[doc = " The parameters #motionBiasEstEnabled, #restBiasEstEnabled, and #magDistRejectionEnabled can be used to enable/disable"]
            #[doc = " the main features of the VQF algorithm. The time constants #tauAcc and #tauMag can be tuned to change the trust on"]
            #[doc = " the accelerometer and magnetometer measurements, respectively. The remaining parameters influence bias estimation"]
            #[doc = " and magnetometer rejection."]
            #[repr(C)]
            pub struct VQFParams {
                #[doc = " @brief Time constant \\f$\\tau_\\mathrm{acc}\\f$ for accelerometer low-pass filtering in seconds."]
                #[doc = ""]
                #[doc = " Small values for \\f$\\tau_\\mathrm{acc}\\f$ imply trust on the accelerometer measurements and while large values of"]
                #[doc = " \\f$\\tau_\\mathrm{acc}\\f$ imply trust on the gyroscope measurements."]
                #[doc = ""]
                #[doc = " The time constant \\f$\\tau_\\mathrm{acc}\\f$ corresponds to the cutoff frequency \\f$f_\\mathrm{c}\\f$ of the"]
                #[doc = " second-order Butterworth low-pass filter as follows: \\f$f_\\mathrm{c} = \\frac{\\sqrt{2}}{2\\pi\\tau_\\mathrm{acc}}\\f$."]
                #[doc = ""]
                #[doc = " Default value: 3.0 s"]
                pub tauAcc: root::vqf_real_t,
                #[doc = " @brief Time constant \\f$\\tau_\\mathrm{mag}\\f$ for magnetometer update in seconds."]
                #[doc = ""]
                #[doc = " Small values for \\f$\\tau_\\mathrm{mag}\\f$ imply trust on the magnetometer measurements and while large values of"]
                #[doc = " \\f$\\tau_\\mathrm{mag}\\f$ imply trust on the gyroscope measurements."]
                #[doc = ""]
                #[doc = " The time constant \\f$\\tau_\\mathrm{mag}\\f$ corresponds to the cutoff frequency \\f$f_\\mathrm{c}\\f$ of the"]
                #[doc = " first-order low-pass filter for the heading correction as follows:"]
                #[doc = " \\f$f_\\mathrm{c} = \\frac{1}{2\\pi\\tau_\\mathrm{mag}}\\f$."]
                #[doc = ""]
                #[doc = " Default value: 9.0 s"]
                pub tauMag: root::vqf_real_t,
                #[doc = " @brief Enables gyroscope bias estimation during motion phases."]
                #[doc = ""]
                #[doc = " If set to true (default), gyroscope bias is estimated based on the inclination correction only, i.e. without"]
                #[doc = " using magnetometer measurements."]
                pub motionBiasEstEnabled: bool,
                #[doc = " @brief Enables rest detection and gyroscope bias estimation during rest phases."]
                #[doc = ""]
                #[doc = " If set to true (default), phases in which the IMU is at rest are detected. During rest, the gyroscope bias"]
                #[doc = " is estimated from the low-pass filtered gyroscope readings."]
                pub restBiasEstEnabled: bool,
                #[doc = " @brief Enables magnetic disturbance detection and magnetic disturbance rejection."]
                #[doc = ""]
                #[doc = " If set to true (default), the magnetic field is analyzed. For short disturbed phases, the magnetometer-based"]
                #[doc = " correction is disabled totally. If the magnetic field is always regarded as disturbed or if the duration of"]
                #[doc = " the disturbances exceeds #magMaxRejectionTime, magnetometer-based updates are performed, but with an increased"]
                #[doc = " time constant."]
                pub magDistRejectionEnabled: bool,
                #[doc = " @brief Standard deviation of the initial bias estimation uncertainty (in degrees per second)."]
                #[doc = ""]
                #[doc = " Default value: 0.5 °/s"]
                pub biasSigmaInit: root::vqf_real_t,
                #[doc = " @brief Time in which the bias estimation uncertainty increases from 0 °/s to 0.1 °/s (in seconds)."]
                #[doc = ""]
                #[doc = " This value determines the system noise assumed by the Kalman filter."]
                #[doc = ""]
                #[doc = " Default value: 100.0 s"]
                pub biasForgettingTime: root::vqf_real_t,
                #[doc = " @brief Maximum expected gyroscope bias (in degrees per second)."]
                #[doc = ""]
                #[doc = " This value is used to clip the bias estimate and the measurement error in the bias estimation update step. It is"]
                #[doc = " further used by the rest detection algorithm in order to not regard measurements with a large but constant"]
                #[doc = " angular rate as rest."]
                #[doc = ""]
                #[doc = " Default value: 2.0 °/s"]
                pub biasClip: root::vqf_real_t,
                #[doc = " @brief Standard deviation of the converged bias estimation uncertainty during motion (in degrees per second)."]
                #[doc = ""]
                #[doc = " This value determines the trust on motion bias estimation updates. A small value leads to fast convergence."]
                #[doc = ""]
                #[doc = " Default value: 0.1 °/s"]
                pub biasSigmaMotion: root::vqf_real_t,
                #[doc = " @brief Forgetting factor for unobservable bias in vertical direction during motion."]
                #[doc = ""]
                #[doc = " As magnetometer measurements are deliberately not used during motion bias estimation, gyroscope bias is not"]
                #[doc = " observable in vertical direction. This value is the relative weight of an artificial zero measurement that"]
                #[doc = " ensures that the bias estimate in the unobservable direction will eventually decay to zero."]
                #[doc = ""]
                #[doc = " Default value: 0.0001"]
                pub biasVerticalForgettingFactor: root::vqf_real_t,
                #[doc = " @brief Standard deviation of the converged bias estimation uncertainty during rest (in degrees per second)."]
                #[doc = ""]
                #[doc = " This value determines the trust on rest bias estimation updates. A small value leads to fast convergence."]
                #[doc = ""]
                #[doc = " Default value: 0.03 °"]
                pub biasSigmaRest: root::vqf_real_t,
                #[doc = " @brief Time threshold for rest detection (in seconds)."]
                #[doc = ""]
                #[doc = " Rest is detected when the measurements have been close to the low-pass filtered reference for the given time."]
                #[doc = ""]
                #[doc = " Default value: 1.5 s"]
                pub restMinT: root::vqf_real_t,
                #[doc = " @brief Time constant for the low-pass filter used in rest detection (in seconds)."]
                #[doc = ""]
                #[doc = " This time constant characterizes a second-order Butterworth low-pass filter used to obtain the reference for"]
                #[doc = " rest detection."]
                #[doc = ""]
                #[doc = " Default value: 0.5 s"]
                pub restFilterTau: root::vqf_real_t,
                #[doc = " @brief Angular velocity threshold for rest detection (in °/s)."]
                #[doc = ""]
                #[doc = " For rest to be detected, the norm of the deviation between measurement and reference must be below the given"]
                #[doc = " threshold. (Furthermore, the absolute value of each component must be below #biasClip)."]
                #[doc = ""]
                #[doc = " Default value: 2.0 °/s"]
                pub restThGyr: root::vqf_real_t,
                #[doc = " @brief Acceleration threshold for rest detection (in m/s²)."]
                #[doc = ""]
                #[doc = " For rest to be detected, the norm of the deviation between measurement and reference must be below the given"]
                #[doc = " threshold."]
                #[doc = ""]
                #[doc = " Default value: 0.5 m/s²"]
                pub restThAcc: root::vqf_real_t,
                #[doc = " @brief Time constant for current norm/dip value in magnetic disturbance detection (in seconds)."]
                #[doc = ""]
                #[doc = " This (very fast) low-pass filter is intended to provide additional robustness when the magnetometer measurements"]
                #[doc = " are noisy or not sampled perfectly in sync with the gyroscope measurements. Set to -1 to disable the low-pass"]
                #[doc = " filter and directly use the magnetometer measurements."]
                #[doc = ""]
                #[doc = " Default value: 0.05 s"]
                pub magCurrentTau: root::vqf_real_t,
                #[doc = " @brief Time constant for the adjustment of the magnetic field reference (in seconds)."]
                #[doc = ""]
                #[doc = " This adjustment allows the reference estimate to converge to the observed undisturbed field."]
                #[doc = ""]
                #[doc = " Default value: 20.0 s"]
                pub magRefTau: root::vqf_real_t,
                #[doc = " @brief Relative threshold for the magnetic field strength for magnetic disturbance detection."]
                #[doc = ""]
                #[doc = " This value is relative to the reference norm."]
                #[doc = ""]
                #[doc = " Default value: 0.1 (10%)"]
                pub magNormTh: root::vqf_real_t,
                #[doc = " @brief Threshold for the magnetic field dip angle for magnetic disturbance detection (in degrees)."]
                #[doc = ""]
                #[doc = " Default vaule: 10 °"]
                pub magDipTh: root::vqf_real_t,
                #[doc = " @brief Duration after which to accept a different homogeneous magnetic field (in seconds)."]
                #[doc = ""]
                #[doc = " A different magnetic field reference is accepted as the new field when the measurements are within the thresholds"]
                #[doc = " #magNormTh and #magDipTh for the given time. Additionally, only phases with sufficient movement, specified by"]
                #[doc = " #magNewMinGyr, count."]
                #[doc = ""]
                #[doc = " Default value: 20.0"]
                pub magNewTime: root::vqf_real_t,
                #[doc = " @brief Duration after which to accept a homogeneous magnetic field for the first time (in seconds)."]
                #[doc = ""]
                #[doc = " This value is used instead of #magNewTime when there is no current estimate in order to allow for the initial"]
                #[doc = " magnetic field reference to be obtained faster."]
                #[doc = ""]
                #[doc = " Default value: 5.0"]
                pub magNewFirstTime: root::vqf_real_t,
                #[doc = " @brief Minimum angular velocity needed in order to count time for new magnetic field acceptance (in °/s)."]
                #[doc = ""]
                #[doc = " Durations for which the angular velocity norm is below this threshold do not count towards reaching #magNewTime."]
                #[doc = ""]
                #[doc = " Default value: 20.0 °/s"]
                pub magNewMinGyr: root::vqf_real_t,
                #[doc = " @brief Minimum duration within thresholds after which to regard the field as undisturbed again (in seconds)."]
                #[doc = ""]
                #[doc = " Default value: 0.5 s"]
                pub magMinUndisturbedTime: root::vqf_real_t,
                #[doc = " @brief Maximum duration of full magnetic disturbance rejection (in seconds)."]
                #[doc = ""]
                #[doc = " For magnetic disturbances up to this duration, heading correction is fully disabled and heading changes are"]
                #[doc = " tracked by gyroscope only. After this duration (or for many small disturbed phases without sufficient time in the"]
                #[doc = " undisturbed field in between), the heading correction is performed with an increased time constant (see"]
                #[doc = " #magRejectionFactor)."]
                #[doc = ""]
                #[doc = " Default value: 60.0 s"]
                pub magMaxRejectionTime: root::vqf_real_t,
                #[doc = " @brief Factor by which to slow the heading correction during long disturbed phases."]
                #[doc = ""]
                #[doc = " After #magMaxRejectionTime of full magnetic disturbance rejection, heading correction is performed with an"]
                #[doc = " increased time constant. This parameter (approximately) specifies the factor of the increase."]
                #[doc = ""]
                #[doc = " Furthermore, after spending #magMaxRejectionTime/#magRejectionFactor seconds in an undisturbed magnetic field,"]
                #[doc = " the time is reset and full magnetic disturbance rejection will be performed for up to #magMaxRejectionTime again."]
                #[doc = ""]
                #[doc = " Default value: 2.0"]
                pub magRejectionFactor: root::vqf_real_t,
            }
            #[doc = " @brief Struct containing the filter state of the VQF class."]
            #[doc = ""]
            #[doc = " The relevant parts of the state can be accessed via functions of the VQF class, e.g. VQF::getQuat6D(),"]
            #[doc = " VQF::getQuat9D(), VQF::getGyrBiasEstimate(), VQF::setGyrBiasEstimate(), VQF::getRestDetected() and"]
            #[doc = " VQF::getMagDistDetected(). To reset the state to the initial values, use VQF::resetState()."]
            #[doc = ""]
            #[doc = " Direct access to the full state is typically not needed but can be useful in some cases, e.g. for debugging. For this"]
            #[doc = " purpose, the state can be accessed by VQF::getState() and set by VQF::setState()."]
            #[repr(C)]
            pub struct VQFState {
                #[doc = " @brief Angular velocity strapdown integration quaternion \\f$^{\\mathcal{S}_i}_{\\mathcal{I}_i}\\mathbf{q}\\f$."]
                pub gyrQuat: [root::vqf_real_t; 4usize],
                #[doc = " @brief Inclination correction quaternion \\f$^{\\mathcal{I}_i}_{\\mathcal{E}_i}\\mathbf{q}\\f$."]
                pub accQuat: [root::vqf_real_t; 4usize],
                #[doc = " @brief Heading difference \\f$\\delta\\f$ between \\f$\\mathcal{E}_i\\f$ and \\f$\\mathcal{E}\\f$."]
                #[doc = ""]
                #[doc = " \\f$^{\\mathcal{E}_i}_{\\mathcal{E}}\\mathbf{q} = \\begin{bmatrix}\\cos\\frac{\\delta}{2} & 0 & 0 &"]
                #[doc = " \\sin\\frac{\\delta}{2}\\end{bmatrix}^T\\f$."]
                pub delta: root::vqf_real_t,
                #[doc = " @brief True if it has been detected that the IMU is currently at rest."]
                #[doc = ""]
                #[doc = " Used to switch between rest and motion gyroscope bias estimation."]
                pub restDetected: bool,
                #[doc = " @brief True if magnetic disturbances have been detected."]
                pub magDistDetected: bool,
                #[doc = " @brief Last low-pass filtered acceleration in the \\f$\\mathcal{I}_i\\f$ frame."]
                pub lastAccLp: [root::vqf_real_t; 3usize],
                #[doc = " @brief Internal low-pass filter state for #lastAccLp."]
                pub accLpState: [f64; 6usize],
                #[doc = " @brief Last inclination correction angular rate."]
                #[doc = ""]
                #[doc = " Change to inclination correction quaternion \\f$^{\\mathcal{I}_i}_{\\mathcal{E}_i}\\mathbf{q}\\f$ performed in the"]
                #[doc = " last accelerometer update, expressed as an angular rate (in rad/s)."]
                pub lastAccCorrAngularRate: root::vqf_real_t,
                #[doc = " @brief Gain used for heading correction to ensure fast initial convergence."]
                #[doc = ""]
                #[doc = " This value is used as the gain for heading correction in the beginning if it is larger than the normal filter"]
                #[doc = " gain. It is initialized to 1 and then updated to 0.5, 0.33, 0.25, ... After VQFParams::tauMag seconds, it is"]
                #[doc = " set to zero."]
                pub kMagInit: root::vqf_real_t,
                #[doc = " @brief Last heading disagreement angle."]
                #[doc = ""]
                #[doc = " Disagreement between the heading \\f$\\hat\\delta\\f$ estimated from the last magnetometer sample and the state"]
                #[doc = " \\f$\\delta\\f$ (in rad)."]
                pub lastMagDisAngle: root::vqf_real_t,
                #[doc = " @brief Last heading correction angular rate."]
                #[doc = ""]
                #[doc = " Change to heading \\f$\\delta\\f$ performed in the last magnetometer update,"]
                #[doc = " expressed as an angular rate (in rad/s)."]
                pub lastMagCorrAngularRate: root::vqf_real_t,
                #[doc = " @brief Current gyroscope bias estimate (in rad/s)."]
                pub bias: [root::vqf_real_t; 3usize],
                #[doc = " @brief Covariance matrix of the gyroscope bias estimate."]
                #[doc = ""]
                #[doc = " The 3x3 matrix is stored in row-major order. Note that for numeric reasons the internal unit used is 0.01 °/s,"]
                #[doc = " i.e. to get the standard deviation in degrees per second use \\f$\\sigma = \\frac{\\sqrt{p_{ii}}}{100}\\f$."]
                pub biasP: [root::vqf_real_t; 9usize],
                #[doc = " @brief Internal state of the Butterworth low-pass filter for the rotation matrix coefficients used in motion"]
                #[doc = " bias estimation."]
                pub motionBiasEstRLpState: [f64; 18usize],
                #[doc = " @brief Internal low-pass filter state for the rotated bias estimate used in motion bias estimation."]
                pub motionBiasEstBiasLpState: [f64; 4usize],
                #[doc = " @brief Last (squared) deviations from the reference of the last sample used in rest detection."]
                #[doc = ""]
                #[doc = " Looking at those values can be useful to understand how rest detection is working and which thresholds are"]
                #[doc = " suitable. The array contains the last values for gyroscope and accelerometer in the respective"]
                #[doc = " units. Note that the values are squared."]
                #[doc = ""]
                #[doc = " The method VQF::getRelativeRestDeviations() provides an easier way to obtain and interpret those values."]
                pub restLastSquaredDeviations: [root::vqf_real_t; 2usize],
                #[doc = " @brief The current duration for which all sensor readings are within the rest detection thresholds."]
                #[doc = ""]
                #[doc = " Rest is detected if this value is larger or equal to VQFParams::restMinT."]
                pub restT: root::vqf_real_t,
                #[doc = " @brief Last low-pass filtered gyroscope measurement used as the reference for rest detection."]
                #[doc = ""]
                #[doc = " Note that this value is also used for gyroscope bias estimation when rest is detected."]
                pub restLastGyrLp: [root::vqf_real_t; 3usize],
                #[doc = " @brief Internal low-pass filter state for #restLastGyrLp."]
                pub restGyrLpState: [f64; 6usize],
                #[doc = " @brief Last low-pass filtered accelerometer measurement used as the reference for rest detection."]
                pub restLastAccLp: [root::vqf_real_t; 3usize],
                #[doc = " @brief Internal low-pass filter state for #restLastAccLp."]
                pub restAccLpState: [f64; 6usize],
                #[doc = " @brief Norm of the currently accepted magnetic field reference."]
                #[doc = ""]
                #[doc = " A value of -1 indicates that no homogeneous field is found yet."]
                pub magRefNorm: root::vqf_real_t,
                #[doc = " @brief Dip angle of the currently accepted magnetic field reference."]
                pub magRefDip: root::vqf_real_t,
                #[doc = " @brief The current duration for which the current norm and dip are close to the reference."]
                #[doc = ""]
                #[doc = " The magnetic field is regarded as undisturbed when this value reaches VQFParams::magMinUndisturbedTime."]
                pub magUndisturbedT: root::vqf_real_t,
                #[doc = " @brief The current duration for which the magnetic field was rejected."]
                #[doc = ""]
                #[doc = " If the magnetic field is disturbed and this value is smaller than VQFParams::magMaxRejectionTime, heading"]
                #[doc = " correction updates are fully disabled."]
                pub magRejectT: root::vqf_real_t,
                #[doc = " @brief Norm of the alternative magnetic field reference currently being evaluated."]
                pub magCandidateNorm: root::vqf_real_t,
                #[doc = " @brief Dip angle of the alternative magnetic field reference currently being evaluated."]
                pub magCandidateDip: root::vqf_real_t,
                #[doc = " @brief The current duration for which the norm and dip are close to the candidate."]
                #[doc = ""]
                #[doc = " If this value exceeds VQFParams::magNewTime (or VQFParams::magNewFirstTime if #magRefNorm < 0), the current"]
                #[doc = " candidate is accepted as the new reference."]
                pub magCandidateT: root::vqf_real_t,
                #[doc = " @brief Norm and dip angle of the current magnetometer measurements."]
                #[doc = ""]
                #[doc = " Slightly low-pass filtered, see VQFParams::magCurrentTau."]
                pub magNormDip: [root::vqf_real_t; 2usize],
                #[doc = " @brief Internal low-pass filter state for the current norm and dip angle."]
                pub magNormDipLpState: [f64; 4usize],
            }
            #[doc = " @brief Struct containing coefficients used by the VQF class."]
            #[doc = ""]
            #[doc = " Coefficients are values that depend on the parameters and the sampling times, but do not change during update steps."]
            #[doc = " They are calculated in VQF::setup()."]
            #[repr(C)]
            pub struct VQFCoefficients {
                #[doc = " @brief Sampling time of the gyroscope measurements (in seconds)."]
                pub gyrTs: root::vqf_real_t,
                #[doc = " @brief Sampling time of the accelerometer measurements (in seconds)."]
                pub accTs: root::vqf_real_t,
                #[doc = " @brief Sampling time of the magnetometer measurements (in seconds)."]
                pub magTs: root::vqf_real_t,
                #[doc = " @brief Numerator coefficients of the acceleration low-pass filter."]
                #[doc = ""]
                #[doc = " The array contains \\f$\\begin{bmatrix}b_0 & b_1 & b_2\\end{bmatrix}\\f$."]
                pub accLpB: [f64; 3usize],
                #[doc = " @brief Denominator coefficients of the acceleration low-pass filter."]
                #[doc = ""]
                #[doc = " The array contains \\f$\\begin{bmatrix}a_1 & a_2\\end{bmatrix}\\f$ and \\f$a_0=1\\f$."]
                pub accLpA: [f64; 2usize],
                #[doc = " @brief Gain of the first-order filter used for heading correction."]
                pub kMag: root::vqf_real_t,
                #[doc = " @brief Variance of the initial gyroscope bias estimate."]
                pub biasP0: root::vqf_real_t,
                #[doc = " @brief System noise variance used in gyroscope bias estimation."]
                pub biasV: root::vqf_real_t,
                #[doc = " @brief Measurement noise variance for the motion gyroscope bias estimation update."]
                pub biasMotionW: root::vqf_real_t,
                #[doc = " @brief Measurement noise variance for the motion gyroscope bias estimation update in vertical direction."]
                pub biasVerticalW: root::vqf_real_t,
                #[doc = " @brief Measurement noise variance for the rest gyroscope bias estimation update."]
                pub biasRestW: root::vqf_real_t,
                #[doc = " @brief Numerator coefficients of the gyroscope measurement low-pass filter for rest detection."]
                pub restGyrLpB: [f64; 3usize],
                #[doc = " @brief Denominator coefficients of the gyroscope measurement low-pass filter for rest detection."]
                pub restGyrLpA: [f64; 2usize],
                #[doc = " @brief Numerator coefficients of the accelerometer measurement low-pass filter for rest detection."]
                pub restAccLpB: [f64; 3usize],
                #[doc = " @brief Denominator coefficients of the accelerometer measurement low-pass filter for rest detection."]
                pub restAccLpA: [f64; 2usize],
                #[doc = " @brief Gain of the first-order filter used for to update the magnetic field reference and candidate."]
                pub kMagRef: root::vqf_real_t,
                #[doc = " @brief Numerator coefficients of the low-pass filter for the current magnetic norm and dip."]
                pub magNormDipLpB: [f64; 3usize],
                #[doc = " @brief Denominator coefficients of the low-pass filter for the current magnetic norm and dip."]
                pub magNormDipLpA: [f64; 2usize],
            }
            #[doc = " @brief A Versatile Quaternion-based Filter for IMU Orientation Estimation."]
            #[doc = ""]
            #[doc = " \\rst"]
            #[doc = " This class implements the orientation estimation filter described in the following publication:"]
            #[doc = ""]
            #[doc = ""]
            #[doc = "     D. Laidig, T. Seel. \"VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic"]
            #[doc = "     Disturbance Rejection.\" arXiv preprint, 2022. `arXiv:2203.17024 <https://arxiv.org/abs/2203.17024>`_."]
            #[doc = ""]
            #[doc = " The filter can perform simultaneous 6D (magnetometer-free) and 9D (gyr+acc+mag) sensor fusion and can also be used"]
            #[doc = " without magnetometer data. It performs rest detection, gyroscope bias estimation during rest and motion, and magnetic"]
            #[doc = " disturbance detection and rejection. Different sampling rates for gyroscopes, accelerometers, and magnetometers are"]
            #[doc = " supported as well. While in most cases, the defaults will be reasonable, the algorithm can be influenced via a"]
            #[doc = " number of tuning parameters."]
            #[doc = ""]
            #[doc = " To use this C++ implementation,"]
            #[doc = ""]
            #[doc = " 1. create a instance of the class and provide the sampling time and, optionally, parameters"]
            #[doc = " 2. for every sample, call one of the update functions to feed the algorithm with IMU data"]
            #[doc = " 3. access the estimation results with :meth:`getQuat6D() <VQF.getQuat6D>`, :meth:`getQuat9D() <VQF.getQuat9D>` and"]
            #[doc = "    the other getter methods."]
            #[doc = ""]
            #[doc = " If the full data is available in (row-major) data buffers, you can use :meth:`updateBatch() <VQF.updateBatch>`."]
            #[doc = ""]
            #[doc = " This class is the main C++ implementation of the algorithm. Depending on use case and programming language of choice,"]
            #[doc = " the following alternatives might be useful:"]
            #[doc = ""]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " |                        | Full Version             | Basic Version            | Offline Version           |"]
            #[doc = " |                        |                          |                          |                           |"]
            #[doc = " +========================+==========================+==========================+===========================+"]
            #[doc = " | **C++**                | **VQF (this class)**     | :cpp:class:`BasicVQF`    | :cpp:func:`offlineVQF`    |"]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " | **Python/C++ (fast)**  | :py:class:`vqf.VQF`      | :py:class:`vqf.BasicVQF` | :py:meth:`vqf.offlineVQF` |"]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " | **Pure Python (slow)** | :py:class:`vqf.PyVQF`    | --                       | --                        |"]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " | **Pure Matlab (slow)** | :mat:class:`VQF.m <VQF>` | --                       | --                        |"]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " \\endrst"]
            #[repr(C)]
            pub struct VQF {
                #[doc = " @brief Contains the current parameters."]
                #[doc = ""]
                #[doc = " See #getParams. To set parameters, pass them to the constructor. Part of the parameters can be changed with"]
                #[doc = " #setTauAcc, #setTauMag, #setMotionBiasEstEnabled, #setRestBiasEstEnabled, #setMagDistRejectionEnabled, and"]
                #[doc = " #setRestDetectionThresholds."]
                params: root::VQFParams,
                #[doc = " @brief Contains the current state."]
                #[doc = ""]
                #[doc = " See #getState, #getState and #resetState."]
                state: root::VQFState,
                #[doc = " @brief Contains the current coefficients (calculated in #setup)."]
                #[doc = ""]
                #[doc = " See #getCoeffs."]
                coeffs: root::VQFCoefficients,
            }
            pub type vqf_real_t = f64;
            impl VQFParams {
                #[doc = " @brief Constructor that initializes the struct with the default parameters."]
                pub fn new() -> impl autocxx::moveit::new::New<Output = Self> {
                    unsafe {
                        autocxx::moveit::new::by_raw(move |this| {
                            let this = this.get_unchecked_mut().as_mut_ptr();
                            cxxbridge::new_autocxx_autocxx_wrapper(this)
                        })
                    }
                }
            }
            impl VQF {
                #[doc = " @brief Performs quaternion multiplication (\\f$\\mathbf{q}_\\mathrm{out} = \\mathbf{q}_1 \\otimes \\mathbf{q}_2\\f$)."]
                pub unsafe fn quatMultiply(q1: *const f64, q2: *const f64, out: *mut f64) {
                    cxxbridge::quatMultiply_autocxx_wrapper(q1, q2, out)
                }
                #[doc = " @brief Calculates the quaternion conjugate (\\f$\\mathbf{q}_\\mathrm{out} = \\mathbf{q}^*\\f$)."]
                pub unsafe fn quatConj(q: *const f64, out: *mut f64) {
                    cxxbridge::quatConj_autocxx_wrapper(q, out)
                }
                #[doc = " @brief Sets the output quaternion to the identity quaternion (\\f$\\mathbf{q}_\\mathrm{out} ="]
                #[doc = " \\begin{bmatrix}1 & 0 & 0 & 0\\end{bmatrix}\\f$)."]
                pub unsafe fn quatSetToIdentity(out: *mut f64) {
                    cxxbridge::quatSetToIdentity_autocxx_wrapper(out)
                }
                #[doc = " @brief Applies a heading rotation by the angle delta (in rad) to a quaternion."]
                #[doc = ""]
                #[doc = " \\f$\\mathbf{q}_\\mathrm{out} = \\begin{bmatrix}\\cos\\frac{\\delta}{2} & 0 & 0 &"]
                #[doc = " \\sin\\frac{\\delta}{2}\\end{bmatrix} \\otimes \\mathbf{q}\\f$"]
                pub unsafe fn quatApplyDelta(q: *mut f64, delta: f64, out: *mut f64) {
                    cxxbridge::quatApplyDelta_autocxx_wrapper(q, delta, out)
                }
                #[doc = " @brief Rotates a vector with a given quaternion."]
                #[doc = ""]
                #[doc = " \\f$\\begin{bmatrix}0 & \\mathbf{v}_\\mathrm{out}\\end{bmatrix} ="]
                #[doc = " \\mathbf{q} \\otimes \\begin{bmatrix}0 & \\mathbf{v}\\end{bmatrix} \\otimes \\mathbf{q}^*\\f$"]
                pub unsafe fn quatRotate(q: *const f64, v: *const f64, out: *mut f64) {
                    cxxbridge::quatRotate_autocxx_wrapper(q, v, out)
                }
                #[doc = " @brief Calculates the Euclidean norm of a vector."]
                #[doc = " @param vec pointer to an array of N elements"]
                #[doc = " @param N number of elements"]
                pub unsafe fn norm(vec: *const f64, N: usize) -> f64 {
                    cxxbridge::norm_autocxx_wrapper(vec, N)
                }
                #[doc = " @brief Normalizes a vector in-place."]
                #[doc = " @param vec pointer to an array of N elements that will be normalized"]
                #[doc = " @param N number of elements"]
                pub unsafe fn normalize(vec: *mut f64, N: usize) {
                    cxxbridge::normalize_autocxx_wrapper(vec, N)
                }
                #[doc = " @brief Clips a vector in-place."]
                #[doc = " @param vec pointer to an array of N elements that will be clipped"]
                #[doc = " @param N number of elements"]
                #[doc = " @param min smallest allowed value"]
                #[doc = " @param max largest allowed value"]
                pub unsafe fn clip(vec: *mut f64, N: usize, min: f64, max: f64) {
                    cxxbridge::clip_autocxx_wrapper(vec, N, min, max)
                }
                #[doc = " @brief Calculates the gain for a first-order low-pass filter from the 1/e time constant."]
                #[doc = ""]
                #[doc = " \\f$k = 1 - \\exp\\left(-\\frac{T_\\mathrm{s}}{\\tau}\\right)\\f$"]
                #[doc = ""]
                #[doc = " The cutoff frequency of the resulting filter is \\f$f_\\mathrm{c} = \\frac{1}{2\\pi\\tau}\\f$."]
                #[doc = ""]
                #[doc = " @param tau time constant \\f$\\tau\\f$ in seconds - use -1 to disable update (\\f$k=0\\f$) or 0 to obtain"]
                #[doc = "        unfiltered values (\\f$k=1\\f$)"]
                #[doc = " @param Ts sampling time \\f$T_\\mathrm{s}\\f$ in seconds"]
                #[doc = " @return filter gain *k*"]
                pub fn gainFromTau(tau: f64, Ts: f64) -> f64 {
                    cxxbridge::gainFromTau_autocxx_wrapper(tau, Ts)
                }
                #[doc = " @brief Calculates coefficients for a second-order Butterworth low-pass filter."]
                #[doc = ""]
                #[doc = " The filter is parametrized via the time constant of the dampened, non-oscillating part of step response and the"]
                #[doc = " resulting cutoff frequency is \\f$f_\\mathrm{c} = \\frac{\\sqrt{2}}{2\\pi\\tau}\\f$."]
                #[doc = ""]
                #[doc = " @param tau time constant \\f$\\tau\\f$ in seconds"]
                #[doc = " @param Ts sampling time \\f$T_\\mathrm{s}\\f$ in seconds"]
                #[doc = " @param outB output array for numerator coefficients"]
                #[doc = " @param outA output array for denominator coefficients (without \\f$a_0=1\\f$)"]
                pub unsafe fn filterCoeffs(tau: f64, Ts: f64, outB: *mut f64, outA: *mut f64) {
                    cxxbridge::filterCoeffs_autocxx_wrapper(tau, Ts, outB, outA)
                }
                #[doc = " @brief Calculates the initial filter state for a given steady-state value."]
                #[doc = " @param x0 steady state value"]
                #[doc = " @param b numerator coefficients"]
                #[doc = " @param a denominator coefficients (without \\f$a_0=1\\f$)"]
                #[doc = " @param out output array for filter state"]
                pub unsafe fn filterInitialState(
                    x0: f64,
                    b: *const f64,
                    a: *const f64,
                    out: *mut f64,
                ) {
                    cxxbridge::filterInitialState_autocxx_wrapper(x0, b, a, out)
                }
                #[doc = " @brief Adjusts the filter state when changing coefficients."]
                #[doc = ""]
                #[doc = " This function assumes that the filter is currently in a steady state, i.e. the last input values and the last"]
                #[doc = " output values are all equal. Based on this, the filter state is adjusted to new filter coefficients so that the"]
                #[doc = " output does not jump."]
                #[doc = ""]
                #[doc = " @param last_y last filter output values (array of size N)"]
                #[doc = " @param N number of values in vector-valued signal"]
                #[doc = " @param b_old previous numerator coefficients"]
                #[doc = " @param a_old previous denominator coefficients (without \\f$a_0=1\\f$)"]
                #[doc = " @param b_new new numerator coefficients"]
                #[doc = " @param a_new new denominator coefficients (without \\f$a_0=1\\f$)"]
                #[doc = " @param state filter state (array of size N*2, will be modified)"]
                pub unsafe fn filterAdaptStateForCoeffChange(
                    last_y: *mut f64,
                    N: usize,
                    b_old: *const f64,
                    a_old: *const f64,
                    b_new: *const f64,
                    a_new: *const f64,
                    state: *mut f64,
                ) {
                    cxxbridge::filterAdaptStateForCoeffChange_autocxx_wrapper(
                        last_y, N, b_old, a_old, b_new, a_new, state,
                    )
                }
                #[doc = " @brief Performs a filter step for a scalar value."]
                #[doc = " @param x input value"]
                #[doc = " @param b numerator coefficients"]
                #[doc = " @param a denominator coefficients (without \\f$a_0=1\\f$)"]
                #[doc = " @param state filter state array (will be modified)"]
                #[doc = " @return filtered value"]
                pub unsafe fn filterStep(
                    x: f64,
                    b: *const f64,
                    a: *const f64,
                    state: *mut f64,
                ) -> f64 {
                    cxxbridge::filterStep_autocxx_wrapper(x, b, a, state)
                }
                #[doc = " @brief Performs filter step for vector-valued signal with averaging-based initialization."]
                #[doc = ""]
                #[doc = " During the first \\f$\\tau\\f$ seconds, the filter output is the mean of the previous samples. At \\f$t=\\tau\\f$, the"]
                #[doc = " initial conditions for the low-pass filter are calculated based on the current mean value and from then on,"]
                #[doc = " regular filtering with the rational transfer function described by the coefficients b and a is performed."]
                #[doc = ""]
                #[doc = " @param x input values (array of size N)"]
                #[doc = " @param N number of values in vector-valued signal"]
                #[doc = " @param tau filter time constant \\f$\\tau\\f$ in seconds (used for initialization)"]
                #[doc = " @param Ts sampling time \\f$T_\\mathrm{s}\\f$ in seconds (used for initialization)"]
                #[doc = " @param b numerator coefficients"]
                #[doc = " @param a denominator coefficients (without \\f$a_0=1\\f$)"]
                #[doc = " @param state filter state (array of size N*2, will be modified)"]
                #[doc = " @param out output array for filtered values (size N)"]
                pub unsafe fn filterVec(
                    x: *const f64,
                    N: usize,
                    tau: f64,
                    Ts: f64,
                    b: *const f64,
                    a: *const f64,
                    state: *mut f64,
                    out: *mut f64,
                ) {
                    cxxbridge::filterVec_autocxx_wrapper(x, N, tau, Ts, b, a, state, out)
                }
                #[doc = " @brief Sets a 3x3 matrix to a scaled version of the identity matrix."]
                #[doc = " @param scale value of diagonal elements"]
                #[doc = " @param out output array of size 9 (3x3 matrix stored in row-major order)"]
                pub unsafe fn matrix3SetToScaledIdentity(scale: f64, out: *mut f64) {
                    cxxbridge::matrix3SetToScaledIdentity_autocxx_wrapper(scale, out)
                }
                #[doc = " @brief Performs 3x3 matrix multiplication (\\f$\\mathbf{M}_\\mathrm{out} = \\mathbf{M}_1\\mathbf{M}_2\\f$)."]
                #[doc = " @param in1 input 3x3 matrix \\f$\\mathbf{M}_1\\f$ (stored in row-major order)"]
                #[doc = " @param in2 input 3x3 matrix \\f$\\mathbf{M}_2\\f$ (stored in row-major order)"]
                #[doc = " @param out output 3x3 matrix \\f$\\mathbf{M}_\\mathrm{out}\\f$ (stored in row-major order)"]
                pub unsafe fn matrix3Multiply(in1: *const f64, in2: *const f64, out: *mut f64) {
                    cxxbridge::matrix3Multiply_autocxx_wrapper(in1, in2, out)
                }
                #[doc = " @brief Performs 3x3 matrix multiplication after transposing the first matrix"]
                #[doc = " (\\f$\\mathbf{M}_\\mathrm{out} = \\mathbf{M}_1^T\\mathbf{M}_2\\f$)."]
                #[doc = " @param in1 input 3x3 matrix \\f$\\mathbf{M}_1\\f$ (stored in row-major order)"]
                #[doc = " @param in2 input 3x3 matrix \\f$\\mathbf{M}_2\\f$ (stored in row-major order)"]
                #[doc = " @param out output 3x3 matrix \\f$\\mathbf{M}_\\mathrm{out}\\f$ (stored in row-major order)"]
                pub unsafe fn matrix3MultiplyTpsFirst(
                    in1: *const f64,
                    in2: *const f64,
                    out: *mut f64,
                ) {
                    cxxbridge::matrix3MultiplyTpsFirst_autocxx_wrapper(in1, in2, out)
                }
                #[doc = " @brief Performs 3x3 matrix multiplication after transposing the second matrix"]
                #[doc = " (\\f$\\mathbf{M}_\\mathrm{out} = \\mathbf{M}_1\\mathbf{M}_2^T\\f$)."]
                #[doc = " @param in1 input 3x3 matrix \\f$\\mathbf{M}_1\\f$ (stored in row-major order)"]
                #[doc = " @param in2 input 3x3 matrix \\f$\\mathbf{M}_2\\f$ (stored in row-major order)"]
                #[doc = " @param out output 3x3 matrix \\f$\\mathbf{M}_\\mathrm{out}\\f$ (stored in row-major order)"]
                pub unsafe fn matrix3MultiplyTpsSecond(
                    in1: *const f64,
                    in2: *const f64,
                    out: *mut f64,
                ) {
                    cxxbridge::matrix3MultiplyTpsSecond_autocxx_wrapper(in1, in2, out)
                }
                #[doc = " @brief Calculates the inverse of a 3x3 matrix (\\f$\\mathbf{M}_\\mathrm{out} = \\mathbf{M}^{-1}\\f$)."]
                #[doc = " @param in input 3x3 matrix \\f$\\mathbf{M}\\f$ (stored in row-major order)"]
                #[doc = " @param out output 3x3 matrix \\f$\\mathbf{M}_\\mathrm{out}\\f$ (stored in row-major order)"]
                pub unsafe fn matrix3Inv(in_: *const f64, out: *mut f64) -> bool {
                    cxxbridge::matrix3Inv_autocxx_wrapper(in_, out)
                }
                #[doc = " Initializes the object with default parameters."]
                #[doc = ""]
                #[doc = " In the most common case (using the default parameters and all data being sampled with the same frequency,"]
                #[doc = " create the class like this:"]
                #[doc = " \\rst"]
                #[doc = " .. code-block:: c++"]
                #[doc = ""]
                #[doc = "     VQF vqf(0.01); // 0.01 s sampling time, i.e. 100 Hz"]
                #[doc = " \\endrst"]
                #[doc = ""]
                #[doc = " @param gyrTs sampling time of the gyroscope measurements in seconds"]
                #[doc = " @param accTs sampling time of the accelerometer measurements in seconds (the value of `gyrTs` is used if set to -1)"]
                #[doc = " @param magTs sampling time of the magnetometer measurements in seconds (the value of `gyrTs` is used if set to -1)"]
                #[doc = ""]
                pub fn new(
                    gyrTs: f64,
                    accTs: f64,
                    magTs: f64,
                ) -> impl autocxx::moveit::new::New<Output = Self> {
                    unsafe {
                        autocxx::moveit::new::by_raw(move |this| {
                            let this = this.get_unchecked_mut().as_mut_ptr();
                            cxxbridge::VQF_new_autocxx_autocxx_wrapper(this, gyrTs, accTs, magTs)
                        })
                    }
                }
                #[doc = " @brief Initializes the object with custom parameters."]
                #[doc = ""]
                #[doc = " Example code to create an object with magnetic disturbance rejection disabled:"]
                #[doc = " \\rst"]
                #[doc = " .. code-block:: c++"]
                #[doc = ""]
                #[doc = "     VQFParams params;"]
                #[doc = "     params.magDistRejectionEnabled = false;"]
                #[doc = "     VQF vqf(0.01); // 0.01 s sampling time, i.e. 100 Hz"]
                #[doc = " \\endrst"]
                #[doc = ""]
                #[doc = " @param params VQFParams struct containing the desired parameters"]
                #[doc = " @param gyrTs sampling time of the gyroscope measurements in seconds"]
                #[doc = " @param accTs sampling time of the accelerometer measurements in seconds (the value of `gyrTs` is used if set to -1)"]
                #[doc = " @param magTs sampling time of the magnetometer measurements in seconds (the value of `gyrTs` is used if set to -1)"]
                pub fn new1<'a>(
                    params: &'a root::VQFParams,
                    gyrTs: f64,
                    accTs: f64,
                    magTs: f64,
                ) -> impl autocxx::moveit::new::New<Output = Self> + 'a {
                    unsafe {
                        autocxx::moveit::new::by_raw(move |this| {
                            let this = this.get_unchecked_mut().as_mut_ptr();
                            cxxbridge::new1_autocxx_wrapper(this, params, gyrTs, accTs, magTs)
                        })
                    }
                }
            }
            impl VQFState {
                #[doc = "Synthesized default constructor."]
                pub fn new() -> impl autocxx::moveit::new::New<Output = Self> {
                    unsafe {
                        autocxx::moveit::new::by_raw(move |this| {
                            let this = this.get_unchecked_mut().as_mut_ptr();
                            cxxbridge::VQFState_new_autocxx_autocxx_wrapper(this)
                        })
                    }
                }
            }
            impl VQFCoefficients {
                #[doc = "Synthesized default constructor."]
                pub fn new() -> impl autocxx::moveit::new::New<Output = Self> {
                    unsafe {
                        autocxx::moveit::new::by_raw(move |this| {
                            let this = this.get_unchecked_mut().as_mut_ptr();
                            cxxbridge::VQFCoefficients_new_autocxx_autocxx_wrapper(this)
                        })
                    }
                }
            }
            unsafe impl autocxx::moveit::new::MoveNew for root::VQFParams {
                #[doc = "Synthesized move constructor."]
                unsafe fn move_new(
                    mut other: ::core::pin::Pin<autocxx::moveit::MoveRef<'_, root::VQFParams>>,
                    this: ::core::pin::Pin<&mut ::core::mem::MaybeUninit<root::VQFParams>>,
                ) {
                    cxxbridge::new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                        this.get_unchecked_mut().as_mut_ptr(),
                        {
                            let r: &mut _ = ::core::pin::Pin::into_inner_unchecked(other.as_mut());
                            r
                        },
                    )
                }
            }
            unsafe impl autocxx::moveit::new::CopyNew for root::VQFParams {
                #[doc = "Synthesized copy constructor."]
                unsafe fn copy_new(
                    other: &root::VQFParams,
                    this: ::core::pin::Pin<&mut ::core::mem::MaybeUninit<root::VQFParams>>,
                ) {
                    cxxbridge::new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                        this.get_unchecked_mut().as_mut_ptr(),
                        other,
                    )
                }
            }
            impl Drop for root::VQFParams {
                #[doc = "Synthesized destructor."]
                fn drop(self: &mut root::VQFParams) {
                    unsafe {
                        cxxbridge::VQFParams_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(
                            self,
                        )
                    }
                }
            }
            unsafe impl autocxx::moveit::new::MoveNew for root::VQFState {
                #[doc = "Synthesized move constructor."]
                unsafe fn move_new(
                    mut other: ::core::pin::Pin<autocxx::moveit::MoveRef<'_, root::VQFState>>,
                    this: ::core::pin::Pin<&mut ::core::mem::MaybeUninit<root::VQFState>>,
                ) {
                    cxxbridge::VQFState_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                        this.get_unchecked_mut().as_mut_ptr(),
                        {
                            let r: &mut _ = ::core::pin::Pin::into_inner_unchecked(other.as_mut());
                            r
                        },
                    )
                }
            }
            unsafe impl autocxx::moveit::new::CopyNew for root::VQFState {
                #[doc = "Synthesized copy constructor."]
                unsafe fn copy_new(
                    other: &root::VQFState,
                    this: ::core::pin::Pin<&mut ::core::mem::MaybeUninit<root::VQFState>>,
                ) {
                    cxxbridge :: VQFState_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper (this . get_unchecked_mut () . as_mut_ptr () , other)
                }
            }
            impl Drop for root::VQFState {
                #[doc = "Synthesized destructor."]
                fn drop(self: &mut root::VQFState) {
                    unsafe {
                        cxxbridge::VQFState_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(
                            self,
                        )
                    }
                }
            }
            unsafe impl autocxx::moveit::new::MoveNew for root::VQFCoefficients {
                #[doc = "Synthesized move constructor."]
                unsafe fn move_new(
                    mut other: ::core::pin::Pin<autocxx::moveit::MoveRef<'_, root::VQFCoefficients>>,
                    this: ::core::pin::Pin<&mut ::core::mem::MaybeUninit<root::VQFCoefficients>>,
                ) {
                    cxxbridge :: VQFCoefficients_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper (this . get_unchecked_mut () . as_mut_ptr () , { let r : & mut _ = :: std :: pin :: Pin :: into_inner_unchecked (other . as_mut ()) ; r })
                }
            }
            unsafe impl autocxx::moveit::new::CopyNew for root::VQFCoefficients {
                #[doc = "Synthesized copy constructor."]
                unsafe fn copy_new(
                    other: &root::VQFCoefficients,
                    this: ::core::pin::Pin<&mut ::core::mem::MaybeUninit<root::VQFCoefficients>>,
                ) {
                    cxxbridge :: VQFCoefficients_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper (this . get_unchecked_mut () . as_mut_ptr () , other)
                }
            }
            impl Drop for root::VQFCoefficients {
                #[doc = "Synthesized destructor."]
                fn drop(self: &mut root::VQFCoefficients) {
                    unsafe {
                        cxxbridge :: VQFCoefficients_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper (self)
                    }
                }
            }
            unsafe impl autocxx::moveit::new::MoveNew for root::VQF {
                #[doc = "Synthesized move constructor."]
                unsafe fn move_new(
                    mut other: ::core::pin::Pin<autocxx::moveit::MoveRef<'_, root::VQF>>,
                    this: ::core::pin::Pin<&mut ::core::mem::MaybeUninit<root::VQF>>,
                ) {
                    cxxbridge::VQF_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                        this.get_unchecked_mut().as_mut_ptr(),
                        {
                            let r: &mut _ = ::core::pin::Pin::into_inner_unchecked(other.as_mut());
                            r
                        },
                    )
                }
            }
            unsafe impl autocxx::moveit::new::CopyNew for root::VQF {
                #[doc = "Synthesized copy constructor."]
                unsafe fn copy_new(
                    other: &root::VQF,
                    this: ::core::pin::Pin<&mut ::core::mem::MaybeUninit<root::VQF>>,
                ) {
                    cxxbridge::VQF_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                        this.get_unchecked_mut().as_mut_ptr(),
                        other,
                    )
                }
            }
            impl Drop for root::VQF {
                #[doc = "Synthesized destructor."]
                fn drop(self: &mut root::VQF) {
                    unsafe {
                        cxxbridge::VQF_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(self)
                    }
                }
            }
            #[allow(unused_imports)]
            use self::super::super::cxxbridge;
            #[allow(unused_imports)]
            use self::super::root;
        }
    }
    #[cxx::bridge]
    mod cxxbridge {
        unsafe extern "C++" {
            #[doc = " @brief Struct containing all tuning parameters used by the VQF class."]
            #[doc = ""]
            #[doc = " The parameters influence the behavior of the algorithm and are independent of the sampling rate of the IMU data. The"]
            #[doc = " constructor sets all parameters to the default values."]
            #[doc = ""]
            #[doc = " The parameters #motionBiasEstEnabled, #restBiasEstEnabled, and #magDistRejectionEnabled can be used to enable/disable"]
            #[doc = " the main features of the VQF algorithm. The time constants #tauAcc and #tauMag can be tuned to change the trust on"]
            #[doc = " the accelerometer and magnetometer measurements, respectively. The remaining parameters influence bias estimation"]
            #[doc = " and magnetometer rejection."]
            type VQFParams = super::bindgen::root::VQFParams;
            #[doc = " @brief Struct containing the filter state of the VQF class."]
            #[doc = ""]
            #[doc = " The relevant parts of the state can be accessed via functions of the VQF class, e.g. VQF::getQuat6D(),"]
            #[doc = " VQF::getQuat9D(), VQF::getGyrBiasEstimate(), VQF::setGyrBiasEstimate(), VQF::getRestDetected() and"]
            #[doc = " VQF::getMagDistDetected(). To reset the state to the initial values, use VQF::resetState()."]
            #[doc = ""]
            #[doc = " Direct access to the full state is typically not needed but can be useful in some cases, e.g. for debugging. For this"]
            #[doc = " purpose, the state can be accessed by VQF::getState() and set by VQF::setState()."]
            type VQFState = super::bindgen::root::VQFState;
            #[doc = " @brief Struct containing coefficients used by the VQF class."]
            #[doc = ""]
            #[doc = " Coefficients are values that depend on the parameters and the sampling times, but do not change during update steps."]
            #[doc = " They are calculated in VQF::setup()."]
            type VQFCoefficients = super::bindgen::root::VQFCoefficients;
            #[doc = " @brief A Versatile Quaternion-based Filter for IMU Orientation Estimation."]
            #[doc = ""]
            #[doc = " \\rst"]
            #[doc = " This class implements the orientation estimation filter described in the following publication:"]
            #[doc = ""]
            #[doc = ""]
            #[doc = "     D. Laidig, T. Seel. \"VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic"]
            #[doc = "     Disturbance Rejection.\" arXiv preprint, 2022. `arXiv:2203.17024 <https://arxiv.org/abs/2203.17024>`_."]
            #[doc = ""]
            #[doc = " The filter can perform simultaneous 6D (magnetometer-free) and 9D (gyr+acc+mag) sensor fusion and can also be used"]
            #[doc = " without magnetometer data. It performs rest detection, gyroscope bias estimation during rest and motion, and magnetic"]
            #[doc = " disturbance detection and rejection. Different sampling rates for gyroscopes, accelerometers, and magnetometers are"]
            #[doc = " supported as well. While in most cases, the defaults will be reasonable, the algorithm can be influenced via a"]
            #[doc = " number of tuning parameters."]
            #[doc = ""]
            #[doc = " To use this C++ implementation,"]
            #[doc = ""]
            #[doc = " 1. create a instance of the class and provide the sampling time and, optionally, parameters"]
            #[doc = " 2. for every sample, call one of the update functions to feed the algorithm with IMU data"]
            #[doc = " 3. access the estimation results with :meth:`getQuat6D() <VQF.getQuat6D>`, :meth:`getQuat9D() <VQF.getQuat9D>` and"]
            #[doc = "    the other getter methods."]
            #[doc = ""]
            #[doc = " If the full data is available in (row-major) data buffers, you can use :meth:`updateBatch() <VQF.updateBatch>`."]
            #[doc = ""]
            #[doc = " This class is the main C++ implementation of the algorithm. Depending on use case and programming language of choice,"]
            #[doc = " the following alternatives might be useful:"]
            #[doc = ""]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " |                        | Full Version             | Basic Version            | Offline Version           |"]
            #[doc = " |                        |                          |                          |                           |"]
            #[doc = " +========================+==========================+==========================+===========================+"]
            #[doc = " | **C++**                | **VQF (this class)**     | :cpp:class:`BasicVQF`    | :cpp:func:`offlineVQF`    |"]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " | **Python/C++ (fast)**  | :py:class:`vqf.VQF`      | :py:class:`vqf.BasicVQF` | :py:meth:`vqf.offlineVQF` |"]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " | **Pure Python (slow)** | :py:class:`vqf.PyVQF`    | --                       | --                        |"]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " | **Pure Matlab (slow)** | :mat:class:`VQF.m <VQF>` | --                       | --                        |"]
            #[doc = " +------------------------+--------------------------+--------------------------+---------------------------+"]
            #[doc = " \\endrst"]
            type VQF = super::bindgen::root::VQF;
            #[doc = " @brief Constructor that initializes the struct with the default parameters."]
            pub unsafe fn new_autocxx_autocxx_wrapper(autocxx_gen_this: *mut VQFParams);
            #[doc = " @brief Performs gyroscope update step."]
            #[doc = ""]
            #[doc = " It is only necessary to call this function directly if gyroscope, accelerometers and magnetometers have"]
            #[doc = " different sampling rates. Otherwise, simply use #update()."]
            #[doc = ""]
            #[doc = " @param gyr gyroscope measurement in rad/s"]
            pub unsafe fn updateGyr(self: Pin<&mut VQF>, gyr: *const f64);
            #[doc = " @brief Performs accelerometer update step."]
            #[doc = ""]
            #[doc = " It is only necessary to call this function directly if gyroscope, accelerometers and magnetometers have"]
            #[doc = " different sampling rates. Otherwise, simply use #update()."]
            #[doc = ""]
            #[doc = " Should be called after #updateGyr and before #updateMag."]
            #[doc = ""]
            #[doc = " @param acc accelerometer measurement in m/s²"]
            pub unsafe fn updateAcc(self: Pin<&mut VQF>, acc: *const f64);
            #[doc = " @brief Performs magnetometer update step."]
            #[doc = ""]
            #[doc = " It is only necessary to call this function directly if gyroscope, accelerometers and magnetometers have"]
            #[doc = " different sampling rates. Otherwise, simply use #update()."]
            #[doc = ""]
            #[doc = " Should be called after #updateAcc."]
            #[doc = ""]
            #[doc = " @param mag magnetometer measurement in arbitrary units"]
            pub unsafe fn updateMag(self: Pin<&mut VQF>, mag: *const f64);
            #[doc = " @brief Performs filter update step for one sample (magnetometer-free)."]
            #[doc = " @param gyr gyroscope measurement in rad/s"]
            #[doc = " @param acc accelerometer measurement in m/s²"]
            pub unsafe fn update(self: Pin<&mut VQF>, gyr: *const f64, acc: *const f64);
            #[cxx_name = "update"]
            #[doc = " @brief Performs filter update step for one sample (with magnetometer measurement)."]
            #[doc = " @param gyr gyroscope measurement in rad/s"]
            #[doc = " @param acc accelerometer measurement in m/s²"]
            #[doc = " @param mag magnetometer measurement in arbitrary units"]
            pub unsafe fn update1(
                self: Pin<&mut VQF>,
                gyr: *const f64,
                acc: *const f64,
                mag: *const f64,
            );
            #[doc = " @brief Performs batch update for multiple samples at once."]
            #[doc = ""]
            #[doc = " In order to use this function, the input data must be available in an array in row-major order. A null pointer"]
            #[doc = " can be passed for mag in order to skip the magnetometer update. All data must have the same sampling rate."]
            #[doc = ""]
            #[doc = " The output pointer arguments must be null pointers or point to sufficiently large data buffers."]
            #[doc = ""]
            #[doc = " Example usage:"]
            #[doc = " \\rst"]
            #[doc = " .. code-block:: c++"]
            #[doc = ""]
            #[doc = "     int N = 1000;"]
            #[doc = "     double gyr[N*3]; // fill with gyroscope measurements in rad/s"]
            #[doc = "     double acc[N*3]; // fill will accelerometer measurements in m/s²"]
            #[doc = "     double mag[N*3]; // fill with magnetometer measurements (arbitrary units)"]
            #[doc = "     double quat9D[N*4]; // output buffer"]
            #[doc = ""]
            #[doc = "     VQF vqf(0.01); // 0.01 s sampling time, i.e. 100 Hz"]
            #[doc = "     vqf.updateBatch(gyr, acc, mag, nullptr, quat9D, nullptr, nullptr, nullptr, nullptr, nullptr);"]
            #[doc = " \\endrst"]
            #[doc = ""]
            #[doc = " @param gyr gyroscope measurement in rad/s (N*3 elements, must not be null)"]
            #[doc = " @param acc accelerometer measurement in m/s² (N*3 elements, must not be null)"]
            #[doc = " @param mag magnetometer measurement in arbitrary units (N*3 elements, can be a null pointer)"]
            #[doc = " @param N number of samples"]
            #[doc = " @param out6D output buffer for the 6D quaternion (N*4 elements, can be a null pointer)"]
            #[doc = " @param out9D output buffer for the 9D quaternion (N*4 elements, can be a null pointer)"]
            #[doc = " @param outDelta output buffer for heading difference angle (N elements, can be a null pointer)"]
            #[doc = " @param outBias output buffer for the gyroscope bias estimate (N*3 elements, can be a null pointer)"]
            #[doc = " @param outBiasSigma output buffer for the bias estimation uncertainty (N elements, can be a null pointer)"]
            #[doc = " @param outRest output buffer for the rest detection state (N elements, can be a null pointer)"]
            #[doc = " @param outMagDist output buffer for the magnetic disturbance state (N elements, can be a null pointer)"]
            pub unsafe fn updateBatch(
                self: Pin<&mut VQF>,
                gyr: *const f64,
                acc: *const f64,
                mag: *const f64,
                N: usize,
                out6D: *mut f64,
                out9D: *mut f64,
                outDelta: *mut f64,
                outBias: *mut f64,
                outBiasSigma: *mut f64,
                outRest: *mut bool,
                outMagDist: *mut bool,
            );
            #[doc = " @brief Returns the angular velocity strapdown integration quaternion"]
            #[doc = " \\f$^{\\mathcal{S}_i}_{\\mathcal{I}_i}\\mathbf{q}\\f$."]
            #[doc = " @param out output array for the quaternion"]
            pub unsafe fn getQuat3D(self: &VQF, out: *mut f64);
            #[doc = " @brief Returns the 6D (magnetometer-free) orientation quaternion"]
            #[doc = " \\f$^{\\mathcal{S}_i}_{\\mathcal{E}_i}\\mathbf{q}\\f$."]
            #[doc = " @param out output array for the quaternion"]
            pub unsafe fn getQuat6D(self: &VQF, out: *mut f64);
            #[doc = " @brief Returns the 9D (with magnetometers) orientation quaternion"]
            #[doc = " \\f$^{\\mathcal{S}_i}_{\\mathcal{E}}\\mathbf{q}\\f$."]
            #[doc = " @param out output array for the quaternion"]
            pub unsafe fn getQuat9D(self: &VQF, out: *mut f64);
            #[doc = " @brief Returns the heading difference \\f$\\delta\\f$ between \\f$\\mathcal{E}_i\\f$ and \\f$\\mathcal{E}\\f$."]
            #[doc = ""]
            #[doc = " \\f$^{\\mathcal{E}_i}_{\\mathcal{E}}\\mathbf{q} = \\begin{bmatrix}\\cos\\frac{\\delta}{2} & 0 & 0 &"]
            #[doc = " \\sin\\frac{\\delta}{2}\\end{bmatrix}^T\\f$."]
            #[doc = ""]
            #[doc = " @return delta angle in rad (VQFState::delta)"]
            pub fn getDelta(self: &VQF) -> f64;
            #[doc = " @brief Returns the current gyroscope bias estimate and the uncertainty."]
            #[doc = ""]
            #[doc = " The returned standard deviation sigma represents the estimation uncertainty in the worst direction and is based"]
            #[doc = " on an upper bound of the largest eigenvalue of the covariance matrix."]
            #[doc = ""]
            #[doc = " @param out output array for the gyroscope bias estimate (rad/s)"]
            #[doc = " @return standard deviation sigma of the estimation uncertainty (rad/s)"]
            pub unsafe fn getBiasEstimate(self: &VQF, out: *mut f64) -> f64;
            #[doc = " @brief Sets the current gyroscope bias estimate and the uncertainty."]
            #[doc = ""]
            #[doc = " If a value for the uncertainty sigma is given, the covariance matrix is set to a corresponding scaled identity"]
            #[doc = " matrix."]
            #[doc = ""]
            #[doc = " @param bias gyroscope bias estimate (rad/s)"]
            #[doc = " @param sigma standard deviation of the estimation uncertainty (rad/s) - set to -1 (default) in order to not"]
            #[doc = "        change the estimation covariance matrix"]
            pub unsafe fn setBiasEstimate(self: Pin<&mut VQF>, bias: *mut f64, sigma: f64);
            #[doc = " @brief Returns true if rest was detected."]
            pub fn getRestDetected(self: &VQF) -> bool;
            #[doc = " @brief Returns true if a disturbed magnetic field was detected."]
            pub fn getMagDistDetected(self: &VQF) -> bool;
            #[doc = " @brief Returns the relative deviations used in rest detection."]
            #[doc = ""]
            #[doc = " Looking at those values can be useful to understand how rest detection is working and which thresholds are"]
            #[doc = " suitable. The output array is filled with the last values for gyroscope and accelerometer,"]
            #[doc = " relative to the threshold. In order for rest to be detected, both values must stay below 1."]
            #[doc = ""]
            #[doc = " @param out output array of size 2 for the relative rest deviations"]
            pub unsafe fn getRelativeRestDeviations(self: &VQF, out: *mut f64);
            #[doc = " @brief Returns the norm of the currently accepted magnetic field reference."]
            pub fn getMagRefNorm(self: &VQF) -> f64;
            #[doc = " @brief Returns the dip angle of the currently accepted magnetic field reference."]
            pub fn getMagRefDip(self: &VQF) -> f64;
            #[doc = " @brief Overwrites the current magnetic field reference."]
            #[doc = " @param norm norm of the magnetic field reference"]
            #[doc = " @param dip dip angle of the magnetic field reference"]
            pub fn setMagRef(self: Pin<&mut VQF>, norm: f64, dip: f64);
            #[doc = " @brief Sets the time constant for accelerometer low-pass filtering."]
            #[doc = ""]
            #[doc = " For more details, see VQFParams.tauAcc."]
            #[doc = ""]
            #[doc = " @param tauAcc time constant \\f$\\tau_\\mathrm{acc}\\f$ in seconds"]
            pub fn setTauAcc(self: Pin<&mut VQF>, tauAcc: f64);
            #[doc = " @brief Sets the time constant for the magnetometer update."]
            #[doc = ""]
            #[doc = " For more details, see VQFParams.tauMag."]
            #[doc = ""]
            #[doc = " @param tauMag time constant \\f$\\tau_\\mathrm{mag}\\f$ in seconds"]
            pub fn setTauMag(self: Pin<&mut VQF>, tauMag: f64);
            #[doc = " @brief Enables/disabled gyroscope bias estimation during motion."]
            pub fn setMotionBiasEstEnabled(self: Pin<&mut VQF>, enabled: bool);
            #[doc = " @brief Enables/disables rest detection and bias estimation during rest."]
            pub fn setRestBiasEstEnabled(self: Pin<&mut VQF>, enabled: bool);
            #[doc = " @brief Enables/disables magnetic disturbance detection and rejection."]
            pub fn setMagDistRejectionEnabled(self: Pin<&mut VQF>, enabled: bool);
            #[doc = " @brief Sets the current thresholds for rest detection."]
            #[doc = ""]
            #[doc = " For details about the parameters, see VQFParams.restThGyr and VQFParams.restThAcc."]
            pub fn setRestDetectionThresholds(self: Pin<&mut VQF>, thGyr: f64, thAcc: f64);
            #[doc = " @brief Returns the current parameters."]
            pub fn getParams(self: &VQF) -> &VQFParams;
            #[doc = " @brief Returns the coefficients used by the algorithm."]
            pub fn getCoeffs(self: &VQF) -> &VQFCoefficients;
            #[doc = " @brief Returns the current state."]
            pub fn getState(self: &VQF) -> &VQFState;
            #[doc = " @brief Overwrites the current state."]
            #[doc = ""]
            #[doc = " This method allows to set a completely arbitrary filter state and is intended for debugging purposes. In"]
            #[doc = " combination with #getState, individual elements of the state can be modified."]
            #[doc = ""]
            #[doc = " @param state A VQFState struct containing the new state"]
            pub fn setState(self: Pin<&mut VQF>, state: &VQFState);
            #[doc = " @brief Resets the state to the default values at initialization."]
            #[doc = ""]
            #[doc = " Resetting the state is equivalent to creating a new instance of this class."]
            pub fn resetState(self: Pin<&mut VQF>);
            #[doc = " @brief Performs quaternion multiplication (\\f$\\mathbf{q}_\\mathrm{out} = \\mathbf{q}_1 \\otimes \\mathbf{q}_2\\f$)."]
            pub unsafe fn quatMultiply_autocxx_wrapper(
                q1: *const f64,
                q2: *const f64,
                out: *mut f64,
            );
            #[doc = " @brief Calculates the quaternion conjugate (\\f$\\mathbf{q}_\\mathrm{out} = \\mathbf{q}^*\\f$)."]
            pub unsafe fn quatConj_autocxx_wrapper(q: *const f64, out: *mut f64);
            #[doc = " @brief Sets the output quaternion to the identity quaternion (\\f$\\mathbf{q}_\\mathrm{out} ="]
            #[doc = " \\begin{bmatrix}1 & 0 & 0 & 0\\end{bmatrix}\\f$)."]
            pub unsafe fn quatSetToIdentity_autocxx_wrapper(out: *mut f64);
            #[doc = " @brief Applies a heading rotation by the angle delta (in rad) to a quaternion."]
            #[doc = ""]
            #[doc = " \\f$\\mathbf{q}_\\mathrm{out} = \\begin{bmatrix}\\cos\\frac{\\delta}{2} & 0 & 0 &"]
            #[doc = " \\sin\\frac{\\delta}{2}\\end{bmatrix} \\otimes \\mathbf{q}\\f$"]
            pub unsafe fn quatApplyDelta_autocxx_wrapper(q: *mut f64, delta: f64, out: *mut f64);
            #[doc = " @brief Rotates a vector with a given quaternion."]
            #[doc = ""]
            #[doc = " \\f$\\begin{bmatrix}0 & \\mathbf{v}_\\mathrm{out}\\end{bmatrix} ="]
            #[doc = " \\mathbf{q} \\otimes \\begin{bmatrix}0 & \\mathbf{v}\\end{bmatrix} \\otimes \\mathbf{q}^*\\f$"]
            pub unsafe fn quatRotate_autocxx_wrapper(q: *const f64, v: *const f64, out: *mut f64);
            #[doc = " @brief Calculates the Euclidean norm of a vector."]
            #[doc = " @param vec pointer to an array of N elements"]
            #[doc = " @param N number of elements"]
            pub unsafe fn norm_autocxx_wrapper(vec: *const f64, N: usize) -> f64;
            #[doc = " @brief Normalizes a vector in-place."]
            #[doc = " @param vec pointer to an array of N elements that will be normalized"]
            #[doc = " @param N number of elements"]
            pub unsafe fn normalize_autocxx_wrapper(vec: *mut f64, N: usize);
            #[doc = " @brief Clips a vector in-place."]
            #[doc = " @param vec pointer to an array of N elements that will be clipped"]
            #[doc = " @param N number of elements"]
            #[doc = " @param min smallest allowed value"]
            #[doc = " @param max largest allowed value"]
            pub unsafe fn clip_autocxx_wrapper(vec: *mut f64, N: usize, min: f64, max: f64);
            #[doc = " @brief Calculates the gain for a first-order low-pass filter from the 1/e time constant."]
            #[doc = ""]
            #[doc = " \\f$k = 1 - \\exp\\left(-\\frac{T_\\mathrm{s}}{\\tau}\\right)\\f$"]
            #[doc = ""]
            #[doc = " The cutoff frequency of the resulting filter is \\f$f_\\mathrm{c} = \\frac{1}{2\\pi\\tau}\\f$."]
            #[doc = ""]
            #[doc = " @param tau time constant \\f$\\tau\\f$ in seconds - use -1 to disable update (\\f$k=0\\f$) or 0 to obtain"]
            #[doc = "        unfiltered values (\\f$k=1\\f$)"]
            #[doc = " @param Ts sampling time \\f$T_\\mathrm{s}\\f$ in seconds"]
            #[doc = " @return filter gain *k*"]
            pub fn gainFromTau_autocxx_wrapper(tau: f64, Ts: f64) -> f64;
            #[doc = " @brief Calculates coefficients for a second-order Butterworth low-pass filter."]
            #[doc = ""]
            #[doc = " The filter is parametrized via the time constant of the dampened, non-oscillating part of step response and the"]
            #[doc = " resulting cutoff frequency is \\f$f_\\mathrm{c} = \\frac{\\sqrt{2}}{2\\pi\\tau}\\f$."]
            #[doc = ""]
            #[doc = " @param tau time constant \\f$\\tau\\f$ in seconds"]
            #[doc = " @param Ts sampling time \\f$T_\\mathrm{s}\\f$ in seconds"]
            #[doc = " @param outB output array for numerator coefficients"]
            #[doc = " @param outA output array for denominator coefficients (without \\f$a_0=1\\f$)"]
            pub unsafe fn filterCoeffs_autocxx_wrapper(
                tau: f64,
                Ts: f64,
                outB: *mut f64,
                outA: *mut f64,
            );
            #[doc = " @brief Calculates the initial filter state for a given steady-state value."]
            #[doc = " @param x0 steady state value"]
            #[doc = " @param b numerator coefficients"]
            #[doc = " @param a denominator coefficients (without \\f$a_0=1\\f$)"]
            #[doc = " @param out output array for filter state"]
            pub unsafe fn filterInitialState_autocxx_wrapper(
                x0: f64,
                b: *const f64,
                a: *const f64,
                out: *mut f64,
            );
            #[doc = " @brief Adjusts the filter state when changing coefficients."]
            #[doc = ""]
            #[doc = " This function assumes that the filter is currently in a steady state, i.e. the last input values and the last"]
            #[doc = " output values are all equal. Based on this, the filter state is adjusted to new filter coefficients so that the"]
            #[doc = " output does not jump."]
            #[doc = ""]
            #[doc = " @param last_y last filter output values (array of size N)"]
            #[doc = " @param N number of values in vector-valued signal"]
            #[doc = " @param b_old previous numerator coefficients"]
            #[doc = " @param a_old previous denominator coefficients (without \\f$a_0=1\\f$)"]
            #[doc = " @param b_new new numerator coefficients"]
            #[doc = " @param a_new new denominator coefficients (without \\f$a_0=1\\f$)"]
            #[doc = " @param state filter state (array of size N*2, will be modified)"]
            pub unsafe fn filterAdaptStateForCoeffChange_autocxx_wrapper(
                last_y: *mut f64,
                N: usize,
                b_old: *const f64,
                a_old: *const f64,
                b_new: *const f64,
                a_new: *const f64,
                state: *mut f64,
            );
            #[doc = " @brief Performs a filter step for a scalar value."]
            #[doc = " @param x input value"]
            #[doc = " @param b numerator coefficients"]
            #[doc = " @param a denominator coefficients (without \\f$a_0=1\\f$)"]
            #[doc = " @param state filter state array (will be modified)"]
            #[doc = " @return filtered value"]
            pub unsafe fn filterStep_autocxx_wrapper(
                x: f64,
                b: *const f64,
                a: *const f64,
                state: *mut f64,
            ) -> f64;
            #[doc = " @brief Performs filter step for vector-valued signal with averaging-based initialization."]
            #[doc = ""]
            #[doc = " During the first \\f$\\tau\\f$ seconds, the filter output is the mean of the previous samples. At \\f$t=\\tau\\f$, the"]
            #[doc = " initial conditions for the low-pass filter are calculated based on the current mean value and from then on,"]
            #[doc = " regular filtering with the rational transfer function described by the coefficients b and a is performed."]
            #[doc = ""]
            #[doc = " @param x input values (array of size N)"]
            #[doc = " @param N number of values in vector-valued signal"]
            #[doc = " @param tau filter time constant \\f$\\tau\\f$ in seconds (used for initialization)"]
            #[doc = " @param Ts sampling time \\f$T_\\mathrm{s}\\f$ in seconds (used for initialization)"]
            #[doc = " @param b numerator coefficients"]
            #[doc = " @param a denominator coefficients (without \\f$a_0=1\\f$)"]
            #[doc = " @param state filter state (array of size N*2, will be modified)"]
            #[doc = " @param out output array for filtered values (size N)"]
            pub unsafe fn filterVec_autocxx_wrapper(
                x: *const f64,
                N: usize,
                tau: f64,
                Ts: f64,
                b: *const f64,
                a: *const f64,
                state: *mut f64,
                out: *mut f64,
            );
            #[doc = " @brief Sets a 3x3 matrix to a scaled version of the identity matrix."]
            #[doc = " @param scale value of diagonal elements"]
            #[doc = " @param out output array of size 9 (3x3 matrix stored in row-major order)"]
            pub unsafe fn matrix3SetToScaledIdentity_autocxx_wrapper(scale: f64, out: *mut f64);
            #[doc = " @brief Performs 3x3 matrix multiplication (\\f$\\mathbf{M}_\\mathrm{out} = \\mathbf{M}_1\\mathbf{M}_2\\f$)."]
            #[doc = " @param in1 input 3x3 matrix \\f$\\mathbf{M}_1\\f$ (stored in row-major order)"]
            #[doc = " @param in2 input 3x3 matrix \\f$\\mathbf{M}_2\\f$ (stored in row-major order)"]
            #[doc = " @param out output 3x3 matrix \\f$\\mathbf{M}_\\mathrm{out}\\f$ (stored in row-major order)"]
            pub unsafe fn matrix3Multiply_autocxx_wrapper(
                in1: *const f64,
                in2: *const f64,
                out: *mut f64,
            );
            #[doc = " @brief Performs 3x3 matrix multiplication after transposing the first matrix"]
            #[doc = " (\\f$\\mathbf{M}_\\mathrm{out} = \\mathbf{M}_1^T\\mathbf{M}_2\\f$)."]
            #[doc = " @param in1 input 3x3 matrix \\f$\\mathbf{M}_1\\f$ (stored in row-major order)"]
            #[doc = " @param in2 input 3x3 matrix \\f$\\mathbf{M}_2\\f$ (stored in row-major order)"]
            #[doc = " @param out output 3x3 matrix \\f$\\mathbf{M}_\\mathrm{out}\\f$ (stored in row-major order)"]
            pub unsafe fn matrix3MultiplyTpsFirst_autocxx_wrapper(
                in1: *const f64,
                in2: *const f64,
                out: *mut f64,
            );
            #[doc = " @brief Performs 3x3 matrix multiplication after transposing the second matrix"]
            #[doc = " (\\f$\\mathbf{M}_\\mathrm{out} = \\mathbf{M}_1\\mathbf{M}_2^T\\f$)."]
            #[doc = " @param in1 input 3x3 matrix \\f$\\mathbf{M}_1\\f$ (stored in row-major order)"]
            #[doc = " @param in2 input 3x3 matrix \\f$\\mathbf{M}_2\\f$ (stored in row-major order)"]
            #[doc = " @param out output 3x3 matrix \\f$\\mathbf{M}_\\mathrm{out}\\f$ (stored in row-major order)"]
            pub unsafe fn matrix3MultiplyTpsSecond_autocxx_wrapper(
                in1: *const f64,
                in2: *const f64,
                out: *mut f64,
            );
            #[doc = " @brief Calculates the inverse of a 3x3 matrix (\\f$\\mathbf{M}_\\mathrm{out} = \\mathbf{M}^{-1}\\f$)."]
            #[doc = " @param in input 3x3 matrix \\f$\\mathbf{M}\\f$ (stored in row-major order)"]
            #[doc = " @param out output 3x3 matrix \\f$\\mathbf{M}_\\mathrm{out}\\f$ (stored in row-major order)"]
            pub unsafe fn matrix3Inv_autocxx_wrapper(in_: *const f64, out: *mut f64) -> bool;
            #[doc = " Initializes the object with default parameters."]
            #[doc = ""]
            #[doc = " In the most common case (using the default parameters and all data being sampled with the same frequency,"]
            #[doc = " create the class like this:"]
            #[doc = " \\rst"]
            #[doc = " .. code-block:: c++"]
            #[doc = ""]
            #[doc = "     VQF vqf(0.01); // 0.01 s sampling time, i.e. 100 Hz"]
            #[doc = " \\endrst"]
            #[doc = ""]
            #[doc = " @param gyrTs sampling time of the gyroscope measurements in seconds"]
            #[doc = " @param accTs sampling time of the accelerometer measurements in seconds (the value of `gyrTs` is used if set to -1)"]
            #[doc = " @param magTs sampling time of the magnetometer measurements in seconds (the value of `gyrTs` is used if set to -1)"]
            #[doc = ""]
            pub unsafe fn VQF_new_autocxx_autocxx_wrapper(
                autocxx_gen_this: *mut VQF,
                gyrTs: f64,
                accTs: f64,
                magTs: f64,
            );
            #[doc = " @brief Initializes the object with custom parameters."]
            #[doc = ""]
            #[doc = " Example code to create an object with magnetic disturbance rejection disabled:"]
            #[doc = " \\rst"]
            #[doc = " .. code-block:: c++"]
            #[doc = ""]
            #[doc = "     VQFParams params;"]
            #[doc = "     params.magDistRejectionEnabled = false;"]
            #[doc = "     VQF vqf(0.01); // 0.01 s sampling time, i.e. 100 Hz"]
            #[doc = " \\endrst"]
            #[doc = ""]
            #[doc = " @param params VQFParams struct containing the desired parameters"]
            #[doc = " @param gyrTs sampling time of the gyroscope measurements in seconds"]
            #[doc = " @param accTs sampling time of the accelerometer measurements in seconds (the value of `gyrTs` is used if set to -1)"]
            #[doc = " @param magTs sampling time of the magnetometer measurements in seconds (the value of `gyrTs` is used if set to -1)"]
            pub unsafe fn new1_autocxx_wrapper(
                autocxx_gen_this: *mut VQF,
                params: &VQFParams,
                gyrTs: f64,
                accTs: f64,
                magTs: f64,
            );
            #[doc = "Synthesized move constructor."]
            pub unsafe fn new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFParams,
                other: *mut VQFParams,
            );
            #[doc = "Synthesized copy constructor."]
            pub unsafe fn new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFParams,
                other: &VQFParams,
            );
            #[doc = "Synthesized destructor."]
            pub unsafe fn VQFParams_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFParams,
            );
            #[doc = "Synthesized default constructor."]
            pub unsafe fn VQFState_new_autocxx_autocxx_wrapper(autocxx_gen_this: *mut VQFState);
            #[doc = "Synthesized move constructor."]
            pub unsafe fn VQFState_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFState,
                other: *mut VQFState,
            );
            #[doc = "Synthesized copy constructor."]
            pub unsafe fn VQFState_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFState,
                other: &VQFState,
            );
            #[doc = "Synthesized destructor."]
            pub unsafe fn VQFState_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFState,
            );
            #[doc = "Synthesized default constructor."]
            pub unsafe fn VQFCoefficients_new_autocxx_autocxx_wrapper(
                autocxx_gen_this: *mut VQFCoefficients,
            );
            #[doc = "Synthesized move constructor."]
            pub unsafe fn VQFCoefficients_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFCoefficients,
                other: *mut VQFCoefficients,
            );
            #[doc = "Synthesized copy constructor."]
            pub unsafe fn VQFCoefficients_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFCoefficients,
                other: &VQFCoefficients,
            );
            #[doc = "Synthesized destructor."]
            pub unsafe fn VQFCoefficients_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQFCoefficients,
            );
            #[doc = "Synthesized move constructor."]
            pub unsafe fn VQF_new_synthetic_move_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQF,
                other: *mut VQF,
            );
            #[doc = "Synthesized copy constructor."]
            pub unsafe fn VQF_new_synthetic_const_copy_ctor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQF,
                other: &VQF,
            );
            #[doc = "Synthesized destructor."]
            pub unsafe fn VQF_synthetic_destructor_0xf714713ddf8d40a8_autocxx_wrapper(
                autocxx_gen_this: *mut VQF,
            );
            include!("vqf-cxx/vqf/vqf/cpp/vqf.hpp");
            include!("vqf-cxx/generated/autocxxgen_ffi.h");
        }
        extern "Rust" {}
    }
    #[allow(unused_imports)]
    use bindgen::root;
    pub use bindgen::root::vqf_real_t;
    pub use bindgen::root::VQFCoefficients;
    pub use bindgen::root::VQFParams;
    pub use bindgen::root::VQFState;
    pub use bindgen::root::VQF;
}
