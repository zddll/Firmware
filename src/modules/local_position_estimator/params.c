#include <systemlib/param/param.h>

// 16 is max name length


/**
 * Enable local position estimatr
 *
 * @group Local Position Estimator
 */
PARAM_DEFINE_INT32(LPE_ENABLED, 0);

/**
 * Enable accelerometer integration for prediction.
 *
 * @group Local Position Estimator
 */
PARAM_DEFINE_FLOAT(LPE_INTEGRATE, 1);

/**
 * Optical flow xy standard deviation
 *
 * @group Local Position Estimator
 */
PARAM_DEFINE_FLOAT(LPE_FLW_XY, 0.01f);

/**
 * Sonar z standard deviation
 *
 * @group Local Position Estimator
 */
PARAM_DEFINE_FLOAT(LPE_SNR_Z, 0.02f);

/**
 * Lidar z standard deviation
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.05
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_LDR_Z, 0.03f);

/**
 * Accelerometer xy noise power (variance*sampling rate)
 *
 * @group Local Position Estimator
 * @unit (m/s^2)^2-s
 * @min 0.05
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_ACC_XY, 0.135f);

/**
 * Accelerometer z noise power (variance*sampling rate)
 *
 * @group Local Position Estimator
 * @unit (m/s^2)^2-s
 * @min 0.05
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_ACC_Z, 0.175f);

/**
 * Barometric presssure altitude z standard deviation
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.05
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_BAR_Z, 0.255f);

/**
 * GPS xy standard deviation
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_GPS_XY, 0.523f);

/**
 * GPS z standard deviation
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_GPS_Z, 3.55f);

/**
 * GPS xy velocity standard deviation
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.1
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VXY, 0.275f);

/**
 * GPS z velocity standard deviation
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.1
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VZ, 0.237f);

/**
 * Vision position standard deviation
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0
 * @max 10
 */
PARAM_DEFINE_FLOAT(LPE_VIS_P, 0.0f); // vision pos std dev

/**
 * Vision velocity standard deviation
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0
 * @max 10
 */
PARAM_DEFINE_FLOAT(LPE_VIS_V, 1.0f);

/**
 * Disable vision input
 *
 * Set to the appropriate key (328754) to disable vision input.
 *
 * @group Local Position Estimator
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(LPE_NO_VISION, 0);

/**
 * Vicon position standard deviation
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 10
 */
PARAM_DEFINE_FLOAT(LPE_VIC_P, 0.05f); // vicon pos std dev

/**
 * Position propagation process noise
 *
 * @group Local Position Estimator
 * @unit m/s^2
 * @min 0.01
 * @max 10
 */
PARAM_DEFINE_FLOAT(LPE_PN_P, 1.0f);

/**
 * Velocity propagation process noise
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 10
 */
PARAM_DEFINE_FLOAT(LPE_PN_V, 0.1f);
