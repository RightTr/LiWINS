#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <omp.h>
#include <IKFoM_toolkit/esekfom/esekfom.hpp>
#include "common_lib.h"

class WheelProcess;

// ===========================
// IMU ESTIMATOR DEFINITIONS
// ===========================

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2;
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
((vect3, pose_last))
((SO3, rot_last))
);

MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))
((vect3, gyro))
);

MTK_BUILD_MANIFOLD(process_noise_ikfom,
((vect3, ng))
((vect3, na))
((vect3, nbg))
((vect3, nba))
);

// ===========================
// IMU ESTIMATOR FUNCTIONS
// ===========================

MTK::get_cov<process_noise_ikfom>::type process_noise_cov();
Eigen::Matrix<double, state_ikfom::DIM, 1> get_f(state_ikfom &s, const input_ikfom &in);
Eigen::Matrix<double, state_ikfom::DIM, state_ikfom::DOF> df_dx(state_ikfom &s, const input_ikfom &in);
Eigen::Matrix<double, state_ikfom::DIM, 12> df_dw(state_ikfom &s, const input_ikfom &in);
vect3 SO3ToEuler(const SO3 &orient);

// ===========================
// IMU ESTIMATOR (PREDICT STEP)
// ===========================

/**
 * IMU propagation step: forward-propagates the EKF state by dt, then calls
 * every update function supplied by the caller.
 *
 * Usage:
 *   // IMU only
 *   estimator(kf, dt, in, Q);
 *
 *   // IMU + ZUPT at IMU frequency
 *   estimator(kf, dt, in, Q, {zupt_updater::update});
 *
 *   // IMU + wheel dual-time constraint
 *   // state_clone() must be called before predict so the clone stores the interval start pose.
 *   wheel_updater::state_clone(kf);
 *   estimator(kf, dt, in, Q, {
 *       [&](auto& k){ wheel_updater::update(k, wheel_process, wheel_rot_in_imu, wheel_pos_in_imu); }
 *   });
 */
using UpdateFn = std::function<void(esekfom::esekf<state_ikfom, 12, input_ikfom>&)>;

void estimator(
    esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
    double dt,
    const input_ikfom& in,
    Eigen::Matrix<double, 12, 12>& Q,
    std::initializer_list<UpdateFn> updates = {});

// ===========================
// ZUPT UPDATERS FUNCTIONS
// (standard single-step KF, no esekfom iteration needed)
// ===========================

namespace zupt_updater  { void update(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state); }

// ===========================
// WHEEL ENCODER UPDATER FUNCTIONS
// ===========================

namespace wheel_updater
{
    // Cache the current pose into the cloned previous-pose state before propagation.
    void state_clone(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state);

    // True two-time update: jointly constrains cached clone pose and current pose.
    bool update(
        esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
        const WheelProcess& wheel_process,
        const M3D& wheel_rot_in_imu = Eye3d,
        const V3D& wheel_pos_in_imu = Zero3d);
}

// ===========================
// LIDAR UPDATER FUNCTIONS
// ===========================

namespace lidar_updater
{
    void build_measurement_model(
        const state_ikfom& s,
        const PointCloudXYZI::Ptr& laserCloudOri,
        const PointCloudXYZI::Ptr& corr_normvect,
        int effct_feat_num,
        bool extrinsic_est_en,
        esekfom::dyn_share_datastruct<double>& ekfom_data);
} // namespace lidar_updater

#endif // ESTIMATOR_H
