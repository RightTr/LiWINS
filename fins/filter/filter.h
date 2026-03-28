#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>
#include "common_lib.h"

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
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in);
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in);
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in);
vect3 SO3ToEuler(const SO3 &orient);

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

// ===========================
// WHEEL ENCODER UPDATER FUNCTIONS
// ===========================

// namespace wheel_updater
// {
//     struct WheelOdom
//     {
//         double x;
//         double y;
//         double yaw;
//     };

//     void build_measurement_model(
//         const state_ikfom& s,
//         const WheelOdom& wheel_odom,
//         bool extrinsic_est_en,
//         esekfom::dyn_share_datastruct<double>& ekfom_data);
// }


#endif // ESTIMATOR_H
