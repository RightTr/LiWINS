#pragma once

#include "filter/filter.h"

inline void pointBodyToWorld(PointType const* pi, PointType* po, const state_ikfom& s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    po->x         = p_global(0);
    po->y         = p_global(1);
    po->z         = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
inline void pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi,
                             Eigen::Matrix<T, 3, 1>&       po,
                             const state_ikfom&             s)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

inline void RGBpointBodyToWorld(PointType const* pi, PointType* po, const state_ikfom& s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    po->x         = p_global(0);
    po->y         = p_global(1);
    po->z         = p_global(2);
    po->intensity = pi->intensity;
}

inline void RGBpointBodyLidarToIMU(PointType const* pi, PointType* po, const state_ikfom& s)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(s.offset_R_L_I * p_body_lidar + s.offset_T_L_I);
    po->x         = p_body_imu(0);
    po->y         = p_body_imu(1);
    po->z         = p_body_imu(2);
    po->intensity = pi->intensity;
}
