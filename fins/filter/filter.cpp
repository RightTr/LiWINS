#include "filter.h"
#include "sensors/wheel_Processing.h"

// ===========================
// IMU ESTIMATOR IMPLEMENTATIONS
// ===========================


MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001);
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001);
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);
	return cov;
}

Eigen::Matrix<double, state_ikfom::DIM, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, state_ikfom::DIM, 1> res = Eigen::Matrix<double, state_ikfom::DIM, 1>::Zero();
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	vect3 a_inertial = s.rot * (in.acc-s.ba);
	for(int i = 0; i < 3; i++){
		res(i) = s.vel[i];
		res(i + 3) = omega[i];
		res(i + 12) = a_inertial[i] + s.grav[i];
	}
	return res;
}

Eigen::Matrix<double, state_ikfom::DIM, state_ikfom::DOF> df_dx(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, state_ikfom::DIM, state_ikfom::DOF> cov =
		Eigen::Matrix<double, state_ikfom::DIM, state_ikfom::DOF>::Zero();
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
	cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 2>(12, 21) = grav_matrix;
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
	return cov;
}

Eigen::Matrix<double, state_ikfom::DIM, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, state_ikfom::DIM, 12> cov = Eigen::Matrix<double, state_ikfom::DIM, 12>::Zero();
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
	return cov;
}

vect3 SO3ToEuler(const SO3 &orient)
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose();
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw;
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) {
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		return vect3(temp, 3);
	}
	if (test < -0.49999*unit) {
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		return vect3(temp, 3);
	}
	_ang <<
		std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2], -sqx - sqy + sqz + sqw),
		std::asin (2*test/unit),
		std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0], sqx - sqy - sqz + sqw);
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
	return vect3(temp, 3);
}

// ===========================
// IMU ESTIMATOR
// ===========================

void estimator(
    esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
    double dt,
    const input_ikfom& in,
    Eigen::Matrix<double, 12, 12>& Q,
    std::initializer_list<UpdateFn> updates)
{
    kf_state.predict(dt, Q, in);
    // for (const auto& update : updates)
    //     update(kf_state);
}

// ===========================
// ZUPT UPDATER FUNCTIONS
// ===========================

namespace zupt_updater
{
void update(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state)
{
	const state_ikfom& s = kf_state.get_x();
	esekfom::dyn_share_datastruct<double> ekfom_data;

	ekfom_data.h_x = MatrixXd::Zero(3, state_ikfom::DOF);
	ekfom_data.h.resize(3);
	// ZUPT measurement model: vel = 0
	ekfom_data.h_x.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	ekfom_data.h.block<3, 1>(0, 0) = -s.vel;
	ekfom_data.R = MatrixXd::Identity(3, 3) * 1e-4;

	auto P = kf_state.get_P();
	const MatrixXd K = P * ekfom_data.h_x.transpose() * (ekfom_data.h_x * P * ekfom_data.h_x.transpose() + ekfom_data.R).inverse();
	const Matrix<double, state_ikfom::DOF, state_ikfom::DOF> I =
		Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity();

	state_ikfom x = kf_state.get_x();
	x.boxplus(Matrix<double, state_ikfom::DOF, 1>(K * ekfom_data.h));
	kf_state.change_x(x);

	const Matrix<double, state_ikfom::DOF, state_ikfom::DOF> KH = K * ekfom_data.h_x;
	Matrix<double, state_ikfom::DOF, state_ikfom::DOF> P_new =
		(I - KH) * P * (I - KH).transpose() + K * ekfom_data.R * K.transpose();
	kf_state.change_P(P_new);
}
} // namespace zupt_updater

// ===========================
// WHEEL UPDATER FUNCTIONS
// ===========================

namespace wheel_updater
{
void state_clone(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state)
{
    state_ikfom x = kf_state.get_x();
    x.pose_last = x.pos;
    x.rot_last = x.rot;
    kf_state.change_x(x);

    auto P_old = kf_state.get_P();
    auto P_new = P_old;
    const int pos_idx = MTK::getStartIdx(&state_ikfom::pos);
    const int rot_idx = MTK::getStartIdx(&state_ikfom::rot);
    const int clone_pos_idx = MTK::getStartIdx(&state_ikfom::pose_last);
    const int clone_rot_idx = MTK::getStartIdx(&state_ikfom::rot_last);

    P_new.block<3, state_ikfom::DOF>(clone_pos_idx, 0) = P_old.block<3, state_ikfom::DOF>(pos_idx, 0);
    P_new.block<state_ikfom::DOF, 3>(0, clone_pos_idx) = P_old.block<state_ikfom::DOF, 3>(0, pos_idx);
    P_new.block<3, state_ikfom::DOF>(clone_rot_idx, 0) = P_old.block<3, state_ikfom::DOF>(rot_idx, 0);
    P_new.block<state_ikfom::DOF, 3>(0, clone_rot_idx) = P_old.block<state_ikfom::DOF, 3>(0, rot_idx);

    kf_state.change_P(P_new);
}

bool update(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
            const WheelProcess& wheel_process)
{
    const int clone_rot_idx = MTK::getStartIdx(&state_ikfom::rot_last);
    const int clone_pos_idx = MTK::getStartIdx(&state_ikfom::pose_last);
    const int rot_idx = MTK::getStartIdx(&state_ikfom::rot);
    const int pos_idx = MTK::getStartIdx(&state_ikfom::pos);

    const state_ikfom& curr_state = kf_state.get_x();
    WheelPoseState pose0;
    pose0.rot = curr_state.rot_last.toRotationMatrix();
    pose0.pos = curr_state.pose_last;
    pose0.rot_fej = pose0.rot;
    pose0.pos_fej = pose0.pos;

    WheelPoseState pose1;
    pose1.rot = curr_state.rot.toRotationMatrix();
    pose1.pos = curr_state.pos;
    pose1.rot_fej = pose1.rot;
    pose1.pos_fej = pose1.pos;

    Eigen::MatrixXd wheel_H;
    Eigen::VectorXd wheel_res;
    wheel_process.ComputeLinearSystem(pose0, pose1, wheel_H, wheel_res);

    const bool is_3d = wheel_process.wheel_type == Wheel3DAng || wheel_process.wheel_type == Wheel3DLin || wheel_process.wheel_type == Wheel3DCen;
    MatrixXd wheel_cov = is_3d ? MatrixXd(wheel_process.latest_result().Cov_3D) : MatrixXd(wheel_process.latest_result().Cov_2D);

    MatrixXd H = MatrixXd::Zero(wheel_H.rows(), state_ikfom::DOF);
    H.block(0, clone_rot_idx, wheel_H.rows(), 3) = wheel_H.block(0, 0, wheel_H.rows(), 3);
    H.block(0, clone_pos_idx, wheel_H.rows(), 3) = wheel_H.block(0, 3, wheel_H.rows(), 3);
    H.block(0, rot_idx, wheel_H.rows(), 3) = wheel_H.block(0, 6, wheel_H.rows(), 3);
    H.block(0, pos_idx, wheel_H.rows(), 3) = wheel_H.block(0, 9, wheel_H.rows(), 3);

    auto P = kf_state.get_P();
    const MatrixXd S = H * P * H.transpose() + wheel_cov;
    const MatrixXd K = P * H.transpose() * S.inverse();
    const Matrix<double, state_ikfom::DOF, state_ikfom::DOF> I =
        Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity();

    state_ikfom x = curr_state;
    const Matrix<double, state_ikfom::DOF, 1> dx = K * wheel_res;
    x.boxplus(dx);
    kf_state.change_x(x);

    const Matrix<double, state_ikfom::DOF, state_ikfom::DOF> KH = K * H;
    Matrix<double, state_ikfom::DOF, state_ikfom::DOF> P_new =
        (I - KH) * P * (I - KH).transpose() + K * wheel_cov * K.transpose();
    kf_state.change_P(P_new);
    return true;
}
} // namespace wheel_updater

// ===========================
// LIDAR UPDATER IMPLEMENTATION
// ===========================

namespace lidar_updater
{

void build_measurement_model(
    const state_ikfom& s,
    const PointCloudXYZI::Ptr& laserCloudOri,
    const PointCloudXYZI::Ptr& corr_normvect,
    int effct_feat_num,
    bool extrinsic_est_en,
    esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType& laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType& norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) <<
                norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) <<
                norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

} // namespace lidar_updater
