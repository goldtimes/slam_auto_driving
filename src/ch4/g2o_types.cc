#include "g2o_types.h"
#include "common/g2o_types.h"

namespace lh {
EdgeInertial::EdgeInertial(std::shared_ptr<IMUPreintegration> preinteg, const Vec3d& gravity, double weight)
    : preint_(preinteg), dt_(preinteg->dt_) {
    resize(6);  // 6个关联顶点
    grav_ = gravity;
    setInformation(preinteg->cov_.inverse() * weight);
}

/**
 * 计算残差项
 */
void EdgeInertial::computeError() {
    // 6个顶点
    auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* ba1 = dynamic_cast<const VertexAccBias*>(_vertices[3]);
    auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
    auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

    Vec3d bg = bg1->estimate();
    Vec3d ba = ba1->estimate();

    // 获取预积分得到的delta量
    const SO3 dR = preint_->GetDeltaRoation(bg);
    const Vec3d dv = preint_->GetDeltaVelocity(bg, ba);
    const Vec3d dp = preint_->GetDeltaPosition(bg, ba);
    // 预积分误差项 4.41

    const Vec3d er = (dR.inverse() * p1->estimate().so3().inverse() * p2->estimate().so3()).log();
    Mat3d RiT = p1->estimate().so3().inverse().matrix();
    const Vec3d ev = RiT * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv;
    // clang-format off
    const Vec3d ep = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ -
                            grav_ * dt_ * dt_ / 2) - dp;
    // clang-format on
    _error << er, ev, ep;
}
/**
 * 残差对于状态量的雅可比矩阵
 */
void EdgeInertial::linearizeOplus() {
    auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* ba1 = dynamic_cast<const VertexAccBias*>(_vertices[3]);
    auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
    auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

    Vec3d bg = bg1->estimate();
    Vec3d ba = ba1->estimate();
    Vec3d dbg = bg - preint_->bg_;

    // 中间的变量
    const SO3 R1 = p1->estimate().so3();
    const SO3 R1T = R1.inverse();
    const SO3 R2 = p2->estimate().so3();

    auto dR_dbg = preint_->dR_dbg_;
    auto dv_dbg = preint_->dv_dbg_;
    auto dv_dba = preint_->dv_dba_;
    auto dp_dba = preint_->dp_dba_;
    auto dp_dbg = preint_->dp_dbg_;

    // 估计值
    Vec3d vi = v1->estimate();
    Vec3d vj = v2->estimate();
    Vec3d pi = p1->estimate().translation();
    Vec3d pj = p2->estimate().translation();

    const SO3 dR = preint_->GetDeltaRoation(bg);
    // R1T * R2 就是得到了估计量的相对旋转 然后用R.inverser() * R1 就得到了旋转的差
    const SO3 eR = SO3(dR).inverse() * R1T * R2;
    const Vec3d er = eR.log();
    const Mat3d invJr = SO3::jr_inv(eR);

    // 接下来是残差对状态量的雅可比矩阵
    // 残差对R1 9x3
    _jacobianOplus[0].setZero();
    // dR / dR1 4.42
    _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * (R2.inverse() * R1).matrix();
    // dv / dR1 4.47
    _jacobianOplus[0].block<3, 3>(3, 0) = SO3::hat(R1T * (vj - vi - grav_ * dt_));
    // dp / dR1 4.48d
    _jacobianOplus[0].block<3, 3>(6, 0) = SO3::hat(R1T * (pj - pi - vi * dt_ - 0.5 * grav_ * dt_ * dt_));

    // 残差对p1 9x3
    // dR / dp1 = 0;
    // dp / dp1 4.48a
    _jacobianOplus[0].block<3, 3>(6, 3) = -R1T.matrix();
    // dv /dp1 = 0

    // 残差对v1, 9x3
    _jacobianOplus[1].setZero();
    // dR/dv1 = 0
    // dv / dv1 4.46a
    _jacobianOplus[1].block<3, 3>(3, 0) = -R1T.matrix();
    // dp / dv1. 4.48c
    _jacobianOplus[1].block<3, 3>(6, 0) = -R1T.matrix() * dt_;

    // 残差对bg1
    _jacobianOplus[2].setZero();
    // dR / dbg1
    _jacobianOplus[2].block<3, 3>(0, 0) = -invJr * eR.inverse().matrix() * SO3::jr((dR_dbg * dbg).eval()) * dR_dbg;
    // dv / dbg1
    _jacobianOplus[2].block<3, 3>(3, 0) = -dv_dbg;
    // dp / dbg1
    _jacobianOplus[2].block<3, 3>(6, 0) = -dp_dba;

    // 残差对ba1
    _jacobianOplus[3].setZero();
    // dR / dba1 = 0, dv/dba1
    _jacobianOplus[3].block<3, 3>(3, 0) = -dv_dba;
    // dp / dba1
    _jacobianOplus[3].block<3, 3>(6, 0) = -dv_dba;

    // 残差对Rj,pj
    _jacobianOplus[4].setZero();
    // dR / dR2 4.43
    _jacobianOplus[4].block<3, 3>(0, 0) = invJr;
    // dp / dR2 4.48b
    _jacobianOplus[4].block<3, 3>(6, 3) = R1T.matrix();

    // 残差对v2
    _jacobianOplus[5].setZero();
    // dv / dv2
    _jacobianOplus[5].block<3, 3>(3, 0) = R1T.matrix();
}
}  // namespace lh