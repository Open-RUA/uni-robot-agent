// Pure kinematics C ABI for Body preflight. No controller, ROS node or CAN access.
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace {
struct Solver {
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik;
    KDL::Vector origin;
    explicit Solver(const char* urdf) {
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf, tree) ||
            !tree.getChain("base_link", "link6", chain) || chain.getNrOfJoints() != 6)
            throw std::runtime_error("invalid X5 chain");
        fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
        // Same solver family as the pinned ARX SDK; bound work per sample.
        ik = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain, 1e-7, 150, 1e-12);
        KDL::JntArray zero(6);
        KDL::Frame home;
        if (fk->JntToCart(zero, home) < 0) throw std::runtime_error("FK failed");
        origin = home.p;
    }
};
KDL::JntArray joints(const double* q) {
    KDL::JntArray result(6);
    for (unsigned i = 0; i < 6; ++i) {
        if (!std::isfinite(q[i])) throw std::runtime_error("nonfinite joint");
        result(i) = q[i];
    }
    return result;
}
}

extern "C" {
void* rua_kinematics_create(const char* urdf) noexcept {
    try { return urdf ? new Solver(urdf) : nullptr; } catch (...) { return nullptr; }
}
void rua_kinematics_destroy(void* context) noexcept { delete static_cast<Solver*>(context); }
int rua_kinematics_origin(void* context, double* out) noexcept {
    if (!context || !out) return -1;
    auto& s = *static_cast<Solver*>(context);
    for (unsigned i = 0; i < 3; ++i) out[i] = s.origin(i);
    return 0;
}
int rua_kinematics_fk(void* context, const double* q, double* out) noexcept {
    if (!context || !q || !out) return -1;
    try {
        auto& s = *static_cast<Solver*>(context);
        KDL::Frame frame;
        if (s.fk->JntToCart(joints(q), frame) < 0) return -1;
        for (unsigned i = 0; i < 3; ++i) out[i] = frame.p(i) - s.origin(i);
        frame.M.GetRPY(out[3], out[4], out[5]);
        return 0;
    } catch (...) { return -1; }
}
// out: six joint angles, position residual (m), rotation residual (rad).
int rua_kinematics_ik(void* context, const double* seed, const double* pose, double* out) noexcept {
    if (!context || !seed || !pose || !out) return -1;
    try {
        auto& s = *static_cast<Solver*>(context);
        for (unsigned i = 0; i < 6; ++i) if (!std::isfinite(pose[i])) return -1;
        KDL::Frame target(KDL::Rotation::RPY(pose[3], pose[4], pose[5]),
                          KDL::Vector(pose[0], pose[1], pose[2]) + s.origin);
        KDL::JntArray solution(6);
        s.ik->CartToJnt(joints(seed), target, solution);
        // Residuals, rather than the solver status alone, decide acceptance.
        KDL::Frame actual;
        if (s.fk->JntToCart(solution, actual) < 0) return -1;
        for (unsigned i = 0; i < 6; ++i) out[i] = solution(i);
        out[6] = (actual.p - target.p).Norm();
        out[7] = KDL::diff(actual.M, target.M).Norm();
        for (unsigned i = 0; i < 8; ++i) if (!std::isfinite(out[i])) return -1;
        return 0;
    } catch (...) { return -1; }
}
}
