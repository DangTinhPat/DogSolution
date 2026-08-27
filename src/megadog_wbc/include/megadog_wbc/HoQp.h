#ifndef CONTROLLER_CONTROL_HOQP_H
#define CONTROLLER_CONTROL_HOQP_H

// Ported near-verbatim from skywoodsz/qm_control's qm_wbc::HoQp (Hierarchical
// Optimization QP, following Escande/Mansard/Wieber's null-space-projection
// task-priority formulation). No robot-specific content - see WbcBase.h for
// where babyDog-specific tasks are formulated.
// See /home/dvt/.claude/plans/purrfect-imagining-hartmanis.md (Milestone 3).

#include "megadog_wbc/Task.h"

#include <memory>
#include <string>

namespace megadog
{
namespace hwbc
{

// Solves one priority level of a hierarchical QP: given the null-space of
// every higher-priority level (via higherProblem), find the decision-vector
// step within that null space that best satisfies this level's task without
// disturbing any higher-priority solution.
class HoQp
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using HoQpPtr = std::shared_ptr<HoQp>;

    explicit HoQp(const Task& task) : HoQp(task, nullptr) {}

    HoQp(Task task, HoQpPtr higherProblem);

    matrix_t getStackedZMatrix() const { return stackedZ_; }

    Task getStackedTasks() const { return stackedTasks_; }

    vector_t getStackedSlackSolutions() const { return stackedSlackVars_; }

    vector_t getSolutions() const
    {
        if (!solved_)
        {
            return xPrev_;
        }
        vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
        return x;
    }

    size_t getSlackedNumVars() const { return stackedTasks_.d_.rows(); }

private:
    void initVars();
    void formulateProblem();
    void solveProblem();

    void buildHMatrix();
    void buildCVector();
    void buildDMatrix();
    void buildFVector();

    void buildZMatrix();
    void stackSlackSolutions();

    [[noreturn]] void throwQpFailure(const std::string& stage, int return_value) const;

    Task task_, stackedTasksPrev_, stackedTasks_;
    HoQpPtr higherProblem_;
    bool solved_ = true;

    bool hasEqConstraints_{}, hasIneqConstraints_{};
    size_t numSlackVars_{}, numDecisionVars_{};
    matrix_t stackedZPrev_, stackedZ_;
    vector_t stackedSlackSolutionsPrev_, xPrev_;
    size_t numPrevSlackVars_{};

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
    vector_t c_, f_;
    vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;

    // Convenience matrices that are used multiple times
    matrix_t eyeNvNv_;
    matrix_t zeroNvNx_;
};

}  // namespace hwbc
}  // namespace megadog

#endif  // CONTROLLER_CONTROL_HOQP_H
