// Ported near-verbatim from skywoodsz/qm_control's qm_wbc::HoQp.cpp - see
// include/megadog_wbc/HoQp.h for provenance/context.

#include "megadog_wbc/HoQp.h"

#include <qpOASES.hpp>

#include <sstream>
#include <stdexcept>
#include <utility>

namespace megadog
{
namespace hwbc
{

HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem)
    : task_(std::move(task)), higherProblem_(std::move(higherProblem))
{
    initVars();
    formulateProblem();
    solveProblem();
    // For next problem
    buildZMatrix();
    stackSlackSolutions();
}

void HoQp::initVars()
{
    // Task variables
    numSlackVars_ = task_.d_.rows();
    hasEqConstraints_ = task_.a_.rows() > 0;
    hasIneqConstraints_ = numSlackVars_ > 0;

    // Pre-Task variables
    if (higherProblem_ != nullptr) {
        stackedZPrev_ = higherProblem_->getStackedZMatrix();
        stackedTasksPrev_ = higherProblem_->getStackedTasks();
        stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions();
        xPrev_ = higherProblem_->getSolutions();
        numPrevSlackVars_ = higherProblem_->getSlackedNumVars();

        numDecisionVars_ = stackedZPrev_.cols();
    } else {
        numDecisionVars_ = std::max(task_.a_.cols(), task_.d_.cols());

        stackedTasksPrev_ = Task(numDecisionVars_);
        stackedZPrev_ = matrix_t::Identity(numDecisionVars_, numDecisionVars_);
        stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);
        xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);
        numPrevSlackVars_ = 0;
    }

    stackedTasks_ = task_ + stackedTasksPrev_;

    // Init convenience matrices
    eyeNvNv_ = matrix_t::Identity(numSlackVars_, numSlackVars_);
    zeroNvNx_ = matrix_t::Zero(numSlackVars_, numDecisionVars_);
}

void HoQp::formulateProblem()
{
    buildHMatrix();
    buildCVector();
    buildDMatrix();
    buildFVector();
}

void HoQp::buildHMatrix()
{
    matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);

    if (hasEqConstraints_) {
        // Make sure that all eigenvalues of A_t_A are non-negative, which could arise due to numerical issues
        matrix_t aCurrZPrev = task_.a_ * stackedZPrev_;
        zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * matrix_t::Identity(numDecisionVars_, numDecisionVars_);
        // This way of splitting up the multiplication is about twice as fast as multiplying 4 matrices
    } else {
        zTaTaz.setZero();
    }

    h_ = (matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_)
              << zTaTaz, zeroNvNx_.transpose(),
          zeroNvNx_, eyeNvNv_)
             .finished();
}

void HoQp::buildCVector()
{
    vector_t c = vector_t::Zero(numDecisionVars_ + numSlackVars_);
    vector_t zeroVec = vector_t::Zero(numSlackVars_);

    vector_t temp(numDecisionVars_);
    if (hasEqConstraints_) {
        temp = (task_.a_ * stackedZPrev_).transpose() * (task_.a_ * xPrev_ - task_.b_);
    } else {
        temp.setZero();
    }

    c_ = (vector_t(numDecisionVars_ + numSlackVars_) << temp, zeroVec).finished();
}

void HoQp::buildDMatrix()
{
    matrix_t stackedZero = matrix_t::Zero(numPrevSlackVars_, numSlackVars_);

    matrix_t dCurrZ;
    if (hasIneqConstraints_) {
        dCurrZ = task_.d_ * stackedZPrev_;
    } else {
        dCurrZ = matrix_t::Zero(0, numDecisionVars_);
    }

    // NOTE: This is upside down compared to the paper,
    // but more consistent with the rest of the algorithm
    d_ = (matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_)
              << zeroNvNx_, -eyeNvNv_,
          stackedTasksPrev_.d_ * stackedZPrev_, stackedZero,
          dCurrZ, -eyeNvNv_)
             .finished();
}

void HoQp::buildFVector()
{
    vector_t zeroVec = vector_t::Zero(numSlackVars_);

    vector_t fMinusDXPrev;
    if (hasIneqConstraints_) {
        fMinusDXPrev = task_.f_ - task_.d_ * xPrev_;
    } else {
        fMinusDXPrev = vector_t::Zero(0);
    }

    f_ = (vector_t(2 * numSlackVars_ + numPrevSlackVars_) << zeroVec,
          stackedTasksPrev_.f_ - stackedTasksPrev_.d_ * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev)
             .finished();
}

void HoQp::buildZMatrix()
{
    if (!solved_) {
        stackedZ_ = stackedZPrev_;
        return;
    }
    if (hasEqConstraints_) {
        assert((task_.a_.cols() > 0));
        stackedZ_ = stackedZPrev_ * (task_.a_ * stackedZPrev_).fullPivLu().kernel();
    } else {
        stackedZ_ = stackedZPrev_;
    }
}

void HoQp::solveProblem()
{
    auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qpProblem.setOptions(options);
    int nWsr = 100;

    const qpOASES::returnValue initStatus =
        qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
    if (initStatus != qpOASES::SUCCESSFUL_RETURN) {
        if (higherProblem_ != nullptr) {
            solved_ = false;
            stackedTasks_ = stackedTasksPrev_;
            decisionVarsSolutions_ = vector_t::Zero(numDecisionVars_);
            slackVarsSolutions_ = vector_t::Zero(numSlackVars_);
            return;
        }
        throwQpFailure("init", static_cast<int>(initStatus));
    }
    vector_t qpSol(numDecisionVars_ + numSlackVars_);

    const qpOASES::returnValue solutionStatus = qpProblem.getPrimalSolution(qpSol.data());
    if (solutionStatus != qpOASES::SUCCESSFUL_RETURN) {
        if (higherProblem_ != nullptr) {
            solved_ = false;
            stackedTasks_ = stackedTasksPrev_;
            decisionVarsSolutions_ = vector_t::Zero(numDecisionVars_);
            slackVarsSolutions_ = vector_t::Zero(numSlackVars_);
            return;
        }
        throwQpFailure("getPrimalSolution", static_cast<int>(solutionStatus));
    }

    decisionVarsSolutions_ = qpSol.head(numDecisionVars_);
    slackVarsSolutions_ = qpSol.tail(numSlackVars_);
}

void HoQp::throwQpFailure(const std::string& stage, const int return_value) const
{
    std::ostringstream message;
    message << "HoQp qpOASES " << stage << " failed: return=" << return_value
            << " decision_vars=" << numDecisionVars_
            << " slack_vars=" << numSlackVars_
            << " constraints=" << f_.size();
    throw std::runtime_error(message.str());
}

void HoQp::stackSlackSolutions()
{
    if (!solved_) {
        stackedSlackVars_ = stackedSlackSolutionsPrev_;
        return;
    }
    if (higherProblem_ != nullptr) {
        stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(), slackVarsSolutions_);
    } else {
        stackedSlackVars_ = slackVarsSolutions_;
    }
}

}  // namespace hwbc
}  // namespace megadog
