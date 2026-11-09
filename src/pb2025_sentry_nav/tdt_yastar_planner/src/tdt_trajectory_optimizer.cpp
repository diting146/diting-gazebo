/*
The MIT License (MIT)

Copyright (c) 2026 Rongxuan Ye and BoLin Li

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
// Extracted from TDT Nav Kit src/MinimumSnapOsqp/minimumSnap.cpp.
// Original author: SnifferCaptain. See TDT_LICENSE_NOTICE.md for provenance.
// Retained: quintic closed equality-constrained solve with derivative order 3
// (minimum jerk), normalized segment time, and original time-allocation heuristic.
// Removed: OSQP/SFC backends and original point-only collision iteration.
// Project adapter below validates the entire emitted polyline with a body-sweep
// callback and falls back to the unchanged validated YAstar path.
#include "tdt_yastar_planner/tdt_trajectory_optimizer.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace tdt_yastar_planner
{
namespace
{
class TdtClosedMinimumJerk
{
public:
  using MatXd = Eigen::MatrixXd;
  using VecXd = Eigen::VectorXd;
  using PointPair = std::array<float, 4>;
  struct Item
  {
    Eigen::Vector2f xy = Eigen::Vector2f::Zero();
    Eigen::Vector2f vxvy = Eigen::Vector2f::Zero();
    bool useVxvy = false;
  };
  static std::vector<double> trapezoidalTimeAllocation(
    const std::vector<Eigen::Vector2f> & path, float maxSpeed, float maxAcc);
  std::pair<MatXd, MatXd> closeSolver(
    const std::vector<double> & times, const std::vector<Item> & items, bool normT) const;
private:
  int order{6};
  int _maxdx{3};
  static int Axx(int from, int n)
  {
    int op = 1;
    for (int i = 0; i < n; ++i) op *= from - i;
    return op;
  }
  MatXd generateQ(const std::vector<double> &, bool) const;
  MatXd generateA(const std::vector<double> &, bool) const;
  VecXd generateLow(const std::vector<Eigen::Vector2f> &,
    float = -std::numeric_limits<float>::infinity()) const;
  VecXd generateUp(const std::vector<Eigen::Vector2f> &,
    float = std::numeric_limits<float>::infinity()) const;
  static std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>>
  postProcess(const std::vector<PointPair> &);
};
using MatXd = TdtClosedMinimumJerk::MatXd;
using VecXd = TdtClosedMinimumJerk::VecXd;
using PointPair = TdtClosedMinimumJerk::PointPair;
static std::pair<MatXd, MatXd> unpack(const MatXd& solution, int order_){
    MatXd x(solution.rows() / order_, order_);
    MatXd y(solution.rows() / order_, order_);
    for(int i = 0; i < x.rows(); i++){
        for(int j = 0; j < order_; j++){
            x(i, order_ - 1 - j) = solution(i * order_ + j, 0);
            y(i, order_ - 1 - j) = solution(i * order_ + j, 1);
        }
    }
    return {x, y};
}

static bool validTimes(const std::vector<double>& times, size_t pointNum, int order_, int maxdx){
    if(pointNum < 2 || times.size() + 1 != pointNum || order_ < 3 || maxdx < 2){
        return false;
    }
    return std::all_of(times.begin(), times.end(), [](double time){
        return std::isfinite(time) && time > 0.;
    });
}

TdtClosedMinimumJerk::MatXd TdtClosedMinimumJerk::generateQ(const std::vector<double>& timeAllocated, bool normT) const{
    MatXd sub_Q = MatXd::Zero(order, order);
    int maxdx = std::min(_maxdx, order-1);
    int aq = static_cast<int>(order * timeAllocated.size());// Q矩阵的行数/列数
    MatXd Q = MatXd::Zero(aq, aq);
    for (int k = 0; k < static_cast<int>(timeAllocated.size()); k++) {
        MatXd sub_Q = MatXd::Zero(order, order);
        if (normT){
            // tau为t/T归一化时间。原系数为a。归一化系数为b。
            // b = T @ a; integral (d^r p/dt^r)^2 dt
            // = T^(1-2r) integral (d^r p/dtau^r)^2 dtau.
            // 遍历多项式的每一项
            for (int i = maxdx; i < order; i++) {
                for (int l = i; l < order; l++) {
                    // l=i或者l=maxdx效果都一样
                    // 积分（pow（四阶导））|0到dt
                    sub_Q(i, l) = std::pow(timeAllocated[k], 1 - 2 * maxdx) *
                        static_cast<double>(Axx(i, maxdx) * Axx(l, maxdx)) / (i + l - 2 * maxdx + 1);
                }
            }
            // 距离惩罚项，依据为s（↓）=v（↓） * t（=）（另外，2，2惩罚项实测没啥区别）
            // 原惩罚是 0.01*a_1^2，换元后为 0.01*b_1^2/T^2。
            sub_Q(1, 1) += 0.01 / (timeAllocated[k] * timeAllocated[k]);
        } else{
            // 遍历多项式的每一项
            for (int i = maxdx; i < order; i++) {
                for (int l = i; l < order; l++) {
                    // l=i或者l=maxdx效果都一样
                    // 积分（pow（四阶导））|0到dt
                    sub_Q(i, l) = static_cast<double>(Axx(i, maxdx) * Axx(l, maxdx)) *
                        pow(timeAllocated[k], i + l - 2 * maxdx + 1) / (i + l - 2 * maxdx + 1);
                }
            }
            // 距离惩罚项，依据为s（↓）=v（↓） * t（=）（另外，2，2惩罚项实测没啥区别）
            sub_Q(1, 1) += 0.01;
        }
        // 拼接Q矩阵
        Q.block(k * order, k * order, order, order) = sub_Q;
    }
    return Q;
}

TdtClosedMinimumJerk::MatXd TdtClosedMinimumJerk::generateA(const std::vector<double>& timeAllocated, bool normT) const{
    const int numSegment = static_cast<int>(timeAllocated.size());// 分段数量
    const int aq = order * numSegment;// A矩阵的列数
    const int continuity = std::min(_maxdx, order-1);

    const int numConstraints =
        1                              // 1个起点速度约束
        + 1                            // 1个起点位置约束
        + numSegment                   // 1~end个控制点位置约束
        + 1                            // 1个终点速度约束
        + (numSegment - 1) * continuity // numSegment - 1个连续性约束
        + (numSegment - 1);            // numSegment - 1个途经点速度约束
    MatXd A = MatXd::Zero(numConstraints, aq);
    // 返回一个子向量矩阵，与x矩阵乘法能得到f(x)的值 [1,order]
    auto getfx = [&](double ip, double segment_t){
        if(normT){
            ip /= segment_t;
        }
        MatXd fx = MatXd::Zero(1, order);
        fx(0, 0) = 1; // 防止0的0次方
        for (int i = 1; i < order; i++) {
            fx(0, i) = pow(ip, i);
        }
        return fx;
    };
    // 始终返回物理时间导数：d^a p/dt^a = T^-a d^a p/dtau^a。
    auto getdnx = [&](double ip, int n, double segment_t){
        MatXd fx = MatXd::Zero(n, order);
        const double u = normT ? ip / segment_t : ip;
        for(int a = 0; a < n; a++){
            const double scale = normT ? std::pow(segment_t, -a) : 1.0;
            for(int b = a; b < order; b++){
                // u=0 且 b=a 时，导数系数仍然是 a!，不能直接设为 1。
                fx(a, b) = Axx(b, a) * (b == a ? 1.0 : std::pow(u, b - a)) * scale;
            }
        }
        return fx;
    };
    // 返回一个子向量矩阵，与x矩阵乘法能得到f(x)的n阶导数值 [1,order]
    // 保留用于未来可能的起始/终点速度约束
    [[maybe_unused]] auto getdx = [&](double ip, int n, double segment_t){
        MatXd fx = getdnx(ip, n + 1, segment_t).row(n);
        return fx;
    };

    int atline = 0;
    // 起点速度约束（一阶导数）
    A.block(atline++, 0, 1, order) = getdx(0, 1, timeAllocated[0]);
    // 起点位置约束
    A.block(atline++, 0, 1, order) = getfx(0, timeAllocated[0]);
    // 1~最后一个点的位置约束，使用t=segmentTime以确保A矩阵不病态
    for(int i = 0; i < numSegment; i++){
        A.block(atline++, i*order, 1, order) = getfx(timeAllocated[i], timeAllocated[i]);
    }
    // 终点速度约束（一阶导数）
    A.block(atline++, (numSegment - 1)*order, 1, order) = getdx(timeAllocated.back(), 1, timeAllocated.back());
    // 安全飞行走廊的交集连续性方程
    for(int i = 0; i < numSegment - 1; i++){
        A.block(atline, i * order, continuity, order) = getdnx(timeAllocated[i], continuity, timeAllocated[i]);
        A.block(atline, (i + 1) * order, continuity, order) = -getdnx(0, continuity, timeAllocated[i + 1]);
        // 归一化避免A矩阵病态
        for(int j = 0; j < continuity; j++){
            // eigen应当避免使用auto
            double norm = A.row(atline + j).norm();
            A.row(atline + j) /= norm;
        }
        atline += continuity;
    }
    // 途经点速度约束行，未启用时由上下界设置为无约束。
    // 途经点速度使用后一段的起始导数，连续性方程会保证两侧导数一致。
    for(int i = 1; i < numSegment; i++){
        // 归一化时间下，升幂多项式的一阶系数需要换算为物理速度。
        A(atline, i * order + 1) = normT ? 1. / timeAllocated[i] : 1.;
        atline++;
    }
    return A;
}

TdtClosedMinimumJerk::VecXd TdtClosedMinimumJerk::generateLow(const std::vector<Eigen::Vector2f>& restrictArea, float initVelocityLow) const{
    // 按顺序：
    // 0：起点速度约束
    // 1：起点位置约束
    // 2~numSegment+1：每个控制点的位置约束
    // numSegment+2：终点速度约束（默认不限制）
    // 其余：连续性约束（等于0）
    // 最后：途经点速度约束（未启用时不限制）
    const int numSegment = static_cast<int>(restrictArea.size()) - 1; // 分段数量
    const int continuity = std::min(_maxdx, order-1);
    VecXd low = VecXd::Zero(
        1                              // 1个起点速度约束
        + numSegment + 1               // 所有控制点的位置约束
        + 1                            // 1个终点速度约束
        + (numSegment - 1) * continuity // numSegment - 1个连续性约束
        + (numSegment - 1)             // numSegment - 1个途经点速度约束
    );
    int atline = 0;
    // 起点速度约束
    low(atline++) = initVelocityLow;
    // 每个控制点的位置约束
    for(const auto & i : restrictArea){
        low(atline++) = std::min(i(0), i(1));
    }
    // 终点速度约束，默认不限制
    low(atline++) = -std::numeric_limits<double>::infinity();
    // 安全飞行走廊交集点的连续性约束 = 0
    atline += (numSegment - 1) * continuity;
    // 途经点速度约束，未启用时不限制
    for(int i = 1; i < numSegment; i++){
        low(atline++) = -std::numeric_limits<double>::infinity();
    }
    return low;
}

TdtClosedMinimumJerk::VecXd TdtClosedMinimumJerk::generateUp(const std::vector<Eigen::Vector2f>& restrictArea, float initVelocityUp) const{
    // 按顺序：
    // 0：起点速度约束
    // 1：起点位置约束
    // 2~numSegment+1：每个控制点的位置约束
    // numSegment+2：终点速度约束（默认不限制）
    // 其余：连续性约束（等于0）
    // 最后：途经点速度约束（未启用时不限制）
    const int numSegment = static_cast<int>(restrictArea.size()) - 1; // 飞行走廊的数量
    const int continuity = std::min(_maxdx, order-1);
    VecXd up = VecXd::Zero(
        1                              // 1个起点速度约束
        + numSegment + 1               // 所有控制点的位置约束
        + 1                            // 1个终点速度约束
        + (numSegment - 1) * continuity // numSegment - 1个连续性约束
        + (numSegment - 1)             // numSegment - 1个途经点速度约束
    );
    int atline = 0;
    // 起点速度约束
    up(atline++) = initVelocityUp;
    // 每个控制点的位置约束
    for(const auto & i : restrictArea){
        up(atline++) = std::max(i(0), i(1));
    }
    // 终点速度约束，默认不限制
    up(atline++) = std::numeric_limits<double>::infinity();
    // 安全飞行走廊交集点的连续性约束 = 0
    atline += (numSegment - 1) * continuity;
    // 途经点速度约束，未启用时不限制
    for(int i = 1; i < numSegment; i++){
        up(atline++) = std::numeric_limits<double>::infinity();
    }
    return up;
}

std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>> TdtClosedMinimumJerk::postProcess(const std::vector<PointPair> &corridor){
    std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>> op(corridor.size(), corridor.size());
    for(size_t i = 0; i < corridor.size(); i++){
        op.first[i] = Eigen::Vector2f(corridor[i][0], corridor[i][2]);// x
        op.second[i] = Eigen::Vector2f(corridor[i][1], corridor[i][3]);// y
    }
    return op;
}

std::pair<TdtClosedMinimumJerk::MatXd, TdtClosedMinimumJerk::MatXd> TdtClosedMinimumJerk::closeSolver(
    const std::vector<double>& timeAllocated,
    const std::vector<TdtClosedMinimumJerk::Item>& items,
    bool normT
) const{
    if(!validTimes(timeAllocated, items.size(), order, _maxdx)){
        return {};
    }

    std::vector<PointPair> corridor;
    for(const auto& item : items){
        corridor.push_back({item.xy.x(), item.xy.y(), item.xy.x(), item.xy.y()});
    }

    for(const auto& item : items){
        if(!item.xy.allFinite() || (item.useVxvy && !item.vxvy.allFinite())){
            return {};
        }
    }

    auto area = postProcess(corridor);
    MatXd A = generateA(timeAllocated, normT);

    // 约束行在generateA、generateLow和generateUp中一次性确定。
    // 下面只根据Item填写已存在约束行的边界值，不增加或删除约束行。
    const int numItems = static_cast<int>(items.size());
    const int endVelocityRow = numItems + 1;
    const int interiorVelocityRow = A.rows() - (numItems - 2);
    auto makeBounds = [&](const std::vector<Eigen::Vector2f>& axisArea, int axis)
        -> std::pair<VecXd, VecXd>{
        VecXd low = generateLow(axisArea);
        VecXd up = generateUp(axisArea);
        for(size_t i = 0; i < items.size(); i++){
            if(!items[i].useVxvy){
                continue;
            }

            const int positionRow = static_cast<int>(i) + 1;
            low(positionRow) = items[i].xy(axis);
            up(positionRow) = low(positionRow);

            int velocityRow = 0;
            if(i + 1 == items.size()){
                velocityRow = endVelocityRow;
            }else if(i > 0){
                velocityRow = interiorVelocityRow + static_cast<int>(i) - 1;
            }
            low(velocityRow) = items[i].vxvy(axis);
            up(velocityRow) = low(velocityRow);
        }
        return std::make_pair(low, up);
    };

    const auto xBounds = makeBounds(area.first, 0);
    const auto yBounds = makeBounds(area.second, 1);
    MatXd low(A.rows(), 2);
    MatXd up(A.rows(), 2);
    low.col(0) = xBounds.first;
    low.col(1) = yBounds.first;
    up.col(0) = xBounds.second;
    up.col(1) = yBounds.second;

    // 位置、连续性和指定速度约束是等式，未指定的首末速度约束不参与闭式求解。
    std::vector<int> equalityRows;
    for(int i = 0; i < A.rows(); i++){
        if(low.row(i).allFinite()){
            equalityRows.push_back(i);
        }
    }

    MatXd equality(static_cast<int>(equalityRows.size()), A.cols());
    MatXd target(static_cast<int>(equalityRows.size()), 2);
    for(size_t i = 0; i < equalityRows.size(); i++){
        double scale = A.row(equalityRows[i]).norm();
        if(!std::isfinite(scale) || scale == 0.){
            return {};
        }
        equality.row(static_cast<int>(i)) = A.row(equalityRows[i]) / scale;
        target.row(static_cast<int>(i)) = low.row(equalityRows[i]) / scale;
    }

    // p=p0+Nz：p0满足等式，N的列张成等式的零空间。
    // 在自由变量z上最小化二次型，两个方向共用同一个分解。
    Eigen::FullPivLU<MatXd> decomposition(equality);
    MatXd solution = decomposition.solve(target);
    if((equality * solution - target).cwiseAbs().maxCoeff() > 1e-7){
        return {};
    }

    if(decomposition.dimensionOfKernel() > 0){
        MatXd kernel = decomposition.kernel();
        Eigen::HouseholderQR<MatXd> qr(kernel);
        kernel = qr.householderQ() * MatXd::Identity(kernel.rows(), kernel.cols());

        MatXd Q = generateQ(timeAllocated, normT).selfadjointView<Eigen::Upper>();
        Q /= std::max(Q.cwiseAbs().maxCoeff(), 1e-12);
        MatXd reducedQ = kernel.transpose() * Q * kernel;
        MatXd rhs = -kernel.transpose() * Q * solution;
        MatXd freeValues = reducedQ.completeOrthogonalDecomposition().solve(rhs);
        solution += kernel * freeValues;
    }

    if(!solution.allFinite() || (equality * solution - target).cwiseAbs().maxCoeff() > 1e-6){
        return {};
    }
    return unpack(solution, order);
}

std::vector<double> TdtClosedMinimumJerk::trapezoidalTimeAllocation(const std::vector<Eigen::Vector2f>& path, float maxSpeed, float maxAcc){
    if(path.size() < 2 || !std::isfinite(maxSpeed) || !std::isfinite(maxAcc) ||
       maxSpeed <= 0.f || maxAcc <= 0.f ||
       !std::all_of(path.begin(), path.end(), [](const Eigen::Vector2f& point){
           return point.allFinite();
       })){
        return {};
    }

    std::vector<double> timeAllocated(path.size()-1);
    // 按小段分配梯形时间，这样也可以起到转弯减速的作用。
    float distThs = 0.5f * maxSpeed * maxSpeed / maxAcc;
    for(size_t i = 0; i < path.size() - 1; i++){
        float dist = (path[i+1] - path[i]).norm();
        if(dist < distThs){
            timeAllocated[i] = 2 * sqrtf(dist / maxAcc);
        }
        else{
            timeAllocated[i] = (dist - distThs) / maxSpeed + 2 * sqrtf(distThs / maxAcc);
        }
    }
    return timeAllocated;
}

using Clock = std::chrono::steady_clock;
struct StopOptimization {const char * reason;};

double pointSegmentDistance(
  const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d ab = b - a;
  const double squared = ab.squaredNorm();
  const double t = squared > 1.0e-20 ? std::clamp((p - a).dot(ab) / squared, 0.0, 1.0) : 0.0;
  return (p - a - t * ab).norm();
}

template<typename Check>
std::vector<std::size_t> cornerIndices(
  const GeometricPath & points, double tolerance, std::size_t limit, Check check)
{
  std::vector<bool> retained(points.size(), false);
  retained.front() = retained.back() = true;
  std::size_t count = 2;
  std::vector<std::pair<std::size_t, std::size_t>> pending{{0, points.size() - 1}};
  while (!pending.empty()) {
    check();
    const auto [first, last] = pending.back();
    pending.pop_back();
    double largest = tolerance;
    std::size_t selected = first;
    for (std::size_t i = first + 1; i < last; ++i) {
      if ((i & 63U) == 0) check();
      const double d = pointSegmentDistance(points[i], points[first], points[last]);
      if (d > largest) {largest = d; selected = i;}
    }
    if (selected != first) {
      if (++count > limit) throw StopOptimization{"corner_point_budget"};
      retained[selected] = true;
      pending.emplace_back(first, selected);
      pending.emplace_back(selected, last);
    }
  }
  std::vector<std::size_t> indices;
  for (std::size_t i = 0; i < retained.size(); ++i) if (retained[i]) indices.push_back(i);
  return indices;
}

double polylineLength(const GeometricPath & points)
{
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) length += (points[i] - points[i - 1]).norm();
  return length;
}

template<typename Check>
GeometricPath makeControls(
  const GeometricPath & path, const TrajectoryOptimizationOptions & options, Check check)
{
  const auto indices = cornerIndices(
    path, options.corner_tolerance, options.max_control_points, check);
  GeometricPath controls{path.front()};
  // Corners are mandatory. Add points on the original polyline, not on shortcuts
  // across a corner; excess budget rejects smoothing rather than dropping bends.
  for (std::size_t j = 1; j < indices.size(); ++j) {
    const std::size_t first = indices[j - 1], last = indices[j];
    std::vector<double> cumulative(last - first + 1, 0.0);
    for (std::size_t i = first + 1; i <= last; ++i) {
      check();
      cumulative[i - first] = cumulative[i - first - 1] + (path[i] - path[i - 1]).norm();
    }
    const double length = cumulative.back();
    const double needed = std::ceil(length / options.control_spacing);
    if (!std::isfinite(needed) || needed > static_cast<double>(options.max_control_points)) {
      throw StopOptimization{"control_point_budget"};
    }
    const auto segments = static_cast<std::size_t>(needed);
    if (controls.size() + segments > options.max_control_points)
    {
      throw StopOptimization{"control_point_budget"};
    }
    for (std::size_t k = 1; k < segments; ++k) {
      check();
      const double distance = length * static_cast<double>(k) / static_cast<double>(segments);
      const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), distance);
      const std::size_t i = std::min<std::size_t>(
        std::distance(cumulative.begin(), upper), cumulative.size() - 1);
      const double alpha = (distance - cumulative[i - 1]) / (cumulative[i] - cumulative[i - 1]);
      controls.push_back(path[first + i - 1] + alpha * (path[first + i] - path[first + i - 1]));
    }
    controls.push_back(path[last]);
  }
  return controls;
}
}  // namespace

void validateTrajectoryOptions(const TrajectoryOptimizationOptions & options)
{
  if (!std::isfinite(options.max_time_seconds) || options.max_time_seconds <= 0.0 ||
    options.max_control_points < 3 || options.max_control_points > 24 ||
    options.max_output_points < options.max_control_points || options.max_output_points > 10000 ||
    !std::isfinite(options.control_spacing) || options.control_spacing <= 0.0 ||
    !std::isfinite(options.corner_tolerance) || options.corner_tolerance <= 0.0 ||
    !std::isfinite(options.sample_spacing) || options.sample_spacing <= 0.0 ||
    !std::isfinite(options.max_deviation) || options.max_deviation <= 0.0 ||
    !std::isfinite(options.max_length_ratio) || options.max_length_ratio < 1.0 ||
    !std::isfinite(options.time_allocation_speed) || options.time_allocation_speed <= 0.0 ||
    !std::isfinite(options.time_allocation_accel) || options.time_allocation_accel <= 0.0)
  {
    throw std::invalid_argument("invalid TDT trajectory optimization options");
  }
}

TrajectoryOptimizationResult optimizeTdtTrajectory(
  const GeometricPath & validated_path, const MotionIsFree & motion_is_free,
  const TrajectoryOptimizationOptions & options, const std::function<bool()> & cancelled)
{
  validateTrajectoryOptions(options);
  TrajectoryOptimizationResult result;
  result.points = validated_path;
  const auto begin = Clock::now();
  const auto finish = [&](const char * reason) {
    result.reason = reason;
    result.elapsed_seconds = std::chrono::duration<double>(Clock::now() - begin).count();
    return result;
  };
  if (!options.enabled) return finish("disabled");
  if (!motion_is_free) throw std::invalid_argument("missing complete-body motion validator");
  const auto check = [&]() {
    if (cancelled && cancelled()) throw StopOptimization{"cancelled"};
    if (std::chrono::duration<double>(Clock::now() - begin).count() >= options.max_time_seconds) {
      throw StopOptimization{"time_budget"};
    }
  };
  try {
    check();
    if (validated_path.size() > 20000) return finish("input_point_budget");
    GeometricPath path;
    path.reserve(validated_path.size());
    for (const auto & p : validated_path) {
      check();
      if (!p.allFinite()) throw std::invalid_argument("nonfinite trajectory input");
      if (path.empty() || (p - path.back()).norm() > 1.0e-9) path.push_back(p);
    }
    if (path.size() < 3) return finish("short_path");
    const auto controls = makeControls(path, options, check);
    result.control_points = controls.size();
    if (controls.size() < 3) return finish("straight_path");
    std::vector<Eigen::Vector2f> float_controls;
    std::vector<TdtClosedMinimumJerk::Item> items;
    for (const auto & p : controls) {
      check();
      TdtClosedMinimumJerk::Item item;
      item.xy = p.cast<float>();
      if (!item.xy.allFinite()) return finish("numeric_failure");
      float_controls.push_back(item.xy);
      items.push_back(item);
    }
    // Fix endpoint tangent velocities using the supported upstream Item
    // constraints. These condition path geometry; current control remains Nav2's.
    items.front().useVxvy = items.back().useVxvy = true;
    items.front().vxvy = ((controls[1] - controls[0]).normalized() *
      options.time_allocation_speed).cast<float>();
    items.back().vxvy = ((controls.back() - controls[controls.size() - 2]).normalized() *
      options.time_allocation_speed).cast<float>();
    const auto times = TdtClosedMinimumJerk::trapezoidalTimeAllocation(
      float_controls, static_cast<float>(options.time_allocation_speed),
      static_cast<float>(options.time_allocation_accel));
    check();
    // One bounded dense factorization is indivisible. Reject a late answer.
    const auto coefficients = TdtClosedMinimumJerk{}.closeSolver(times, items, true);
    check();
    if (coefficients.first.rows() != static_cast<Eigen::Index>(times.size()) ||
      coefficients.first.cols() != 6 || !coefficients.first.allFinite() ||
      coefficients.second.rows() != coefficients.first.rows() ||
      coefficients.second.cols() != 6 || !coefficients.second.allFinite())
    {
      return finish("numeric_failure");
    }
    GeometricPath candidate{validated_path.front()};
    for (std::size_t segment = 0; segment < times.size(); ++segment) {
      const double intervals = std::max(10.0,
        std::ceil((controls[segment + 1] - controls[segment]).norm() / options.sample_spacing));
      if (intervals > static_cast<double>(options.max_output_points) ||
        candidate.size() + static_cast<std::size_t>(intervals) > options.max_output_points)
      {
        return finish("output_point_budget");
      }
      for (std::size_t j = 1; j <= static_cast<std::size_t>(intervals); ++j) {
        check();
        const double u = static_cast<double>(j) / intervals;
        Eigen::Vector2d p = Eigen::Vector2d::Zero();
        for (Eigen::Index power = 0; power < 6; ++power) {
          p.x() = p.x() * u + coefficients.first(segment, power);
          p.y() = p.y() * u + coefficients.second(segment, power);
        }
        if (!p.allFinite()) return finish("numeric_failure");
        if (segment + 1 == times.size() && j == static_cast<std::size_t>(intervals)) {
          p = validated_path.back();
        }
        if ((p - candidate.back()).norm() > 1.0e-9) candidate.push_back(p);
      }
    }
    // Every returned edge receives the same swept-body checker as YAstar. The
    // endpoint replacement happens before this check; no float-only exception.
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      check();
      if (!motion_is_free(candidate[i == 0 ? 0 : i - 1], candidate[i])) {
        return finish("body_collision");
      }
      double deviation = std::numeric_limits<double>::infinity();
      for (std::size_t j = 1; j < path.size(); ++j) {
        if ((j & 63U) == 0) check();
        deviation = std::min(deviation,
          pointSegmentDistance(candidate[i], path[j - 1], path[j]));
      }
      if (deviation > options.max_deviation) return finish("deviation_limit");
    }
    check();
    if (polylineLength(candidate) > options.max_length_ratio * polylineLength(path)) {
      return finish("length_limit");
    }
    result.points = std::move(candidate);
    result.applied = true;
    return finish("applied");
  } catch (const StopOptimization & stop) {
    return finish(stop.reason);
  }
}
}  // namespace tdt_yastar_planner
