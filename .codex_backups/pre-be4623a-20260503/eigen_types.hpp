#ifndef _TRAJ_UTILS_EIGEN_TYPES_HPP_
#define _TRAJ_UTILS_EIGEN_TYPES_HPP_

#include <Eigen/Eigen>
#include <vector>

namespace ego_planner
{

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using Vec3dList = AlignedVector<Eigen::Vector3d>;
using Vec3dListList = std::vector<Vec3dList>;

} // namespace ego_planner

#endif
