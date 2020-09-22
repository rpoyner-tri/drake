#pragma once

/// @file
/// This is the entry point for all operations with spatial vectors.
/// Spatial vectors represent spatial physical quantities such as spatial
/// velocities, spatial accelerations and spatial forces. Spatial vectors are
/// 6-element quantities that are pairs of ordinary 3-vectors. Elements 0-2 are
/// always the rotational component while elements 3-5 are always the
/// translational component.
/// For a more detailed introduction on spatial vectors please refer to
/// section @ref multibody_spatial_vectors.

#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_momentum.h"
#include "drake/multibody/math/spatial_velocity.h"

// The specializations below enable only the supported combinations of dot
// products.

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
struct Dot<SpatialVelocity, SpatialForce, T> {
  T operator()(const SpatialVector<SpatialVelocity, T>& a,
               const SpatialVector<SpatialForce, T>& b) {
    return do_dot_permitted(a, b);
  }
};

template <typename T>
struct Dot<SpatialForce, SpatialVelocity, T> {
  T operator()(const SpatialVector<SpatialForce, T>& a,
               const SpatialVector<SpatialVelocity, T>& b) {
    return do_dot_permitted(a, b);
  }
};

template <typename T>
struct Dot<SpatialVelocity, SpatialMomentum, T> {
  T operator()(const SpatialVector<SpatialVelocity, T>& a,
               const SpatialVector<SpatialMomentum, T>& b) {
    return do_dot_permitted(a, b);
  }
};

template <typename T>
struct Dot<SpatialMomentum, SpatialVelocity, T> {
  T operator()(const SpatialVector<SpatialMomentum, T>& a,
               const SpatialVector<SpatialVelocity, T>& b) {
    return do_dot_permitted(a, b);
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
