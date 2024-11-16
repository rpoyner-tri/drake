#pragma once

#include <memory>
#include <string>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"

namespace drake {

/// Checks if @p ptr, a pointer to `FromType` class, can be safely converted to
/// a pointer to `ToType` class. Our motivation is to provide a good diagnostic
/// for what @p ptr _actually_ was when the test fails.
///
/// Here is an illustrative code snippet. We assume that `Prius` and `Camry` are
/// derived classes of `Car`.
///
/// @code
/// const Car* prius = new Prius{};
///
/// // The following assertion fails without diagnostic info.
/// EXPECT_TRUE(dynamic_cast<Camry>(ptr) != nullptr)
///
/// // The following assertion returns `::testing::AssertionFailure()` with
/// // diagnostic info attached.
/// EXPECT_TRUE(is_dynamic_castable<Camry>(prius));
/// // Output:
/// // Value of: is_dynamic_castable<Camry>(prius)
/// // Actual: false (is_dynamic_castable<Camry>(Car* ptr) failed
/// //                because ptr is of dynamic type Prius.)
/// // Expected: true
/// @endcode
template <typename ToType, typename FromType>
[[nodiscard]] std::string is_dynamic_castable(
    const FromType* ptr) {
  if (ptr == nullptr) {
    const std::string from_name{NiceTypeName::Get<FromType>()};
    const std::string to_name{NiceTypeName::Get<ToType>()};
    return fmt::format("is_dynamic_castable<{}>({}* ptr) failed because"
		       " ptr was already nullptr.",
		       to_name, from_name);
  }
  if (dynamic_cast<const ToType* const>(ptr) == nullptr) {
    const std::string from_name{NiceTypeName::Get<FromType>()};
    const std::string to_name{NiceTypeName::Get<ToType>()};
    const void* to_ptr = &typeid(ToType);
    const std::string dynamic_name{NiceTypeName::Get(*ptr)};
    const void* dynamic_ptr = &typeid(*ptr);
    return fmt::format("is_dynamic_castable<{}@{}>({}* ptr) failed"
		       " because ptr is of dynamic type {}@{}.",
		       to_name, fmt::ptr(to_ptr), from_name,
		       dynamic_name, dynamic_ptr);
  }
  return {};
}

/// Checks if @p ptr, a shared pointer to `FromType` class, can be safely
/// converted to a shared pointer to `ToType` class. Our motivation is to
/// provide a good diagnostic for what @p ptr _actually_ was when the test
/// fails.
template <typename ToType, typename FromType>
[[nodiscard]] std::string is_dynamic_castable(
    std::shared_ptr<FromType> ptr) {
  return is_dynamic_castable<ToType, FromType>(ptr.get());
}

/// Checks if @p ptr, a shared pointer to `FromType` class, can be safely
/// converted to a shared pointer to `ToType` class. Our motivation is to
/// provide a good diagnostic for what @p ptr _actually_ was when the test
/// fails.
template <typename ToType, typename FromType>
[[nodiscard]] std::string is_dynamic_castable(
    const std::unique_ptr<FromType>& ptr) {
  return is_dynamic_castable<ToType, FromType>(ptr.get());
}

}  // namespace drake
