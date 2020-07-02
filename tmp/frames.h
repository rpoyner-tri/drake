#include <deque>
#include <optional>
#include <string>
#include <tuple>

#include "drake/common/drake_assert.h"

namespace drake {
namespace tmp {

class Frames {
 public:
  int add(double t, const std::string& value);
  int add(const std::string& value);
  int size() const { return static_cast<int>(times_.size()); }
  std::tuple<double, std::string> get(int i) const;
  std::optional<int> first_diff(const Frames& other) const;
  bool operator==(const Frames& other) const { return !first_diff(other); }
  size_t hash() const;
  std::string text_diff(const Frames& other) const;

  class Current {
   public:
    Current(Frames* current) {
      DRAKE_ASSERT(instance_ == nullptr);
      instance_ = current;
    }

    ~Current() {
      DRAKE_ASSERT(instance_ != nullptr);
      instance_ = nullptr;
    }

    static Frames* the() {
      DRAKE_ASSERT(instance_ != nullptr);
      return instance_;
    }

   private:
    static Frames* instance_;
  };

 private:
  std::deque<double> times_;
  std::deque<std::string> values_;
};
}
}

namespace std {
template<>
struct hash<drake::tmp::Frames> {
  size_t operator()(const drake::tmp::Frames& x) const {
    return x.hash();
  }
};
}

