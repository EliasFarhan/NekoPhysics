#include "main/samples/sample.h"

#include <algorithm>
#include <array>
#include <utility>

#include "neko/core/profile.h"
#include "neko/physics/physics.h"

#include "main/samples/01_kinematic.h"
#include "main/samples/02_planets.h"
#include "main/samples/03_triggers.h"
#include "main/samples/04_ground.h"
#include "main/samples/05_collisions.h"
#include "main/samples/06_move_into.h"

namespace neko {

void BounceInside(Body& body, const Aabbf& bounds) {
  if ((body.velocity.x() > Scalar{0} &&
       body.position.x() > bounds.max_bound().x()) ||
      (body.velocity.x() < Scalar{0} &&
       body.position.x() < bounds.min_bound().x())) {
    body.velocity.set_x(-body.velocity.x());
  }
  if ((body.velocity.y() > Scalar{0} &&
       body.position.y() > bounds.max_bound().y()) ||
      (body.velocity.y() < Scalar{0} &&
       body.position.y() < bounds.min_bound().y())) {
    body.velocity.set_y(-body.velocity.y());
  }
}

int ToDecimal(Scalar value, int scale) {
  const Scalar scaled = value * static_cast<Scalar>(scale);
  // The conversion truncates toward zero: add half away from zero first
  const Scalar half = Ratio(1, 2);
  return static_cast<int>(scaled < Scalar{0} ? scaled - half : scaled + half);
}

std::size_t FormatDecimal(int numerator, int scale, std::span<char> buffer) {
  // Digits of |numerator| with a decimal point before the last `decimals`
  // ones, built backwards; unsigned so that INT_MIN has a magnitude
  int decimals = 0;
  for (int rest = scale; rest >= 10; rest /= 10) {
    ++decimals;
  }
  const bool is_negative = numerator < 0;
  auto magnitude = static_cast<unsigned>(numerator);
  if (is_negative) {
    magnitude = 0u - magnitude;
  }
  std::array<char, 16> reversed{};
  std::size_t length = 0;
  for (int digit = 0; digit <= decimals || magnitude != 0; ++digit) {
    if (digit == decimals && decimals > 0) {
      reversed[length++] = '.';
    }
    reversed[length++] = static_cast<char>('0' + magnitude % 10u);
    magnitude /= 10u;
  }
  if (is_negative) {
    reversed[length++] = '-';
  }
  const std::size_t written = std::min(length, buffer.size());
  for (std::size_t i = 0; i < written; ++i) {
    buffer[i] = reversed[length - 1 - i];
  }
  return written;
}

SampleRegistry::SampleRegistry() {
  samples_.reserve(6);
  samples_.push_back({"01 Kinematic", std::make_unique<KinematicSample>()});
  samples_.push_back({"02 Planets", std::make_unique<PlanetSample>()});
  samples_.push_back({"03 Triggers", std::make_unique<TriggersSample>()});
  samples_.push_back({"04 Ground", std::make_unique<GroundSample>()});
  samples_.push_back({"05 Collisions", std::make_unique<CollisionsSample>()});
  samples_.push_back({"06 Move Into", std::make_unique<MoveIntoSample>()});
}

SampleRegistry::SampleRegistry(std::vector<Entry> samples)
    : samples_(std::move(samples)) {}

SampleRegistry::~SampleRegistry() { Select(kNone, context_); }

void SampleRegistry::Select(std::size_t index, const SampleContext& context) {
  if (current_index_ != kNone) {
    samples_[current_index_].sample->End();
  }
  current_index_ = index < samples_.size() ? index : kNone;
  context_ = context;
  if (current_index_ != kNone) {
    samples_[current_index_].sample->Begin(context_);
  }
}

void SampleRegistry::FixedUpdate() {
  NEKO_PROFILE_FUNCTION();
  if (current_index_ != kNone) {
    samples_[current_index_].sample->FixedUpdate();
  }
}

std::span<const SampleParameter> SampleRegistry::parameters() const {
  Sample* sample = current();
  return sample == nullptr ? std::span<const SampleParameter>{}
                           : sample->parameters();
}

void SampleRegistry::SetParameter(std::size_t parameter_index, int value) {
  Sample* sample = current();
  if (sample == nullptr) {
    return;
  }
  const auto sample_parameters = sample->parameters();
  if (parameter_index >= sample_parameters.size()) {
    return;
  }
  SampleParameter& parameter = sample_parameters[parameter_index];
  const int step = std::max(parameter.step, 1);
  const int clamped = std::clamp(value, parameter.min, parameter.max);
  parameter.value = parameter.min + (clamped - parameter.min) / step * step;
  Select(current_index_, context_);
}

std::span<const SampleReadout> SampleRegistry::readouts() const {
  const Sample* sample = current();
  return sample == nullptr ? std::span<const SampleReadout>{}
                           : sample->readouts();
}

void SampleRegistry::Draw(DrawList& draw_list) const {
  NEKO_PROFILE_FUNCTION();
  draw_list.Clear();
  draw_list.camera = context_.world_bounds;
  if (current_index_ != kNone) {
    samples_[current_index_].sample->Draw(draw_list);
  }
}

}  // namespace neko
