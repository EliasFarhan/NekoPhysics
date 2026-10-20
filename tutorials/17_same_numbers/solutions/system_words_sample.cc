#include "tutorials/17_same_numbers/solutions/system_words_sample.h"

namespace neko::tutorial {
namespace {
// FNV-1a over 32-bit words
constexpr std::uint32_t kFnvOffset = 2166136261u;
std::uint32_t Mix(std::uint32_t hash, std::uint32_t word) {
  for (int byte = 0; byte < 4; ++byte) {
    hash ^= (word >> (8 * byte)) & 0xFFu;
    hash *= 16777619u;
  }
  return hash;
}
}  // namespace

SystemWords ComputeSystemWords(const Board& board) {
  const PhysicsWorld& world = board.world();
  SystemWords words{.colliders = kFnvOffset,
                    .bodies = kFnvOffset,
                    .touching = kFnvOffset,
                    .world = world.GenerateChecksum()};
  // One collider per body, with the body's index
  for (int i = 0; i < board.body_count(); ++i) {
    const auto index = static_cast<Index>(i);
    words.bodies =
        Mix(words.bodies, GenerateChecksum(world.body(BodyIndex{index})));
    words.colliders =
        Mix(words.colliders,
            GenerateChecksum(world.collider(ColliderIndex{index})));
  }
  for (const TouchingPair& pair : world.touching_pairs()) {
    words.touching = Mix(words.touching, pair.pair.c1().index());
    words.touching = Mix(words.touching, pair.pair.c2().index());
  }
  return words;
}

void SystemWordsSample::Begin(const SampleContext& context) {
  board_a_.Build(context.world_bounds, 0);
  board_b_.Build(context.world_bounds, 0);
  Collider& peg = board_b_.world().collider(board_b_.top_peg());
  peg.restitution = NudgeBits(peg.restitution, parameters_[kNudge].value);
  watch_.Reset();
  colliders_step_.reset();
  bodies_step_.reset();
  touching_step_.reset();
  world_step_.reset();
  // Before the first step: the level as built
  Compare(0);
}

void SystemWordsSample::Compare(int step) {
  const SystemWords a = ComputeSystemWords(board_a_);
  const SystemWords b = ComputeSystemWords(board_b_);
  const auto note = [step](std::optional<int>& first, bool differs) {
    if (!first.has_value() && differs) {
      first = step;
    }
  };
  note(colliders_step_, a.colliders != b.colliders);
  note(bodies_step_, a.bodies != b.bodies);
  note(touching_step_, a.touching != b.touching);
  note(world_step_, a.world != b.world);
  const auto set_step = [](SampleReadout& readout, std::optional<int> value) {
    readout.is_valid = value.has_value();
    readout.value = value.value_or(0);
  };
  set_step(readouts_[0], colliders_step_);
  set_step(readouts_[1], bodies_step_);
  set_step(readouts_[2], touching_step_);
  set_step(readouts_[3], world_step_);
  set_step(readouts_[4], watch_.first_visible_step());
}

void SystemWordsSample::FixedUpdate() {
  board_a_.Step();
  board_b_.Step();
  watch_.Update(board_a_, board_b_);
  // Step n's words are compared after n steps
  Compare(board_a_.step());
}

void SystemWordsSample::Draw(DrawList& draw_list) const {
  board_a_.DrawStatic(draw_list);
  // The changed peg, in red
  const Body& peg = board_b_.world().body(
      board_b_.world().collider(board_b_.top_peg()).body_index);
  draw_list.circles.push_back(
      {peg.position, Board::kPegRadius, sample_color::kRed});
  board_a_.DrawBalls(draw_list, sample_color::kSky, Scalar{1});
  board_b_.DrawBalls(draw_list, sample_color::kOrange, Ratio(3, 5));
}

void SystemWordsSample::End() {
  board_a_.Clear();
  board_b_.Clear();
}

std::uint32_t SystemWordsSample::GenerateChecksum() const {
  return board_a_.world().GenerateChecksum() * 31u +
         board_b_.world().GenerateChecksum();
}

}  // namespace neko::tutorial
