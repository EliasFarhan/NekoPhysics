#ifndef NEKO_CORE_ALLOCATOR_H_
#define NEKO_CORE_ALLOCATOR_H_

#include <cstddef>
#include <limits>
#include <vector>

namespace neko {
/// Interface for the memory sources used by the library.
///
/// Every container of NekoPhysics (@ref ArrayList) gets its memory through
/// an `Allocator` instead of calling `new` directly. A game can then decide
/// where the physics memory lives: a fixed arena carved out of the DS's
/// 4 MB of RAM, a tracking allocator to measure usage, or simply the heap
/// (@ref HeapAllocator, the default). The library allocates only at
/// construction; @ref PhysicsWorld::Step and @ref PhysicsWorld::CopyFrom never
/// do, which keeps frame times stable and rollback resimulations cheap.
///
/// To write your own, derive from this class and implement the two virtual
/// functions:
///
/// @code
/// class ArenaAllocator final : public neko::Allocator {
///  public:
///   void* Allocate(std::size_t count, std::size_t type_size) override {
///     const std::size_t size = (count * type_size + 7) & ~std::size_t{7};
///     if (used_ + size > sizeof(buffer_)) return nullptr;
///     void* ptr = buffer_ + used_;
///     used_ += size;
///     return ptr;
///   }
///   void Deallocate(void*) override {}  // freed all at once
///
///  private:
///   alignas(8) unsigned char buffer_[256 * 1024];
///   std::size_t used_ = 0;
/// };
///
/// ArenaAllocator arena;  // must outlive the world
/// neko::PhysicsWorldConfig config;
/// config.allocator = &arena;
/// neko::PhysicsWorld world{config};
/// @endcode
class Allocator {
 public:
  /// Virtual destructor, so allocators can be deleted through the base.
  virtual ~Allocator() = default;
  /// Allocates uninitialized memory for @p count objects of type `T`.
  ///
  /// It does not construct the objects.
  ///
  /// @tparam T Type of the objects.
  /// @param count Number of objects.
  /// @return The memory, or nullptr on failure.
  template <typename T>
  T* Allocate(std::size_t count) {
    return static_cast<T*>(Allocate(count, sizeof(T)));
  }
  /// Allocates uninitialized memory for @p count objects of @p type_size
  /// bytes each.
  ///
  /// @param count Number of objects.
  /// @param type_size Size of one object in bytes.
  /// @return Memory aligned for objects of that size, or nullptr on
  /// failure (@ref StandardAllocator then aborts).
  virtual void* Allocate(std::size_t count, std::size_t type_size) = 0;
  /// Releases memory returned by @ref Allocate.
  ///
  /// @param ptr Pointer returned by this allocator, or nullptr.
  virtual void Deallocate(void* ptr) = 0;
};

/// @ref Allocator backed by `std::malloc` and `std::free`.
///
/// When Tracy profiling is enabled (`ENABLE_PROFILING`), it also reports
/// every allocation to Tracy's memory view.
class HeapAllocator final : public Allocator {
 public:
  /// Allocates `count * type_size` bytes with `std::malloc`.
  ///
  /// @param count Number of objects.
  /// @param type_size Size of one object in bytes.
  /// @return The memory, or nullptr when `std::malloc` fails.
  void* Allocate(std::size_t count, std::size_t type_size) override;

  /// Releases memory with `std::free`.
  ///
  /// @param ptr Pointer returned by @ref Allocate, or nullptr.
  void Deallocate(void* ptr) override;
};

/// Returns the process-wide @ref HeapAllocator used when none is given.
///
/// It is a function-local static, created on first use and never destroyed
/// before the end of the program.
///
/// @return The default allocator, the same object on every call.
[[nodiscard]] Allocator& DefaultAllocator() noexcept;

namespace internal {
// Aborts: NekoPhysics does not use exceptions (Google style)
[[noreturn]] void OnAllocationFailure();
}  // namespace internal

/// Adapter that lets standard containers allocate through an @ref Allocator.
///
/// Standard containers such as `std::vector` take their allocator as a
/// template parameter that must follow the C++ "Allocator" requirements
/// (`value_type`, `allocate`, `deallocate`, a rebinding constructor and
/// `==`). This class meets them and forwards every request to an
/// @ref Allocator. It only holds a pointer, so containers can be copied and
/// moved freely as long as the @ref Allocator outlives them.
///
/// Since the library is built without exceptions, an allocation failure
/// aborts the program instead of throwing `std::bad_alloc`.
///
/// @code
/// neko::ArrayList<int> values{neko::StandardAllocator<int>{my_allocator}};
/// values.reserve(128);  // allocates through my_allocator
/// @endcode
///
/// @tparam T Type of the objects allocated.
template <typename T>
class StandardAllocator {
 public:
  /// Type of the objects allocated (required by the standard).
  // NOLINTNEXTLINE(readability-identifier-naming): allocator requirements
  using value_type = T;
  /// Builds an adapter for @ref DefaultAllocator.
  StandardAllocator() noexcept : allocator_(&DefaultAllocator()) {}
  /// Builds an adapter for `allocator`.
  ///
  /// @param allocator @ref Allocator to forward to; must outlive the adapter
  /// and every container using it.
  explicit StandardAllocator(Allocator& allocator) noexcept
      : allocator_(&allocator) {}
  /// Builds an adapter for another type sharing the same @ref Allocator.
  ///
  /// Implicit on purpose: containers "rebind" their allocator to allocate
  /// internal types (nodes, for example), and the standard requires this
  /// conversion.
  ///
  /// @tparam U Value type of the other adapter.
  template <class U>
  // NOLINTNEXTLINE(google-explicit-constructor)
  StandardAllocator(const StandardAllocator<U>& allocator) noexcept
      : allocator_(&allocator.GetAllocator()) {}

  /// Allocates uninitialized memory for @p n objects of type `T`.
  ///
  /// @param n Number of objects.
  /// @return The memory; never nullptr when @p n is not zero.
  /// @note Aborts when the size overflows or the @ref Allocator returns
  /// nullptr.
  // NOLINTNEXTLINE(readability-identifier-naming): allocator requirements
  [[nodiscard]] T* allocate(std::size_t n) {
    if (n > std::numeric_limits<std::size_t>::max() / sizeof(T)) {
      internal::OnAllocationFailure();
    }
    auto* ptr = allocator_->Allocate(n, sizeof(T));
    if (ptr == nullptr && n != 0) {
      internal::OnAllocationFailure();
    }
    return static_cast<T*>(ptr);
  }

  /// Releases memory returned by @ref allocate.
  ///
  /// @param ptr Pointer returned by @ref allocate.
  /// @param n Number of objects passed to @ref allocate (unused).
  // NOLINTNEXTLINE(readability-identifier-naming): allocator requirements
  void deallocate(T* ptr, [[maybe_unused]] std::size_t n) noexcept {
    allocator_->Deallocate(ptr);
  }

  /// Returns the @ref Allocator this adapter forwards to.
  ///
  /// @return The allocator given to the constructor.
  [[nodiscard]] Allocator& GetAllocator() const noexcept { return *allocator_; }

  /// Returns true when both adapters forward to the same @ref Allocator.
  ///
  /// Equal allocators can free each other's memory, which lets containers
  /// swap or move their buffers instead of copying them.
  ///
  /// @tparam U Value type of the other adapter.
  template <class U>
  [[nodiscard]] bool operator==(
      const StandardAllocator<U>& other) const noexcept {
    return allocator_ == &other.GetAllocator();
  }

 private:
  Allocator* allocator_;
};

/// `std::vector` allocating through an @ref Allocator.
///
/// The library uses it for every array. Pass a @ref StandardAllocator for `T`
/// to the constructor to choose the @ref Allocator (the default one otherwise),
/// and `reserve` the capacity up front so that adding elements later never
/// allocates.
///
/// @tparam T Element type.
template <typename T>
using ArrayList = std::vector<T, StandardAllocator<T>>;
}  // namespace neko

#endif  // NEKO_CORE_ALLOCATOR_H_
