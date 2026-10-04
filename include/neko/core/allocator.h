#ifndef NEKO_CORE_ALLOCATOR_H_
#define NEKO_CORE_ALLOCATOR_H_

#include <cstddef>
#include <limits>
#include <vector>

namespace neko
{
class Allocator
{
public:
    virtual ~Allocator() = default;
    template<typename T>
    T* Allocate(std::size_t count)
    {
        return static_cast<T*>(Allocate(count, sizeof(T)));
    }
    virtual void* Allocate(std::size_t count, std::size_t type_size) = 0;
    virtual void Deallocate(void* ptr) = 0;
};

class HeapAllocator final : public Allocator
{
public:
    void* Allocate(std::size_t count, std::size_t type_size) override;

    void Deallocate(void *ptr) override;

};

/**
 * @brief Process-wide heap allocator used when none is given
 */
[[nodiscard]] Allocator& DefaultAllocator() noexcept;

namespace detail
{
/**
 * @brief Throws std::bad_alloc, or aborts when exceptions are disabled (usual on the DS)
 */
[[noreturn]] void OnAllocationFailure();
} // namespace detail

/**
 * \brief Proxy allocator respecting allocator_traits. Holds a pointer, so containers can be copied and moved freely
 * as long as the Allocator outlives them.
 */
template<typename T>
class StandardAllocator
{
public:
    // NOLINTNEXTLINE(readability-identifier-naming): allocator requirements
    using value_type = T;
    StandardAllocator() noexcept : allocator_(&DefaultAllocator()) {}
    explicit StandardAllocator(Allocator& allocator) noexcept : allocator_(&allocator) {}
    template <class U>
    StandardAllocator(const StandardAllocator<U>& allocator) noexcept : allocator_(&allocator.GetAllocator()) {}

    // NOLINTNEXTLINE(readability-identifier-naming): allocator requirements
    [[nodiscard]] T* allocate(std::size_t n)
    {
        if (n > std::numeric_limits<std::size_t>::max() / sizeof(T))
        {
            detail::OnAllocationFailure();
        }
        auto* ptr = allocator_->Allocate(n, sizeof(T));
        if (ptr == nullptr && n != 0)
        {
            detail::OnAllocationFailure();
        }
        return static_cast<T*>(ptr);
    }

    // NOLINTNEXTLINE(readability-identifier-naming): allocator requirements
    void deallocate(T* ptr, [[maybe_unused]] std::size_t n) noexcept
    {
        allocator_->Deallocate(ptr);
    }

    [[nodiscard]] Allocator& GetAllocator() const noexcept { return *allocator_; }

    template <class U>
    [[nodiscard]] bool operator==(const StandardAllocator<U>& other) const noexcept
    {
        return allocator_ == &other.GetAllocator();
    }
private:
    Allocator* allocator_;
};

template<typename T>
using ArrayList = std::vector<T, StandardAllocator<T>>;
}

#endif  // NEKO_CORE_ALLOCATOR_H_
