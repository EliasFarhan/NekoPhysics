#pragma once

#include <cstdlib>
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
    virtual void* Allocate(std::size_t count, std::size_t typeSize) = 0;
    virtual void Deallocate(void* ptr) = 0;
};

class HeapAllocator final : public Allocator
{
public:
    void* Allocate(std::size_t count, std::size_t typeSize) override;

    void Deallocate(void *ptr) override;

};

/**
 * \brief Custom proxy allocator respecting allocator_traits
 */
template<typename T>
class StandardAllocator
{
public:
    using value_type = T;
    explicit StandardAllocator(Allocator& allocator);
    template <class U>
    explicit StandardAllocator(const StandardAllocator<U>& allocator) noexcept : allocator_(allocator.GetAllocator()) {}
    T* allocate(std::size_t n);
    void deallocate(T* ptr, std::size_t n);
    [[nodiscard]] Allocator& GetAllocator() const { return allocator_; }
protected:
    Allocator& allocator_;
};


template <class T, class U>
constexpr bool operator== (const StandardAllocator<T>&, const StandardAllocator<U>&) noexcept
{
    return true;
}

template <class T, class U>
constexpr bool operator!= (const StandardAllocator<T>&, const StandardAllocator<U>&) noexcept
{
    return false;
}

template <typename T>
StandardAllocator<T>::StandardAllocator(Allocator& allocator) : allocator_(allocator)
{
}

template <typename T>
T* StandardAllocator<T>::allocate(std::size_t n)
{
    return static_cast<T*>(allocator_.Allocate(n * sizeof(T), alignof(T)));
}

template <typename T>
void StandardAllocator<T>::deallocate(T* ptr, [[maybe_unused]] std::size_t n)
{
    allocator_.Deallocate(ptr);
}
}

template<typename T>
using ArrayList = std::vector<T, neko::StandardAllocator<T>>;