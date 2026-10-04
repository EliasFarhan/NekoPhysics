#include "neko/core/allocator.h"

#include <cstdlib>
#include <new>

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

namespace neko
{

void* HeapAllocator::Allocate(std::size_t count, std::size_t type_size)
{
    const auto size = count*type_size;
    auto* ptr = std::malloc(size);
#ifdef TRACY_ENABLE
    TracyAlloc(ptr, size);
#endif
    return ptr;
}

void HeapAllocator::Deallocate(void *ptr)
{
#ifdef TRACY_ENABLE
    TracyFree(ptr);
#endif
    std::free(ptr);
}

Allocator& DefaultAllocator() noexcept
{
    static HeapAllocator allocator;
    return allocator;
}

namespace internal
{
void OnAllocationFailure()
{
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    throw std::bad_alloc();
#else
    std::abort();
#endif
}
}  // namespace internal
}  // namespace neko
