#include "core/allocator.h"

#ifdef TRACY_ENABLE
#include <Tracy.hpp>
#endif

namespace neko
{

void* HeapAllocator::Allocate(std::size_t count, std::size_t typeSize)
{
    const auto size = count*typeSize;
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
}