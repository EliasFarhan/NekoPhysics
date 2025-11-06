#include "core/allocator.h"

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

namespace neko
{

void* HeapAllocator::Allocate(std::size_t count, std::size_t typeSize)
{
    const auto size = count*typeSize;
    auto* ptr = operator new(size);
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
    operator delete(ptr);
}
}