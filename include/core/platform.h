#pragma once

/**
 * @brief Marks a hot function definition: placed in the ARM9 instruction TCM on the DS (32 KB, no wait states,
 * not behind the instruction cache), nothing elsewhere.
 */
#if defined(NEKO_PLATFORM_NDS)
#include <nds/ndstypes.h>
#define NEKO_HOT ITCM_CODE
#else
#define NEKO_HOT
#endif
