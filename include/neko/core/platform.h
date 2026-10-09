#ifndef NEKO_CORE_PLATFORM_H_
#define NEKO_CORE_PLATFORM_H_

// Marks a hot function definition: placed in the ARM9 instruction TCM on the DS
// (32 KB, no wait states, not behind the instruction cache), nothing elsewhere.
//
// TCM (tightly coupled memory) is a tiny on-chip RAM the CPU reads at full
// speed, while main RAM is slow and the 8 KB instruction cache can miss. Put
// it only on the functions that run every step (for example
// `NEKO_HOT void SweepAndPrune::CalculatePairs()`): the 32 KB fill up fast.
// MrDocs does not extract macros, so this stays a plain comment.
#if defined(NEKO_PLATFORM_NDS)
#include <nds/ndstypes.h>
#define NEKO_HOT ITCM_CODE
#else
#define NEKO_HOT
#endif

#endif  // NEKO_CORE_PLATFORM_H_
