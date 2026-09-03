/* Shim over the firmware's APDB.h (found next via the include path).
 * APPL_START is (ubyte4)&_cstart, the downloader's jump address; a 64-bit
 * host address cannot be a 32-bit constant initializer, and no downloader
 * ever reads the host binary's APDB. Everything else is the real header. */
#include_next "APDB.h"

#undef APPL_START
#define APPL_START 0
