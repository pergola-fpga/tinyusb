#ifndef UF2_H
#define UF2_H 1

#include "board.h"
#include "uf2_version.h"

#define UF2_VERSION                                                                                \
    UF2_VERSION_BASE " SF"

// needs to be more than ~4200 (to force FAT16)
#define NUM_FAT_BLOCKS 64000

#define UF2_PAYLOAD_SIZE 256
#define UF2_BLOCK_SIZE 512

#define DBL_TAP_MAGIC 0xf01669ef // Randomly selected, adjusted to have first and last bit set
#define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef

//! Static block size for all memories
#define UDI_MSC_BLOCK_SIZE 512L

#define CONCAT_1(a, b) a##b
#define CONCAT_0(a, b) CONCAT_1(a, b)
#define STATIC_ASSERT(e) enum { CONCAT_0(_static_assert_, __LINE__) = 1 / ((e) ? 1 : 0) }

#define MAX_BLOCKS (BOARD_FLASH_SIZE / UF2_PAYLOAD_SIZE + 100)

typedef struct {
    uint32_t numBlocks;
    uint32_t numWritten;
    uint8_t writtenMask[MAX_BLOCKS / 8 + 1];
} WriteState;

extern uint32_t reset_millis;

#endif
