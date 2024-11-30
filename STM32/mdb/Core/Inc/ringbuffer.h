#include <stdint.h>

typedef struct ringbuffer_s {
    uint8_t *data;
    uint32_t max_size;
    uint32_t start;
    uint32_t size;
    int32_t nb_overflows;
} ringbuffer_t;

void ringbuffer_init(ringbuffer_t* buffer, uint8_t* data, uint32_t max_size);

// The number of bytes written inside the DMA buffer from the index 0
void ringbuffer_dma_set_written_size(ringbuffer_t* buffer, uint32_t size);

// To call when the DMA overflow/has reached the end and return to index 0
void ringbuffer_dma_add_overflow(ringbuffer_t* buffer);

uint32_t ringbuffer_read(ringbuffer_t* buffer, uint8_t *data, uint32_t max_size);

void ringbuffer_write(ringbuffer_t* buffer, uint8_t *data, uint32_t size);
