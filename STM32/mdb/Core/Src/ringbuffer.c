#include <ringbuffer.h>
#include <stdio.h>

void ringbuffer_init(ringbuffer_t* buffer, uint8_t* data, uint32_t max_size) {
    buffer->data = data;
    buffer->max_size = max_size;
    buffer->nb_overflows = 0;
    buffer->start = 0;
    buffer->size = 0;
}

void ringbuffer_dma_reset(ringbuffer_t* buffer, uint32_t size) {
    buffer->nb_overflows = 0;
    buffer->start = size;
}

// The number of bytes written inside the DMA buffer from the index 0
void ringbuffer_dma_set_written_size(ringbuffer_t* buffer, uint32_t size) {
    if (buffer->nb_overflows == 1) {
        if (buffer->start < size) {
        	ringbuffer_dma_reset(buffer, size);
        	printf("A\n\r");
        } else {
            buffer->size = buffer->max_size - buffer->start + size;
        }
    } else if (buffer->nb_overflows > 1){
    	ringbuffer_dma_reset(buffer, size);
    	printf("B\n\r");
    } else if (buffer->nb_overflows < 0) {
        // Here ringbuffer_dma_add_overflow has not yet been called but buffer->start has go back to 0
        if (size < buffer->start) {
            // ERROR! (ringbuffer_dma_add_overflow has never been called)
        	printf("WTF DMA ERROOOOR\n\r");
        } else {
            buffer->size = size - buffer->start;
        }
    } else {
        if (size < buffer->start) {
            // Here an overfow has occured but ringbuffer_dma_add_overflow has not yet been called
            buffer->size = buffer->max_size - buffer->start + size;
        } else {
            buffer->size = size - buffer->start;
        }
    }
}

// To call when the DMA overflow/has reached the end and return to index 0
void ringbuffer_dma_add_overflow(ringbuffer_t* buffer) {
    buffer->nb_overflows++;
}

uint32_t ringbuffer_get_lost_bytes(ringbuffer_t* buffer) {
    if (buffer->size < buffer->max_size) {
        return 0;
    }
    return buffer->size - buffer->max_size;
}

uint32_t ringbuffer_get_size(ringbuffer_t* buffer) {
    return buffer->size;
}

uint32_t ringbuffer_read(ringbuffer_t* buffer, uint8_t *data, uint32_t max_size) {
	int32_t lost_bytes = ringbuffer_get_lost_bytes(buffer);
    if (lost_bytes) {
    	printf("DMA ERROR: lost %ld bytes\n\r", lost_bytes);
    	printf("max_size=%ld, nb_overflow=%ld\n\r", max_size, buffer->nb_overflows);
    }

    uint32_t to_read = buffer->size;
    // Check if nb bytes > bytes user wants to read
    if (to_read > max_size) {
        to_read = max_size;
    }

    buffer->size -= to_read;

    // nb bytes > ringbuffer
    if (to_read > buffer->max_size) {
        to_read = buffer->max_size;
    }

    const uint32_t buffer_max_size = buffer->max_size;
    uint8_t *buffer_data = buffer->data;
    uint32_t start = buffer->start;

    for (uint32_t i = 0; i < to_read; i++) {
        data[i] = buffer_data[start];
        start++;
        if (start >= buffer_max_size) {
            start = 0;
            buffer->nb_overflows--;
        }
    }

    buffer->start = start;

    return to_read;
}

void ringbuffer_write(ringbuffer_t* buffer, uint8_t *data, uint32_t size) {
    const uint32_t buffer_max_size = buffer->max_size;
    uint8_t *buffer_data = buffer->data;
    uint32_t start = buffer->start;

    for (uint32_t i = 0; i < size; i++) {
        buffer_data[start] = data[i];
        if (start >= buffer_max_size) {
            start = 0;
        }
    }

    buffer->start = start;
}
