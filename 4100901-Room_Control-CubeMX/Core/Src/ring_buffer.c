#include "ring_buffer.h"
/**
* @brief  Initializes the ring buffer variable to their initial values.
* @param  rb Pointer to the ring buffer structure.
* @param  buffer Pointer to the buffer that will be used for the ring buffer.
* @param  capacity The maximum number of elements the ring buffer can hold.
*/ 
void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity) 
{
rb->buffer = buffer;
    rb->head = 0;
    rb->tail = 0;
    rb->capacity = capacity;
    rb-> is_full = false;
}
/**
 * @brief  Writes a byte of data to the ring buffer, discard the old data if is_full.
 * @param  rb Pointer to the ring buffer structure.
 * @param  data The byte of data to write.
 * @return true if the write was successful, false if the buffer is full.
 */
bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
if (rb->is_full) {
        // If the buffer is full, overwrite the oldest data
        rb->tail = (rb->tail + 1) % rb->capacity;
    }
    rb->buffer[rb->head] = data; //write the new data
    rb->head = (rb->head + 1) % rb->capacity; // Move head forward
    if (rb->head == rb->tail) {
        rb->is_full = true; // If head meets tail, buffer is full
    }

    return true;
}

/**
 * @brief  Reads a byte of data from the ring buffer.
 * @param  rb Pointer to the ring buffer structure.
 * @param  data Pointer to the variable where the read data will be stored.
 * @return true if the read was successful, false if the buffer is empty.
 */

bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
if (rb->head == rb->tail && !rb->is_full) {
        // Buffer is empty
        return false;
    }
    *data = rb->buffer[rb->tail]; // Read the data
    rb->tail = (rb->tail + 1) % rb->capacity; // Move tail forward
    rb->is_full = false; // After reading, the buffer can't be full
    return true;
}

/**
 * @brief  Returns the number of elements currently in the ring buffer.
 * @param  rb Pointer to the ring buffer structure.
 * @return The number of elements in the ring buffer.
 */
uint16_t ring_buffer_count(ring_buffer_t *rb)
{
    if(rb->is_full) {
        return rb->capacity; // If the buffer is full, return its capacity
    }
    if(rb->head >= rb->tail) {
        return rb->head - rb->tail; // If head is ahead of tail, return the difference
    } else {
        return (rb->capacity - rb->tail) + rb->head; // wrap around case
    }
}

/**
 * @brief  Checks if the ring buffer is empty.
 * @param  rb Pointer to the ring buffer structure.
 * @return true if the buffer is empty, false otherwise.
 */
bool ring_buffer_is_empty(ring_buffer_t *rb)
{
    return (rb->head == rb->tail && !rb->is_full); // Buffer is empty if head equals tail and is not full

}

/**
 * @brief  Checks if the ring buffer is full.
 * @param  rb Pointer to the ring buffer structure.
 * @return true if the buffer is full, false otherwise.
 */
bool ring_buffer_is_full(ring_buffer_t *rb)
{
    return rb->is_full; // Buffer is full if the is_full flag is set

}

/**
 * @brief  Flushes the ring buffer, clearing all data.
 * @param  rb Pointer to the ring buffer structure.
 * @note   This function resets the head and tail pointers and clears the is_full flag.
 */

void ring_buffer_flush(ring_buffer_t *rb)
{
    rb->head = 0; // Reset head pointer
    rb->tail = 0; // Reset tail pointer
    rb->is_full = false; // Clear the full flag
   
}