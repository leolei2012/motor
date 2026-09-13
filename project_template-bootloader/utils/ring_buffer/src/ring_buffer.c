#include "ring_buffer.h"

#include <string.h>


void ring_buffer_init(struct ring_buffer *self, uint8_t *buffer, uint8_t capacity)
{
    if (self == NULL || buffer == NULL || capacity == 0)
    {
        return;
    }
    memset(self, 0, sizeof(struct ring_buffer));

    self->buffer = buffer;
    self->capacity = capacity;
    self->head = 0;
    self->tail = 0;
    self->count = 0;
}

void ring_buffer_clear(struct ring_buffer *self)
{
    if (self == NULL)
    {
        return;
    }

    self->head = 0;
    self->tail = 0;
    self->count = 0;
}

uint16_t ring_buffer_readable(struct ring_buffer *self)
{
    if (self == NULL)
    {
        return 0;
    }

    return self->count;
}

uint16_t ring_buffer_writable(struct ring_buffer *self)
{
    if (self == NULL)
    {
        return 0;
    }

    return self->capacity - self->count;
}

uint8_t ring_buffer_is_empty(struct ring_buffer *self)
{
    if (self == NULL)
    {
        return 1; /// 视为为空
    }

    return (self->count == 0);
}

uint8_t ring_buffer_is_full(struct ring_buffer *self)
{
    if (self == NULL)
    {
        return 0; /// 视为不为满
    }

    return (self->count == self->capacity);
}

uint8_t ring_buffer_put(struct ring_buffer *self, uint8_t byte)
{
    if (self == NULL || ring_buffer_is_full(self))
    {
        return 0;
    }

    self->buffer[self->tail] = byte;
    self->tail = (self->tail + 1) % self->capacity;

    self->count++;
    if (self->count > self->capacity)
    {
        self->count = self->capacity; /// 防止溢出
    }

    return 1;
}

uint8_t ring_buffer_get(struct ring_buffer *self, uint8_t *byte)
{
    if (self == NULL || ring_buffer_is_empty(self))
    {
        return 0;
    }

    if (byte != NULL)
    {
        *byte = self->buffer[self->head];
    }

    self->head = (self->head + 1) % self->capacity;
    self->count--;

    return 1;
}