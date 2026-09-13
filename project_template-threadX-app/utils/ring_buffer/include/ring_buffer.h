#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "platform.h"
#include <stddef.h>



struct ring_buffer
{
    uint8_t *buffer;  /// 只支持 uint8_t 类型的缓冲区
    uint8_t capacity; /// 缓冲区容量
    uint8_t head;     /// 写指针（下一个写入位置）
    uint8_t tail;     /// 读指针（下一个读取位置）
    uint8_t count;    /// 当前数据量
};

void ring_buffer_init(struct ring_buffer *self, uint8_t *buffer, uint8_t capacity);
void ring_buffer_clear(struct ring_buffer *self);
uint16_t ring_buffer_readable(struct ring_buffer *self);
uint16_t ring_buffer_writable(struct ring_buffer *self);
uint8_t ring_buffer_is_empty(struct ring_buffer *self);
uint8_t ring_buffer_is_full(struct ring_buffer *self);
uint8_t ring_buffer_put(struct ring_buffer *self, uint8_t byte);
uint8_t ring_buffer_get(struct ring_buffer *self, uint8_t *byte);

#endif // RING_BUFFER_H
