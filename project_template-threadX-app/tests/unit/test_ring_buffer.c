/**
 * @file  test_ring_buffer.c
 * @brief ring_buffer 单元测试（使用 Unity 测试框架）
 *
 * 编译（PC 上运行）：
 *   gcc test_ring_buffer.c ../../utils/src/ring_buffer.c \
 *       unity/unity.c -I../../utils/include -Iunity -o test_rb && ./test_rb
 */
#include "unity.h"
#include "ring_buffer.h"

static uint8_t   buf[16];
static ring_buf_t rb;

void setUp(void)    { ring_buf_init(&rb, buf, sizeof(buf)); }
void tearDown(void) {}

void test_initial_state_is_empty(void)
{
    TEST_ASSERT_TRUE(ring_buf_empty(&rb));
    TEST_ASSERT_EQUAL(0, ring_buf_len(&rb));
}

void test_put_and_get_single_byte(void)
{
    TEST_ASSERT_TRUE(ring_buf_put(&rb, 0xAB));
    uint8_t out = 0;
    TEST_ASSERT_TRUE(ring_buf_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0xAB, out);
    TEST_ASSERT_TRUE(ring_buf_empty(&rb));
}

void test_full_condition(void)
{
    for (int i = 0; i < 16; i++) ring_buf_put(&rb, (uint8_t)i);
    TEST_ASSERT_TRUE(ring_buf_full(&rb));
    TEST_ASSERT_FALSE(ring_buf_put(&rb, 0xFF));  /* 满时拒绝写入 */
}

void test_fifo_order(void)
{
    ring_buf_put(&rb, 1);
    ring_buf_put(&rb, 2);
    ring_buf_put(&rb, 3);
    uint8_t out;
    ring_buf_get(&rb, &out); TEST_ASSERT_EQUAL(1, out);
    ring_buf_get(&rb, &out); TEST_ASSERT_EQUAL(2, out);
    ring_buf_get(&rb, &out); TEST_ASSERT_EQUAL(3, out);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_empty);
    RUN_TEST(test_put_and_get_single_byte);
    RUN_TEST(test_full_condition);
    RUN_TEST(test_fifo_order);
    return UNITY_END();
}
