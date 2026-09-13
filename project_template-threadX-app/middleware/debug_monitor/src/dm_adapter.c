#include "dm_adapter.h"

#include "dm_motor.h"

/** ---------- Modbus 测试寄存器映射（最小骨架） ---------- */

/** 只读测试段（0xE000 起）：魔数 / 版本 / 自增计数 */
enum test_reg_t
{
    TEST_REG_MAGIC = 0,   /**< 固定魔数 0x1234（验证读链路） */
    TEST_REG_VERSION,     /**< 版本号 0x0100 */
    TEST_REG_COUNTER_L,   /**< 自增计数器低 16 位 */
    TEST_REG_COUNTER_H,   /**< 自增计数器高 16 位 */
    TEST_REG_NUM,
};

static uint16_t s_test_regs[TEST_REG_NUM];

/** 可写测试段（0x0000 起）：写回显 */
enum ctrl_reg_t
{
    CTRL_REG_TEST_WORD = 0,
    CTRL_REG_TEST_CMD,
    CTRL_REG_NUM,
};

static uint16_t s_ctrl_regs[CTRL_REG_NUM] = { 0x0000u, 0x0000u };

/** 写回调：测试字直接回显 */
static enum mb_err_t ctrl_on_write(uint16_t addr, uint16_t val)
{
    switch (addr)
    {
    case CTRL_REG_TEST_WORD:
        s_ctrl_regs[CTRL_REG_TEST_WORD] = val;
        return MB_OK;

    case CTRL_REG_TEST_CMD:
        s_ctrl_regs[CTRL_REG_TEST_CMD] = val;
        return MB_OK;

    default:
        return MB_ERR_ADDR;
    }
}

static struct mb_reg_map s_reg_map =
{
    .coils        = {{0, 0, NULL}},
    .coils_num    = 0,
    .discrete     = {{0, 0, NULL}},
    .discrete_num = 0,
    .holding      = {
        {
            .start_addr = 0x0000,   /**< 测试写段 */
            .num        = CTRL_REG_NUM,
            .data       = s_ctrl_regs,
            .on_write   = ctrl_on_write,
        },
        {
            .start_addr = 0xE000,   /**< 测试读段 */
            .num        = TEST_REG_NUM,
            .data       = s_test_regs,
        },
        {
            /* 电机观测段（0x2000 起，只读 float×2 寄存器），由 dm_motor_reg_seg 填充 */
        },
    },
    .holding_num  = 3,
    .input        = {{0, 0, NULL}},
    .input_num    = 0,
};

static uint32_t s_counter = 0u;

void dm_adapter_refresh(void)
{
    s_counter++;

    s_test_regs[TEST_REG_MAGIC]     = 0x1234u;
    s_test_regs[TEST_REG_VERSION]   = 0x0100u;
    s_test_regs[TEST_REG_COUNTER_L] = (uint16_t)(s_counter & 0xFFFFu);
    s_test_regs[TEST_REG_COUNTER_H] = (uint16_t)((s_counter >> 16u) & 0xFFFFu);
}

static const dm_adapter s_adapter =
{
    .init = NULL,
    .poll = NULL,
};

const struct mb_reg_map *dm_adapter_reg_map(void)
{
    /* 运行时填充电机观测段（第 3 段，0x2000 起只读） */
    dm_motor_reg_seg(&s_reg_map.holding[2]);
    return &s_reg_map;
}

const dm_adapter *dm_adapter_get(void)
{
    return &s_adapter;
}
