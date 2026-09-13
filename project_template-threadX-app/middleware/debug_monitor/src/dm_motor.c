#include "dm_motor.h"

#include <string.h>

#include "drv_motor.h"

/**
 * dm_motor：mcl 全量可观测变量 → Modbus 保持寄存器（只读观测段 0x2000 起）。
 *
 * 直接读 mcl 结构体公开字段（mcl.h 中 motor 对象字段为 public），
 * 不改动 mcl 库。on_read 回调按绝对地址分派到具体变量，动态取最新值。
 *
 * 布局（从 DM_MOTOR_REG_BASE=0x2000 起）：
 *   [0x00] state            uint16 枚举
 *   [0x01] fault            uint16 枚举
 *   [0x02] ctrl_mode        uint16 枚举
 *   [0x03] mode             uint16 枚举
 *   [0x04] ol_stage         uint16 枚举
 *   [0x05] tick_hi          uint32 tick_count 高字
 *   [0x06] tick_lo          uint32 tick_count 低字
 *   [0x07] (保留对齐)
 *   [0x08] speed_rpm        float32
 *   [0x0A] position_rad     float32
 *   [0x0C] iq               float32
 *   [0x0E] id               float32
 *   [0x10] vbus             float32
 *   [0x12] ibus             float32
 *   [0x14] duty             float32
 *   [0x16] temp_motor       float32
 *   [0x18] temp_fet         float32
 *   [0x1A] est_phase        float32
 *   [0x1C] est_speed        float32
 *   [0x1E] iq_ref           float32
 *   [0x20] speed_ref_rpm    float32
 *   [0x22] openloop_mag     float32
 *   [0x24] openloop_speed   float32
 *   [0x26] openloop_angle   float32
 *   [0x28] v_alpha_prev     float32
 *   [0x2A] v_beta_prev      float32
 *   [0x2C] fault_current    float32
 *   [0x2E] fault_voltage    float32
 *   [0x30] fault_speed      float32
 *   [0x32] fault_temp       float32
 *   [0x34] fault_tick_hi    uint32 高字
 *   [0x35] fault_tick_lo    uint32 低字
 */

/** 绑定电机数据源（drivers 层注入） */
static const struct drv_motor *s_motor = NULL;

void dm_motor_bind(const struct drv_motor *motor)
{
    s_motor = motor;
}

/** 把一个 float 拆成高/低字写到 out（half=0 高字，1 低字） */
static void put_float16(float value, uint8_t half, uint16_t *out)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    if (half == 0u)
    {
        *out = (uint16_t)((bits >> 16u) & 0xFFFFu);
    }
    else
    {
        *out = (uint16_t)(bits & 0xFFFFu);
    }
}

static enum mb_err_t dm_motor_read_half(uint16_t addr, uint16_t *out)
{
    uint16_t local = (uint16_t)(addr - DM_MOTOR_REG_BASE);

    if (s_motor == NULL)
    {
        return MB_ERR_ADDR;
    }

    /* 需要完整 mcl 对象（直接读公开字段） */
    const struct drv_motor *dm = s_motor;
    const mcl *m = &dm->motor;

    /* ---- 整数段（uint16 枚举，1 寄存器） ---- */
    switch (local)
    {
    case 0x00: *out = (uint16_t)m->state;     return MB_OK;
    case 0x01: *out = (uint16_t)m->fault;     return MB_OK;
    case 0x02: *out = (uint16_t)m->ctrl_mode; return MB_OK;
    case 0x03: *out = (uint16_t)m->mode;      return MB_OK;
    case 0x04: *out = (uint16_t)m->ol_stage;  return MB_OK;
    /* tick_count uint32：高字在前 */
    case 0x05: *out = (uint16_t)((m->tick_count >> 16u) & 0xFFFFu); return MB_OK;
    case 0x06: *out = (uint16_t)(m->tick_count & 0xFFFFu);          return MB_OK;
    /* 0x07：对齐填充（float 段从 0x08 偶数起），恒 0 */
    case 0x07: *out = 0u; return MB_OK;
    /* fault_info.tick uint32 */
    case 0x34: *out = (uint16_t)((m->fault_info.tick >> 16u) & 0xFFFFu); return MB_OK;
    case 0x35: *out = (uint16_t)(m->fault_info.tick & 0xFFFFu);          return MB_OK;
    default:
        break;
    }

    /* ---- float32 段（每 2 寄存器，高字在前） ---- */
    {
        float value;
        switch (local)
        {
        case 0x08: value = (float)m->speed_rad_s * 9.5492966f / (float)m->cfg.pole_pairs; break; /* speed_rpm */
        case 0x09: value = (float)m->speed_rad_s * 9.5492966f / (float)m->cfg.pole_pairs; break;
        case 0x0A: value = (float)m->phase_rad;           break;
        case 0x0B: value = (float)m->phase_rad;           break;
        case 0x0C: value = (float)m->iq_now;              break;
        case 0x0D: value = (float)m->iq_now;              break;
        case 0x0E: value = (float)m->id_now;              break;
        case 0x0F: value = (float)m->id_now;              break;
        case 0x10: value = (float)m->vbus;                break;
        case 0x11: value = (float)m->vbus;                break;
        case 0x12: value = 0.0f;                          break; /* ibus 无传感器，恒 0 */
        case 0x13: value = 0.0f;                          break;
        case 0x14: value = (float)m->duty_now;            break;
        case 0x15: value = (float)m->duty_now;            break;
        case 0x16: value = 0.0f;                          break; /* temp_motor 未接 */
        case 0x17: value = 0.0f;                          break;
        case 0x18: value = 0.0f;                          break; /* temp_fet 未接 */
        case 0x19: value = 0.0f;                          break;
        case 0x1A: value = (float)m->phase_rad;           break; /* est_phase 观测器相位 */
        case 0x1B: value = (float)m->phase_rad;           break;
        case 0x1C: value = (float)m->speed_rad_s;         break; /* est_speed */
        case 0x1D: value = (float)m->speed_rad_s;         break;
        case 0x1E: value = (float)m->iq_ref;              break;
        case 0x1F: value = (float)m->iq_ref;              break;
        case 0x20: value = (float)m->speed_ref_rpm;       break;
        case 0x21: value = (float)m->speed_ref_rpm;       break;
        case 0x22: value = (float)m->openloop_mag;        break;
        case 0x23: value = (float)m->openloop_mag;        break;
        case 0x24: value = (float)m->openloop_speed;      break;
        case 0x25: value = (float)m->openloop_speed;      break;
        case 0x26: value = (float)m->openloop_angle;      break;
        case 0x27: value = (float)m->openloop_angle;      break;
        case 0x28: value = (float)m->v_alpha_prev;        break;
        case 0x29: value = (float)m->v_alpha_prev;        break;
        case 0x2A: value = (float)m->v_beta_prev;         break;
        case 0x2B: value = (float)m->v_beta_prev;         break;
        case 0x2C: value = (float)m->fault_info.current;  break;
        case 0x2D: value = (float)m->fault_info.current;  break;
        case 0x2E: value = (float)m->fault_info.voltage;  break;
        case 0x2F: value = (float)m->fault_info.voltage;  break;
        case 0x30: value = (float)m->fault_info.speed;    break;
        case 0x31: value = (float)m->fault_info.speed;    break;
        case 0x32: value = (float)m->fault_info.temp;     break;
        case 0x33: value = (float)m->fault_info.temp;     break;
        /* ---- ORTEGA 观测器内部状态（诊断观测器是否跟踪转子） ---- */
        case 0x36: value = (float)dm->observer.x1;          break; /* 定子磁链 α */
        case 0x37: value = (float)dm->observer.x1;          break;
        case 0x38: value = (float)dm->observer.x2;          break; /* 定子磁链 β */
        case 0x39: value = (float)dm->observer.x2;          break;
        case 0x3A: value = (float)dm->observer.lambda_est;   break; /* 转子磁链幅值 */
        case 0x3B: value = (float)dm->observer.lambda_est;   break;
        case 0x3C: value = (float)dm->observer.i_alpha_last; break; /* 上一拍 α 电流 */
        case 0x3D: value = (float)dm->observer.i_alpha_last; break;
        default:
            return MB_ERR_ADDR;
        }
        put_float16(value, (uint8_t)(local & 0x01u), out);
        return MB_OK;
    }
}

void dm_motor_reg_seg(struct mb_reg_seg *out_seg)
{
    if (out_seg == NULL)
    {
        return;
    }

    out_seg->start_addr = DM_MOTOR_REG_BASE;
    out_seg->num        = DM_MOTOR_REG_NUM;
    out_seg->data       = NULL;             /* 用 on_read 动态取值 */
    out_seg->on_read    = dm_motor_read_half;
    out_seg->on_write   = NULL;             /* 只读 */
}
