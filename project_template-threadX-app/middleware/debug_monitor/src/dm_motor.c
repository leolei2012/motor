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
 *   [0x36] obs_x1 / obs_x2 / obs_lambda / obs_i_alpha   float32×4（0x2036~0x203D）
 *   [0x40] r_meas           float32 启动实测相电阻 Ω
 *   [0x42] l_meas           float32 启动实测相电感 H
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

/** 切换后逐拍采集段读取（定义在后） */
static enum mb_err_t dm_motor_read_cap2(uint16_t local, uint16_t *out);

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

    /* 0x2050 起为切换后逐拍采集段 */
    if (local >= 0x50u)
    {
        return dm_motor_read_cap2(local, out);
    }

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
    /* 0x07：启动进度/切换捕获标志（1=使能驱动 2=TIM1 3=零漂校准 4=MOE 5=R/L
       实测 6=已 start；128+1=切换已捕获），替代原对齐填充 0 */
    case 0x07: *out = (uint16_t)(dm->start_step | (dm->cap_valid ? 0x80u : 0u)); return MB_OK;
    /* fault_info.tick uint32 */
    case 0x34: *out = (uint16_t)((*(uint32_t *)(void *)&dm->cap_pll_last) >> 16u); return MB_OK;
    case 0x35: *out = (uint16_t)((*(uint32_t *)(void *)&dm->cap_pll_last) & 0xFFFFu);          return MB_OK;
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
        case 0x12: value = dm->cap_post_lam;                 break; /* 复用：切换后观测器角 rad（原 ibus 恒 0） */
        case 0x13: value = dm->cap_post_lam;                 break;
        case 0x14: value = (float)m->duty_now;            break;
        case 0x15: value = (float)m->duty_now;            break;
        case 0x16: value = dm->cap_in_prev1;                 break; /* 复用：拖动倒数第二拍 PLL 输入角 rad */
        case 0x17: value = dm->cap_in_prev1;                 break;
        case 0x18: value = dm->cap_in_prev2;                 break; /* 复用：拖动倒数第三拍 PLL 输入角 rad */
        case 0x19: value = dm->cap_in_prev2;                 break;
        case 0x1A: value = (float)m->phase_rad;           break; /* est_phase 观测器相位 */
        case 0x1B: value = (float)m->phase_rad;           break;
        case 0x1C: value = (float)m->speed_rad_s;         break; /* est_speed */
        case 0x1D: value = (float)m->speed_rad_s;         break;
        case 0x1E: value = (float)m->iq_ref;              break;
        case 0x1F: value = (float)m->iq_ref;              break;
        case 0x20: value = (float)m->speed_ref_rpm;       break;
        case 0x21: value = (float)m->speed_ref_rpm;       break;
        case 0x22: value = dm->cap_pre_est;                  break; /* 复用：切换前帧角 rad（原 openloop_mag） */
        case 0x23: value = dm->cap_pre_est;                  break;
        case 0x24: value = dm->cap_pre_lam;                  break; /* 复用：切换前观测器角 rad */
        case 0x25: value = dm->cap_pre_lam;                  break;
        case 0x26: value = dm->cap_post_est;                 break; /* 复用：切换后帧角 rad（原 openloop_angle） */
        case 0x27: value = dm->cap_post_est;                 break;
        case 0x28: value = (float)m->v_alpha_prev;        break;
        case 0x29: value = (float)m->v_alpha_prev;        break;
        case 0x2A: value = (float)m->v_beta_prev;         break;
        case 0x2B: value = (float)m->v_beta_prev;         break;
        case 0x2C: value = dm->ia_now;                     break; /* 复用：A 相电流 A（原 fault_current） */
        case 0x2D: value = dm->ia_now;                     break;
        case 0x2E: value = dm->ib_now;                     break; /* 复用：B 相电流 A（原 fault_voltage） */
        case 0x2F: value = dm->ib_now;                     break;
        case 0x30: value = dm->ic_now;                     break; /* 复用：C 相电流 A（原 fault_speed） */
        case 0x31: value = dm->ic_now;                     break;
        case 0x32: value = dm->cap_post_spd;                 break; /* 复用：切换后 PLL 速度 rad/s（原 fault_temp） */
        case 0x33: value = dm->cap_post_spd;                 break;
        /* ---- Ortega 观测器内部状态（诊断：定子磁链/磁链幅值） ---- */
        case 0x36: value = (float)dm->observer.x1;          break; /* 定子磁链 α */
        case 0x37: value = (float)dm->observer.x1;          break;
        case 0x38: value = (float)dm->observer.x2;          break; /* 定子磁链 β */
        case 0x39: value = (float)dm->observer.x2;          break;
        case 0x3A: value = (float)dm->observer.lambda_est;   break; /* 转子磁链幅值 */
        case 0x3B: value = (float)dm->observer.lambda_est;   break;
        case 0x3C: value = (float)dm->observer.i_alpha_last; break; /* 上一拍 α 电流 */
        case 0x3D: value = (float)dm->observer.i_alpha_last; break;
        case 0x3E: value = (float)dm->observer.i_beta_last;  break; /* 上一拍 β 电流 */
        case 0x3F: value = (float)dm->observer.i_beta_last;  break;
        case 0x40: value = dm->r_meas;                       break; /* 启动实测相电阻 Ω */
        case 0x41: value = dm->r_meas;                       break;
        case 0x42: value = dm->l_meas;                       break; /* 启动实测相电感 H */
        case 0x43: value = dm->l_meas;                       break;
        case 0x44: value = dm->ia_now;                       break; /* A 相电流 A（减零漂） */
        case 0x45: value = dm->ia_now;                       break;
        case 0x46: value = dm->ib_now;                       break; /* B 相电流 A */
        case 0x47: value = dm->ib_now;                       break;
        case 0x48: value = dm->ic_now;                       break; /* C 相电流 A */
        case 0x49: value = dm->ic_now;                       break;
        default:
            return MB_ERR_ADDR;
        }
        put_float16(value, (uint8_t)(local & 0x01u), out);
        return MB_OK;
    }
}

/** 切换后逐拍采集段（0x2050 起）：对数间隔 14 点 × 8 字段（frame/obs/spd/va/vb/x1/x2/lam），
    读锁存影子（cap2_s_*，重开环瞬间 ISR 内整体拷贝，保证整轮一致）。
    布局：0x50 起摘要（u32 计数 ×2、u16 valid_n、f32 min_spd/max_iq），
    0x60 + 16k：第 k 点 frame/obs/spd/va/vb/x1/x2/lam 各占 2 寄存器。 */
static enum mb_err_t dm_motor_read_cap2(uint16_t local, uint16_t *out)
{
    const struct drv_motor *dm = s_motor;
    float value;

    if (dm == NULL)
    {
        return MB_ERR_ADDR;
    }

    switch (local)
    {
    case 0x50: *out = (uint16_t)(dm->cap2_s_switch_count >> 16u); return MB_OK;
    case 0x51: *out = (uint16_t)(dm->cap2_s_switch_count & 0xFFFFu); return MB_OK;
    case 0x52: *out = (uint16_t)(dm->cap2_s_ticks >> 16u); return MB_OK;
    case 0x53: *out = (uint16_t)(dm->cap2_s_ticks & 0xFFFFu); return MB_OK;
    case 0x54: *out = (uint16_t)dm->cap2_s_valid_n; return MB_OK;
    case 0x55: *out = 0u; return MB_OK;                        /* 对齐填充 */
    case 0x56: value = dm->cap2_s_min_spd; break;
    case 0x57: value = dm->cap2_s_min_spd; break;
    case 0x58: value = dm->cap2_s_max_iq; break;
    case 0x59: value = dm->cap2_s_max_iq; break;
    case 0x5A: value = 0.0f; break;                            /* 对齐填充 */
    case 0x5B: value = 0.0f; break;
    case 0x5C: value = 0.0f; break;
    case 0x5D: value = 0.0f; break;
    case 0x5E: value = 0.0f; break;
    case 0x5F: value = 0.0f; break;
    default:
        if (local >= 0x60u && local < 0x60u + 14u * 16u)
        {
            uint16_t off = (uint16_t)(local - 0x60u);
            uint8_t k = (uint8_t)(off / 16u);
            switch (off % 16u)
            {
            case 0:  value = dm->cap2_s_frame[k]; break;
            case 1:  value = dm->cap2_s_frame[k]; break;
            case 2:  value = dm->cap2_s_obs[k];   break;
            case 3:  value = dm->cap2_s_obs[k];   break;
            case 4:  value = dm->cap2_s_spd[k];   break;
            case 5:  value = dm->cap2_s_spd[k];   break;
            case 6:  value = dm->cap2_s_va[k];    break;
            case 7:  value = dm->cap2_s_va[k];    break;
            case 8:  value = dm->cap2_s_vb[k];    break;
            case 9:  value = dm->cap2_s_vb[k];    break;
            case 10: value = dm->cap2_s_x1[k];    break;
            case 11: value = dm->cap2_s_x1[k];    break;
            case 12: value = dm->cap2_s_x2[k];    break;
            case 13: value = dm->cap2_s_x2[k];    break;
            default: value = dm->cap2_s_lam[k];   break;
            }
            put_float16(value, (uint8_t)(local & 0x01u), out);
            return MB_OK;
        }
        return MB_ERR_ADDR;
    }
    put_float16(value, (uint8_t)(local & 0x01u), out);
    return MB_OK;
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
