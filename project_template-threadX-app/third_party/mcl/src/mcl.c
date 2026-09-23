/**
 * @file    mcl.c
 * @brief   mcl 电机控制库：门面实现（生命周期、指令、查询、控制节拍、校准封装）
 *
 * 控制节拍 mcl_control_tick 在电流环中断内由宿主调用，
 * 串起采样 → Clarke → 相位/速度 → 外环 → 电流环 → 反 Park → SVPWM → 保护 → 输出。
 */

#include "mcl.h"
#include "mcl_observer_ortega.h"
#include "mcl_observer_smo.h"

/* 60/(2π)：rad/s → rpm 换算系数（TODO 定点：需缩放） */
#define MCL_RPM_PER_RAD_S  9.5492966f
/* 2π/60：rpm → rad/s 换算系数 */
#define MCL_RAD_PER_S_PER_RPM  0.1047197551f
#define MCL_SWITCH_BLEND_TIME 0.5f
#define MCL_SMO_LOCK_TIME     0.02f
#define MCL_SMO_WAIT_TIME     0.5f

/* ============================ 角度回绕 ============================ */

/* 角度归一到 [0, 整圈)：float 用 2π；定点用 1.0（归一化角）。
   仅当 observer（自动开环）或 openloop（手动开环）任一启用时才需要 */
#if !defined(MCL_DISABLE_OBSERVER) || !defined(MCL_DISABLE_OPENLOOP)

/* 每控制周期的「相位增量」换算：speed(电气速度) × dt → 相位增量。
 * - float：phase/speed 都是物理弧度/rad/s，dt 物理秒，speed×dt 已是弧度，系数 1。
 * - 定点：phase 是「归一化圈」(1.0=2π)，speed 是电气速度 pu(ω/W_BASE)，dt 是时间 pu
 *   (dt·W_BASE)。speed_pu×dt_pu = ω·dt = 弧度，需再 ×1/(2π) 才是「圈」。
 *   此前的定点实现漏乘 1/(2π)，导致开环拖动的相位斜率偏 ~2π 倍
 *   （VF 目标 100rpm 实测 ~630rpm，自动开环无法切换到闭环）。
 */
static mcl_scalar mcl_speed_to_phase_incr(mcl_scalar speed, mcl_scalar dt)
{
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    return MCL_MUL(MCL_MUL(speed, dt), MCL_FROM_FLOAT(1.0f / 6.28318530718f));
#else
    return MCL_MUL(speed, dt);
#endif
}

static mcl_scalar mcl_wrap_full_turn(mcl_scalar x)
{
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    const mcl_scalar full = MCL_FROM_FLOAT(1.0f);
#else
    const mcl_scalar full = (mcl_scalar)MCL_TWO_PI;
#endif
    while (x >= full)
    {
        x = MCL_SUB(x, full);
    }
    while (x < (mcl_scalar)0)
    {
        x = MCL_ADD(x, full);
    }
    return x;
}
#endif

/* ============================ 控制节拍内部实现 ============================ */

/**
 * Sensorless startup: align -> ramp -> drag -> observer qualification.
 * During drag the observer runs freely with a signed bandwidth hint.
 * Handover releases the hint, seeds PLL speed history consistently, and
 * removes the control-frame offset gradually instead of jumping its angle.
 * ol_stage: 1=alignment, 2=drag, 0=closed loop (including handover blend).
 */
#ifndef MCL_DISABLE_OBSERVER

/* 若观测器是 SMO，写入 seed 用的开环角速度（Ortega 无此字段，仅按 ops 判断）。 */
static void mcl_observer_seed_omega_set(const mcl_observer *obs, mcl_scalar omega)
{
    if (obs != NULL && obs->ops == &mcl_observer_smo_ops)
    {
        mcl_observer_smo_set_seed_omega(obs->impl, omega);
    }
}

static void mcl_openloop_auto(mcl *self, mcl_scalar *phase, mcl_scalar *speed)
{
    const mcl_scalar dt = self->dt;
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    /* 定点：rpm_pu == 电气速度_pu，rpm↔rad/s 换算省略（见 mcl_fixed_point.md）。
       rpm_pu 到电气速度_pu 系数 = RAD_PER_S_PER_RPM × pp = 1（在各自基值下） */
    const mcl_scalar rpm_to_espeed = MCL_FROM_FLOAT(1.0f);
#else
    const mcl_scalar rpm_to_espeed = MCL_FROM_FLOAT(MCL_RAD_PER_S_PER_RPM * (float)self->cfg.pole_pairs);
#endif
    const mcl_scalar t_lock = self->cfg.openloop_time_lock;
    const mcl_scalar t_ramp = self->cfg.openloop_time_ramp;
    const mcl_scalar t_const = self->cfg.openloop_time;
    const mcl_scalar total = MCL_ADD(MCL_ADD(t_lock, t_ramp), t_const);
    mcl_scalar ol_rpm_max;
    mcl_scalar ol_speed_max;
    mcl_scalar ol_rpm;
    mcl_scalar time_fwd;
    mcl_scalar dir;
    mcl_scalar sign;
    mcl_scalar s_ramp;
    mcl_scalar c_ramp;

    /* Zero explicitly disables automatic startup. */
    if (self->cfg.openloop_rpm <= (mcl_scalar)0 || total <= (mcl_scalar)0)
    {
        self->ol_stage = 0u;
        return;
    }

    /* 开环转速上限（固定值）。
       TODO：VESC 按 |iq| 自适应（电流越大转速越高，I/F），依赖 openloop_rpm_low /
       openloop_boost_q / openloop_max_q；mcl 当前简化为固定 openloop_rpm，后续恢复。 */
    ol_rpm_max = self->cfg.openloop_rpm;
    ol_speed_max = MCL_MUL(ol_rpm_max, rpm_to_espeed);

    /* 重开环迟滞阈值 = 开环拖动转速的一半。历史教训：用全量 ol_speed_max 作阈值时，
       闭环目标转速（=拖动转速 300rpm）下电机稳态 ~280rpm（146rad/s）正好卡在阈值
       157rad/s 之下 → 永远在「低速」迟滞累计 → 每 0.1s 重开环一次（实测 survival
       445ms 上限）。降到一半（150rpm→78.6rad/s）后，稳态 280rpm 远高于阈值，
       只在真实掉速/失步时才重开环。 */
    ol_speed_max = MCL_MUL(ol_speed_max, MCL_FROM_FLOAT(0.5f));

    /* 3. 拖动方向：速度/位置环用 speed_ref 符号，电流环用 iq_ref 符号 */
    dir = (self->ctrl_mode == MCL_CTRL_SPEED || self->ctrl_mode == MCL_CTRL_POSITION)
        ? self->speed_ref_rpm : self->iq_ref;
    sign = (dir >= (mcl_scalar)0) ? MCL_FROM_FLOAT(1.0f) : MCL_FROM_FLOAT(-1.0f);

    /* 4. 迟滞进入：估计速度低于开环上限则向 hyst 饱和累加，否则递减 */
    if (MCL_ABS(*speed) < ol_speed_max)
    {
        if (self->ol_hyst_timer < self->cfg.openloop_hyst)
        {
            self->ol_hyst_timer = MCL_ADD(self->ol_hyst_timer, dt);
            if (self->ol_hyst_timer > self->cfg.openloop_hyst)
            {
                self->ol_hyst_timer = self->cfg.openloop_hyst;
            }
        }
    }
    else if (self->ol_hyst_timer > (mcl_scalar)0)
    {
        self->ol_hyst_timer = MCL_SUB(self->ol_hyst_timer, dt);
        if (self->ol_hyst_timer < (mcl_scalar)0)
        {
            self->ol_hyst_timer = (mcl_scalar)0;
        }
    }

    /* 5. 启动开环序列 */
    if (self->ol_hyst_timer >= self->cfg.openloop_hyst && self->ol_timer <= (mcl_scalar)0)
    {
        self->ol_timer = total;
        self->ol_phase = self->phase_rad; /* resume from the actual control frame */
        self->ol_lock_timer = (mcl_scalar)0;
        self->ol_wait_timer = (mcl_scalar)0;
        self->switch_blend_timer = (mcl_scalar)0;
        self->switch_phase_offset = (mcl_scalar)0;
    }

    /* 6. 开环运行 */
    if (self->ol_timer > (mcl_scalar)0)
    {
        /* 三段式时间序列：锁定(转速0，id 对齐) → 斜坡(15rpm→max) → 匀速(max)。
           首次与重开环同一条时间轴（t_lock=0 时不锁定）：
           牵入机制（应用层 IF 测试已验证）——磁场从静止以 ≤15rpm 起步，转子从
           任意位置都能被牵入；固定 0.1×openloop_rpm（80rpm）的旧地板跳变太大，
           转子跟不上 → 打滑后 I/F 平均转矩≈0 再也牵不回来（实测反电动势停在
           0.43~0.7V = 转子 100~200rpm 爬行）。重开环同样走斜坡：掉速后的转子
           从低转速重新被 15rpm→800rpm 斜坡牵回（旧「重开环直接全速」在转子
           掉到 ~200rpm 后无法牵入，只会原地爬行）。 */
        time_fwd = MCL_SUB(total, self->ol_timer);
        if (time_fwd < t_lock && self->ol_started_once == 0u)
        {
            ol_rpm = (mcl_scalar)0;
            self->ol_stage = 1;   /* 首次启动锁定：固定相位，id 对齐（当前 t_lock=0 已不用） */
        }
        else
        {
            self->ol_stage = 2;   /* 拖动：升余弦 S 形斜坡，从 15rpm 地板起步 */
            ol_rpm = ol_rpm_max;
            if (time_fwd < MCL_ADD(t_lock, t_ramp))
            {
                /* S 形：ol_rpm = max/2·(1 − cos(π·(t−t_lock)/t_ramp))。
                   起点/终点加速度 = 0：线性斜坡终点 α 突变（279rad/s²→0）会激发
                   转子的 I/F 摆动（ω_n≈4Hz 欠阻尼，实测撒手后转子以 ±250rpm 摆过
                   800→1046→…），摆动与速度环/观测器相互作用 → 闭环 ~1s 失锁。
                   S 形终点 α=0，转子在斜坡尾部已稳定跟随，切闭环无摆动。 */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
                mcl_math_sincos(MCL_MUL(MCL_DIV(MCL_SUB(time_fwd, t_lock), t_ramp),
                                        MCL_FROM_FLOAT(0.5f)), &s_ramp, &c_ramp);
#else
                mcl_math_sincos(MCL_MUL(MCL_DIV(MCL_SUB(time_fwd, t_lock), t_ramp),
                                        MCL_FROM_FLOAT(3.14159265f)), &s_ramp, &c_ramp);
#endif
                ol_rpm = MCL_MUL(MCL_MUL(ol_rpm_max, MCL_FROM_FLOAT(0.5f)),
                                 MCL_SUB(MCL_FROM_FLOAT(1.0f), c_ramp));
            }
            if (ol_rpm < MCL_FROM_FLOAT(15.0f))
            {
                ol_rpm = MCL_FROM_FLOAT(15.0f); /* 牵入地板 15rpm（=1.25Hz 场频，转子可靠牵入） */
            }
        }

        self->ol_speed = MCL_MUL(MCL_MUL(ol_rpm, rpm_to_espeed), sign);

        /* 锁定阶段相位固定；拖动阶段相位积分 */
        self->ol_phase = mcl_wrap_full_turn(MCL_ADD(self->ol_phase, mcl_speed_to_phase_incr(self->ol_speed, dt)));

        *phase = self->ol_phase;
        *speed = self->ol_speed;

        /* 拖动期间每拍把开环角速度写入 SMO（供其 Kslf 用已知 ω 替代未收敛的 w_est，
           让反电动势 e 在撒手前收敛到 ωλ；闭环后不再写，SMO 恢复自适应）。 */
        mcl_observer_seed_omega_set(&self->observer, self->ol_speed);

        self->ol_timer = MCL_SUB(self->ol_timer, dt);
        if (self->ol_timer <= (mcl_scalar)0)
        {
            self->ol_timer = (mcl_scalar)0;
            self->ol_started_once = 1u;   /* 首次开环序列已完成，之后重开环跳过锁定段 */

            /* Float board path: don't hand over solely because time elapsed.
             * Check independent SMO rotation and EMF amplitude, not the
             * overridden open-loop speed. Hold at the end of the ramp while
             * qualifying, and latch STALL if the observer never converges. */
#if !defined(MCL_USE_Q15) && !defined(MCL_USE_Q31)
            if (self->observer.ops == &mcl_observer_smo_ops)
            {
                mcl_observer_smo *obs = (mcl_observer_smo *)self->observer.impl;
                mcl_scalar obs_speed = obs->w_est / dt;
                mcl_scalar emf2 = obs->e_alpha_final * obs->e_alpha_final +
                                  obs->e_beta_final * obs->e_beta_final;
                mcl_scalar expected = MCL_ABS(self->ol_speed) * obs->params.flux * 0.31622777f;
                mcl_scalar error = obs->phase - self->ol_phase;
                while (error > MCL_PI) { error -= MCL_TWO_PI; }
                while (error < -MCL_PI) { error += MCL_TWO_PI; }
                if (MCL_ABS(obs_speed - self->ol_speed) < MCL_ABS(self->ol_speed) * 0.2f &&
                    emf2 > expected * expected * 0.25f &&
                    emf2 < expected * expected * 2.25f && MCL_ABS(error) < 1.74532925f)
                {
                    self->ol_lock_timer += dt;
                }
                else
                {
                    self->ol_lock_timer = 0.0f;
                }
                self->ol_wait_timer += dt;
                if (self->ol_lock_timer < MCL_SMO_LOCK_TIME)
                {
                    if (self->ol_wait_timer >= MCL_SMO_WAIT_TIME)
                    {
                        mcl_fault_assert(self, MCL_FAULT_STALL);
                    }
                    self->ol_timer = dt; /* stay at final drag speed, don't restart ramp */
                    self->ol_hyst_timer = 0.0f;
                    return;
                }
                self->pll.speed = obs_speed;
                self->pll.speed_est_fast = obs_speed;
            }
            else
#endif
            {
                self->pll.speed = self->ol_speed;
                self->pll.speed_est_fast = self->ol_speed;
            }
            self->pll.phase = self->pll.last_phase;
            self->switch_phase_offset = MCL_SUB(self->ol_phase, self->pll.phase);
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
            if (self->switch_phase_offset > MCL_FROM_FLOAT(0.5f))
                self->switch_phase_offset = MCL_SUB(self->switch_phase_offset, MCL_FROM_FLOAT(1.0f));
            if (self->switch_phase_offset < MCL_FROM_FLOAT(-0.5f))
                self->switch_phase_offset = MCL_ADD(self->switch_phase_offset, MCL_FROM_FLOAT(1.0f));
#else
            while (self->switch_phase_offset > MCL_PI) { self->switch_phase_offset -= MCL_TWO_PI; }
            while (self->switch_phase_offset < -MCL_PI) { self->switch_phase_offset += MCL_TWO_PI; }
#endif
            /* Keep the control frame continuous: PLL uses observer angle,
             * the frame offset decays together with the current reference.
             * No instantaneous rotation of the d/q PI state is then needed. */
            self->switch_blend_timer = MCL_FROM_FLOAT(MCL_SWITCH_BLEND_TIME);
            self->switch_blend_iq0 = (mcl_scalar)0;
            if (self->ctrl_mode != MCL_CTRL_CURRENT)
            {
                /* 斜坡从当前开环转速起，积分清零。预置成拖动电流（约 3A）会在
                   空载上把转子一下推过最终目标。 */
                self->speed_ramp_rpm = MCL_DIV(self->ol_speed, rpm_to_espeed);
                self->pid_speed.i_term = (mcl_scalar)0;
                self->pid_speed.prev_out = (mcl_scalar)0;
                self->pid_speed.prev_error = (mcl_scalar)0;
                self->iq_ref = (mcl_scalar)0;
            }
            mcl_observer_seed_omega_set(&self->observer, (mcl_scalar)0);
        }
        self->ol_hyst_timer = (mcl_scalar)0;
    }
    else
    {
        /* 非开环：相位跟随观测器估计，保证下次进入连续 */
        self->ol_stage = 0;
        if (self->switch_blend_timer > (mcl_scalar)0)
        {
            mcl_scalar blend = MCL_DIV(self->switch_blend_timer, MCL_FROM_FLOAT(MCL_SWITCH_BLEND_TIME));
            *phase = mcl_wrap_full_turn(MCL_ADD(*phase, MCL_MUL(self->switch_phase_offset, blend)));
        }
        self->ol_phase = *phase;
    }
}
#endif /* MCL_DISABLE_OBSERVER */

/* 速度环跟踪斜坡后的指令。rate<=0 时直接等于最终目标。 */
static mcl_scalar mcl_speed_ref_ramped(mcl *self, mcl_scalar dt)
{
    mcl_scalar target = self->speed_ref_rpm;
    mcl_scalar rate = self->cfg.speed_ramp_rpm_s;
    mcl_scalar cmd = self->speed_ramp_rpm;
    mcl_scalar step;
    mcl_scalar err;

    if (rate <= (mcl_scalar)0)
    {
        self->speed_ramp_rpm = target;
        return target;
    }
    step = MCL_MUL(rate, dt);
    err = MCL_SUB(target, cmd);
    if (err > step)
    {
        err = step;
    }
    if (err < MCL_NEG(step))
    {
        err = MCL_NEG(step);
    }
    cmd = MCL_ADD(cmd, err);
    self->speed_ramp_rpm = cmd;
    return cmd;
}

static void mcl_control_tick_foc(mcl *self)
{
    mcl_scalar ia;
    mcl_scalar ib;
    mcl_scalar ic;
    mcl_scalar i_alpha;
    mcl_scalar i_beta;
    mcl_scalar id;
    mcl_scalar iq;
    mcl_scalar id_ref;
    mcl_scalar iq_ref;
    mcl_scalar vd;
    mcl_scalar vq;
    mcl_scalar v_alpha;
    mcl_scalar v_beta;
    mcl_scalar da;
    mcl_scalar db;
    mcl_scalar dc;
    mcl_scalar phase;
    mcl_scalar speed;
    mcl_scalar vbus;
    mcl_scalar temp_max;
    mcl_scalar derate;
    mcl_fault fault;

    /* 1. 采样（减零漂） */
    if (self->hal->adc_read_phase == NULL)
    {
        /* 无电流采样能力：无法运行 FOC 电流环（ia/ib/ic 未定义会导致 UB），
           关断 PWM 安全停机，不进控制环。 */
        if (self->hal->pwm_set_duty != NULL)
        {
            self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        }
        return;
    }
    if (self->hal->adc_read_phase(self->hal_ctx, &ia, &ib, &ic) != MCL_OK)
    {
        /* ADC 采样失败：关断 PWM 安全停机，不伪造故障码。
           具体故障原因由宿主在 HAL 出错时自行 mcl_fault_assert 上报（如 MCL_FAULT_DRV）。 */
        if (self->hal->pwm_set_duty != NULL)
        {
            self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        }
        return;
    }
    ia = MCL_SUB(ia, self->cfg.current_offset[0]);
    ib = MCL_SUB(ib, self->cfg.current_offset[1]);
    ic = MCL_SUB(ic, self->cfg.current_offset[2]);

    vbus = self->cfg.bus_voltage;
    if (self->hal->adc_read_bus != NULL)
    {
        mcl_scalar ibus;
        (void)self->hal->adc_read_bus(self->hal_ctx, &vbus, &ibus);
    }
    self->vbus = vbus;

    /* 读温度（电机/FET 取更高者） */
    temp_max = (mcl_scalar)0;
    if (self->hal->read_temp != NULL)
    {
        mcl_scalar t_motor = (mcl_scalar)0;
        mcl_scalar t_fet = (mcl_scalar)0;
        if (self->hal->read_temp(self->hal_ctx, &t_motor, &t_fet) == MCL_OK)
        {
            temp_max = (t_motor > t_fet) ? t_motor : t_fet;
        }
    }
    derate = mcl_protection_derate(&self->protection, temp_max);

    /* 2. Clarke */
    mcl_transform_clarke(ia, ib, ic, &i_alpha, &i_beta);

    /* 3. 相位 / 速度 */
    phase = self->phase_rad;
    speed = self->speed_rad_s;

    if (self->mode == MCL_MODE_FOC_SENSORED)
    {
        if (self->hal->enc_read_angle != NULL)
        {
            (void)self->hal->enc_read_angle(self->hal_ctx, &phase);
        }
        if (self->hal->enc_read_speed != NULL)
        {
            (void)self->hal->enc_read_speed(self->hal_ctx, &speed);
        }
    }
    else /* MCL_MODE_FOC_SENSORLESS */
    {
#ifndef MCL_DISABLE_OBSERVER
        /* 观测器（用上一周期电压）估相位，PLL 跟踪 + 估速度。
           电压归一化约定（统一）：mcl 的 v_alpha/v_beta 是「母线归一化」
           （1.0 = 母线 vbus，SVPWM 内 da=((va+v0)*0.5+0.5)*max_duty 的 0.5 映射
           即把母线归一化转到相电压）。观测器接口输入是「相电压物理量 V」，
           故 v_pu × vbus/2 转物理相电压。 */
        mcl_observer_update(&self->observer,
                            MCL_MUL(self->v_alpha_prev, MCL_MUL(vbus, MCL_FROM_FLOAT(0.5f))),
                            MCL_MUL(self->v_beta_prev, MCL_MUL(vbus, MCL_FROM_FLOAT(0.5f))),
                            i_alpha, i_beta, self->dt, &phase, NULL);

        /* SMO supplies compensated rotor angle; PLL tracks angle and speed. */
        mcl_pll_run(&self->pll, phase, self->dt, &phase, &speed);

        /* 自动开环→闭环切换：闭环控制模式下，低速段开环拖动 */
        if (self->ctrl_mode == MCL_CTRL_CURRENT ||
            self->ctrl_mode == MCL_CTRL_SPEED ||
            self->ctrl_mode == MCL_CTRL_POSITION)
        {
            mcl_openloop_auto(self, &phase, &speed);
            if (self->state != MCL_STATE_RUN) { return; }
        }
#endif
    }

    /* 开环：相位/速度覆盖（绕过编码器与观测器） */
#ifndef MCL_DISABLE_OPENLOOP
    if (self->ctrl_mode == MCL_CTRL_OPENLOOP_VF ||
        self->ctrl_mode == MCL_CTRL_OPENLOOP_IF)
    {
        /* 旋转矢量：相位按 openloop_speed 积分斜坡前进 */
        self->openloop_angle = mcl_wrap_full_turn(
            MCL_ADD(self->openloop_angle, mcl_speed_to_phase_incr(self->openloop_speed, self->dt)));
        phase = self->openloop_angle;
        speed = self->openloop_speed;
    }
    else if (self->ctrl_mode == MCL_CTRL_OPENLOOP_ALIGN)
    {
        /* 固定矢量：相位锁定（转子预定位） */
        phase = self->openloop_phase;
        speed = (mcl_scalar)0;
    }
#endif

    self->phase_rad = phase;
    self->speed_rad_s = speed;

    /* 4. 外环（位置环 → 速度环 → 电流环，按分频级联） */
    iq_ref = self->iq_ref;

    /* 位置环（最外环）：输出速度参考 rpm */
#ifndef MCL_DISABLE_POSITION
    if (self->ctrl_mode == MCL_CTRL_POSITION && self->cfg.pos_loop_divider > 0u)
    {
        if ((self->tick_count % (uint32_t)self->cfg.pos_loop_divider) == 0u)
        {
            /* 机械角 = 电气角 / pole_pairs；乘倒数 1/pole_pairs（<1，定点可表示） */
            mcl_scalar pos_now = MCL_MUL(phase, MCL_FROM_FLOAT(1.0f / (float)self->cfg.pole_pairs));
            self->speed_ref_rpm = mcl_pid_run(&self->pid_pos,
                                              MCL_SUB(self->pos_ref_rad, pos_now),
                                              MCL_MUL(self->dt, (mcl_scalar)self->cfg.pos_loop_divider));
        }
    }
#endif

    /* 速度环：反馈转速 rpm = 电气角速度 rad/s ÷ 极对数 × 60/(2π) */
    if (self->ctrl_mode == MCL_CTRL_SPEED || self->ctrl_mode == MCL_CTRL_POSITION)
    {
        if (self->ol_stage == 0u && self->cfg.speed_loop_divider > 0u &&
            (self->tick_count % (uint32_t)self->cfg.speed_loop_divider) == 0u)
        {
            mcl_scalar speed_dt = MCL_MUL(self->dt, (mcl_scalar)self->cfg.speed_loop_divider);
            mcl_scalar ref_rpm = mcl_speed_ref_ramped(self, speed_dt);
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
            /* 定点：speed 与 speed_ref_rpm 均已归一化（HAL/调用方在边界分别 ÷电气速度基值
               / ÷rpm 基值；两者数值相等，故直接比较，无 rpm↔rad/s 换算。
               详见 docs/spec/mcl_fixed_point.md） */
            iq_ref = mcl_pid_run(&self->pid_speed,
                                 MCL_SUB(ref_rpm, speed),
                                 speed_dt);
#else
            /* 速度环反馈直接取 PLL 瞬时估速，不再低通滤波。
               历史教训：曾加 20Hz 反馈滤波，但滤波滞后让速度环永远"追不上"
               PLL 已爬升的原始速度 → 正反馈持续爬升（实测 spd 一路 157→259rad/s
               不回头、avg 320~385 偏高）。去掉滤波后反馈即时，速度环才能锁住 300。 */
            mcl_scalar fb_rpm = MCL_MUL(speed, MCL_FROM_FLOAT(MCL_RPM_PER_RAD_S /
                                        (float)self->cfg.pole_pairs));
            iq_ref = mcl_pid_run(&self->pid_speed,
                                 MCL_SUB(ref_rpm, fb_rpm),
                                 speed_dt);
#endif
        }
        self->iq_ref = iq_ref;

        /* 角度掺混仍用这个计时。电流不再从拖动电流掺进来，否则斜坡期间 iq 一直是 3A。 */
        if (self->switch_blend_timer > (mcl_scalar)0)
        {
            self->switch_blend_timer = MCL_SUB(self->switch_blend_timer, self->dt);
            if (self->switch_blend_timer < (mcl_scalar)0)
            {
                self->switch_blend_timer = (mcl_scalar)0;
            }
        }
    }

    if (self->ctrl_mode == MCL_CTRL_CURRENT && self->switch_blend_timer > (mcl_scalar)0)
    {
        mcl_scalar blend = MCL_DIV(self->switch_blend_timer, MCL_FROM_FLOAT(MCL_SWITCH_BLEND_TIME));
        iq_ref = MCL_ADD(MCL_MUL(self->switch_blend_iq0, blend),
                        MCL_MUL(iq_ref, MCL_SUB(MCL_FROM_FLOAT(1.0f), blend)));
        self->switch_blend_timer = MCL_SUB(self->switch_blend_timer, self->dt);
        if (self->switch_blend_timer < (mcl_scalar)0) { self->switch_blend_timer = (mcl_scalar)0; }
    }

    /* 温度降额：限制 iq 幅值（derate 满幅 = 不降额） */
    if (derate < MCL_FROM_FLOAT(1.0f))
    {
        mcl_scalar iq_limit = MCL_MUL(self->cfg.rated_current, derate);
        if (iq_ref > iq_limit) { iq_ref = iq_limit; }
        if (iq_ref < MCL_NEG(iq_limit)) { iq_ref = MCL_NEG(iq_limit); }
    }

    /* 5. 电流环：MTPA/弱磁 → Park → PI + 解耦前馈 → 反 Park → SVPWM */
    if (self->ctrl_mode == MCL_CTRL_OPENLOOP_VF)
    {
        /* VF：全开环，直接输出旋转电压矢量（vd = 幅值，vq = 0），电流环不闭合。
           id/iq 仍做 Park 供遥测展示实际 dq 电流。 */
        mcl_transform_park(i_alpha, i_beta, phase, &id, &iq);
        vd = self->openloop_mag;
        vq = (mcl_scalar)0;
    }
    else
    {
        mcl_scalar iq_ref_loop = iq_ref;

        if (self->ctrl_mode == MCL_CTRL_OPENLOOP_IF)
        {
            /* I/F：电流环闭环，相位开环（iq = 幅值，id = 0），不做 MTPA */
            id_ref = (mcl_scalar)0;
            iq_ref_loop = self->openloop_mag;
        }
        else if (self->ctrl_mode == MCL_CTRL_OPENLOOP_ALIGN)
        {
            /* 预定位：固定相位，电流矢量沿 d 轴（id = 幅值，iq = 0），不做 MTPA */
            id_ref = self->openloop_mag;
            iq_ref_loop = (mcl_scalar)0;
        }
        else if (self->ol_stage == 1u)
        {
            /* 自动开环锁定：id 对齐预定位，把转子吸到开环相位。
               对齐电流用 openloop_drag_q（≈拖动电流），不用 rated_current，
               避免 4A 级对齐电流触发硬件过流（空载对齐 1A 足够克服齿槽）。 */
            id_ref = self->cfg.openloop_drag_q;
            iq_ref_loop = (mcl_scalar)0;
        }
        else if (self->ol_stage == 2u)
        {
            /* 自动开环拖动：I/F 固定 q 轴电流（带方向），id=0，相位开环积分 */
            mcl_scalar dir = (self->ctrl_mode == MCL_CTRL_SPEED || self->ctrl_mode == MCL_CTRL_POSITION)
                           ? self->speed_ref_rpm : self->iq_ref;
            id_ref = (mcl_scalar)0;
            iq_ref_loop = (dir >= (mcl_scalar)0) ? self->cfg.openloop_drag_q
                                                 : MCL_NEG(self->cfg.openloop_drag_q);
        }
        else
        {
            mcl_mtpa_fw_id_ref(&self->mtpa_fw, iq_ref, speed, vbus, &id_ref);
        }

        mcl_transform_park(i_alpha, i_beta, phase, &id, &iq);
        mcl_foc_run(&self->foc, id, iq, id_ref, iq_ref_loop, speed, vbus, self->dt, &vd, &vq);
    }
    mcl_transform_inv_park(vd, vq, phase, &v_alpha, &v_beta);
    mcl_svpwm_run(v_alpha, v_beta, self->cfg.max_duty, &da, &db, &dc);

    /* 从最终占空比重建实际平均相电压，含 max_duty 缩放和调制钳位。
       缓存单位为 vbus/2；下拍再乘母线电压转为 V。 */
    self->v_alpha_prev = MCL_FROM_FLOAT((2.0f / 3.0f) *
        (2.0f * MCL_TO_FLOAT(da) - MCL_TO_FLOAT(db) - MCL_TO_FLOAT(dc)));
    self->v_beta_prev = MCL_FROM_FLOAT((2.0f / 1.73205080757f) *
        (MCL_TO_FLOAT(db) - MCL_TO_FLOAT(dc)));

    /* 6. 保护 */
    fault = mcl_protection_check(&self->protection, ia, ib, ic, vbus,
                                 temp_max, speed, self->dt);
    if (fault != MCL_FAULT_NONE)
    {
        self->fault = fault;
        self->state = MCL_STATE_FAULT;
        /* 记录故障现场快照（用于诊断） */
        self->fault_info.fault = fault;
        self->fault_info.current = iq;
        self->fault_info.voltage = vbus;
        self->fault_info.speed = speed;
        self->fault_info.temp = temp_max;
        self->fault_info.tick = self->tick_count;
        self->fault_timer = (mcl_scalar)0;
        if (self->hal->pwm_set_duty != NULL)
        {
            self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        }
        return;
    }

    /* 7. 输出 */
    if (self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, da, db, dc);
    }

    /* 8. 更新遥测缓存 */
    self->id_now = id;
    self->iq_now = iq;
    self->duty_now = da;
}

#ifndef MCL_DISABLE_BLDC
static void mcl_control_tick_bldc(mcl *self)
{
    mcl_scalar da;
    mcl_scalar db;
    mcl_scalar dc;
    mcl_scalar duty = self->duty_now;

    if (self->hal == NULL)
    {
        return;
    }

    if (self->mode == MCL_MODE_BLDC_HALL)
    {
        /* 霍尔有感换相 */
        if (self->hal->read_hall == NULL)
        {
            return;
        }
        {
            uint8_t hall = 0u;
            if (self->hal->read_hall(self->hal_ctx, &hall) != MCL_OK)
            {
                return;
            }
            mcl_bldc_comm_step_hall(&self->bldc, hall, duty, &da, &db, &dc);
        }
    }
    else /* MCL_MODE_BLDC_SENSORLESS */
    {
        /* 无感 BEMF 换相：读三相端电压 */
        if (self->hal->adc_read_phase_voltage == NULL)
        {
            return;
        }
        {
            mcl_scalar va;
            mcl_scalar vb;
            mcl_scalar vc;
            if (self->hal->adc_read_phase_voltage(self->hal_ctx, &va, &vb, &vc) != MCL_OK)
            {
                return;
            }
            mcl_bldc_comm_step_bemf(&self->bldc, va, vb, vc, duty, self->dt,
                                    &da, &db, &dc);
        }
    }

    if (self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, da, db, dc);
    }
}
#endif /* MCL_DISABLE_BLDC */

/* ============================ 生命周期 ============================ */

int mcl_init(mcl *self, const mcl_config *cfg,
             const mcl_hal_ops *hal, void *hal_ctx,
             const void *obs_ops, void *obs_impl, void *obs_params)
{
    (void)obs_ops; (void)obs_impl; (void)obs_params;   /* 裁剪 observer 时未用 */
    if (self == NULL || cfg == NULL || hal == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 配置校验：非法参数直接拒绝初始化，不进 FAULT（参数错误 ≠ 运行时故障）。
       校验失败必须显式返回错误码，并把 hal 置空，避免 mcl_start 把 state 置 RUN
       后 mcl_control_tick 因 hal==NULL 每拍静默 return（tick 不增、duty 不更新）。 */
    if (mcl_config_validate(cfg) != MCL_OK)
    {
        self->hal = NULL;
        self->hal_ctx = NULL;
        self->state = MCL_STATE_IDLE;
        self->fault = MCL_FAULT_NONE;
        return MCL_ERR_PARAM;
    }

    self->cfg = *cfg;
    self->hal = hal;
    self->hal_ctx = hal_ctx;
    self->mode = MCL_MODE_FOC_SENSORLESS;
    self->state = MCL_STATE_IDLE;
    self->fault = MCL_FAULT_NONE;
    self->ctrl_mode = MCL_CTRL_CURRENT;

    mcl_foc_init(&self->foc, cfg);
#ifndef MCL_DISABLE_BLDC
    mcl_bldc_comm_init(&self->bldc);
#endif
#ifndef MCL_DISABLE_OBSERVER
    mcl_observer_init(&self->observer, (const mcl_observer_ops *)obs_ops, obs_impl, obs_params);
    mcl_pll_init(&self->pll, cfg->pll_kp, cfg->pll_ki);
#endif
    mcl_pid_init(&self->pid_speed, &cfg->speed_pid);
    mcl_pid_init(&self->pid_pos, &cfg->pos_pid);
    mcl_mtpa_fw_init(&self->mtpa_fw, cfg->phase_inductance, cfg->phase_inductance,
                     cfg->bemf_const, cfg->rated_current);
    mcl_protection_init(&self->protection, &cfg->limits);

    self->iq_ref = (mcl_scalar)0;
    self->speed_ref_rpm = (mcl_scalar)0;
    self->speed_ramp_rpm = (mcl_scalar)0;
    self->pos_ref_rad = (mcl_scalar)0;
    self->phase_rad = (mcl_scalar)0;
    self->speed_rad_s = (mcl_scalar)0;
    self->fb_speed_filt = (mcl_scalar)0;
    self->vbus = cfg->bus_voltage;
    self->id_now = (mcl_scalar)0;
    self->iq_now = (mcl_scalar)0;
    self->duty_now = (mcl_scalar)0;
    self->v_alpha_prev = (mcl_scalar)0;
    self->v_beta_prev = (mcl_scalar)0;
    self->openloop_speed = (mcl_scalar)0;
    self->openloop_angle = (mcl_scalar)0;
    self->openloop_phase = (mcl_scalar)0;
    self->openloop_mag = (mcl_scalar)0;
    self->ol_timer = (mcl_scalar)0;
    self->ol_hyst_timer = (mcl_scalar)0;
    self->ol_anchor_timer = (mcl_scalar)0;
    self->ol_speed = (mcl_scalar)0;
    self->ol_phase = (mcl_scalar)0;
    self->ol_stage = 0u;
    self->ol_started_once = 0u;
    self->switch_blend_timer = (mcl_scalar)0;
    self->switch_blend_iq0 = (mcl_scalar)0;
    self->switch_phase_offset = (mcl_scalar)0;
    self->ol_lock_timer = (mcl_scalar)0;
    self->ol_wait_timer = (mcl_scalar)0;
    self->tick_count = 0u;
    self->fault_timer = (mcl_scalar)0;
    self->fault_info.fault = MCL_FAULT_NONE;
    self->fault_info.current = (mcl_scalar)0;
    self->fault_info.voltage = (mcl_scalar)0;
    self->fault_info.speed = (mcl_scalar)0;
    self->fault_info.temp = (mcl_scalar)0;
    self->fault_info.tick = 0u;

    /* dt = 1/电流环频率 ÷ time_base 归一化（per-unit 时间 dt_pu = dt/T_BASE = dt·W_BASE）。
       time_base 默认 1.0 时保持物理秒；定点设 T_BASE（<1）消除 dt 的 Q15 量化误差。 */
    self->dt = MCL_DIV(MCL_FROM_FLOAT(1.0f / (float)cfg->current_loop_freq_hz), cfg->time_base);

    return MCL_OK;
}

void mcl_deinit(mcl *self)
{
    if (self == NULL)
    {
        return;
    }

    if (self->hal != NULL && self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    }
    self->state = MCL_STATE_IDLE;
}

int mcl_set_config(mcl *self, const mcl_config *cfg)
{
    if (self == NULL || cfg == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 更新可运行时调整的字段（快照） */
    self->cfg.current_pid = cfg->current_pid;
    self->cfg.speed_pid = cfg->speed_pid;
    self->cfg.pos_pid = cfg->pos_pid;
    self->cfg.limits = cfg->limits;
    self->cfg.fault_stop_time = cfg->fault_stop_time;
    self->cfg.max_duty = cfg->max_duty;
    self->cfg.speed_ramp_rpm_s = cfg->speed_ramp_rpm_s;
    self->cfg.openloop_rpm = cfg->openloop_rpm;
    self->cfg.openloop_drag_q = cfg->openloop_drag_q;
    self->cfg.openloop_seed_angle = cfg->openloop_seed_angle;

    /* 同步到子模块 */
    mcl_pid_set_params(&self->foc.pid_d, &cfg->current_pid);
    mcl_pid_set_params(&self->foc.pid_q, &cfg->current_pid);
    mcl_pid_set_params(&self->pid_speed, &cfg->speed_pid);
    mcl_pid_set_params(&self->pid_pos, &cfg->pos_pid);
    self->protection.limits = cfg->limits;   /* 只更新阈值，保留堵转计时状态 */

    return MCL_OK;
}

int mcl_get_config(mcl *self, mcl_config *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }
    *out = self->cfg;
    return MCL_OK;
}

/* ============================ 模式与启停 ============================ */

int mcl_set_mode(mcl *self, mcl_mode mode)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    if (self->state == MCL_STATE_RUN)
    {
        return MCL_ERR_STATE;
    }
    self->mode = mode;
    return MCL_OK;
}

int mcl_start(mcl *self)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    if (self->state == MCL_STATE_RUN)
    {
        return MCL_OK;
    }
    if (self->fault != MCL_FAULT_NONE)
    {
        return MCL_ERR_STATE;
    }

    self->state = MCL_STATE_RUN;
    self->tick_count = 0u;
    self->ol_timer = (mcl_scalar)0;
    self->ol_hyst_timer = (mcl_scalar)0;
    self->ol_lock_timer = (mcl_scalar)0;
    self->ol_wait_timer = (mcl_scalar)0;
    self->ol_stage = 0u;
    self->ol_started_once = 0u;
    self->switch_blend_timer = (mcl_scalar)0;
    self->switch_blend_iq0 = (mcl_scalar)0;
    self->switch_phase_offset = (mcl_scalar)0;
    self->speed_ramp_rpm = (mcl_scalar)0;
    self->v_alpha_prev = (mcl_scalar)0;
    self->v_beta_prev = (mcl_scalar)0;
    mcl_pid_reset(&self->foc.pid_d);
    mcl_pid_reset(&self->foc.pid_q);
    mcl_pid_reset(&self->pid_speed);
#ifndef MCL_DISABLE_OBSERVER
    mcl_observer_reset(&self->observer);
    mcl_pll_reset(&self->pll);
#endif

    /* 无感闭环启动：预充电开环迟滞，使首拍即进入开环（预定位 + 拖动），
       避免低速段观测器未收敛时闭环乱甩转子（VESC 式自动开环启动） */
    if (self->mode == MCL_MODE_FOC_SENSORLESS &&
        (self->ctrl_mode == MCL_CTRL_CURRENT ||
         self->ctrl_mode == MCL_CTRL_SPEED ||
         self->ctrl_mode == MCL_CTRL_POSITION))
    {
        self->ol_hyst_timer = self->cfg.openloop_hyst;
    }

    return MCL_OK;
}

int mcl_stop(mcl *self)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    if (self->hal != NULL && self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    }
    self->state = MCL_STATE_IDLE;
    return MCL_OK;
}

/* ============================ 指令 ============================ */

int mcl_set_current(mcl *self, mcl_scalar iq_ref)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    self->ctrl_mode = MCL_CTRL_CURRENT;
    self->iq_ref = iq_ref;
    return MCL_OK;
}

int mcl_set_speed(mcl *self, mcl_scalar speed_rpm)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    self->ctrl_mode = MCL_CTRL_SPEED;
    self->speed_ref_rpm = speed_rpm;
    return MCL_OK;
}

#ifndef MCL_DISABLE_POSITION
int mcl_set_position(mcl *self, mcl_scalar pos_rad)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    self->ctrl_mode = MCL_CTRL_POSITION;
    self->pos_ref_rad = pos_rad;
    return MCL_OK;
}
#endif

int mcl_set_torque(mcl *self, mcl_scalar torque_nm)
{
    mcl_scalar kt;

    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* iq = T / (1.5 · p · λ) = (T / λ) · (2 / (3p))。
       把 1.5=3/2 改写为乘法倒数 2/(3p)（≤ 2/3 <1），避免 1.5 在 Q15/Q31 下溢出。
       定点提示：T 与 λ 仍需按转矩/磁链 per-unit 基值归一化（见 mcl_fixed_point.md）。 */
    kt = MCL_FROM_FLOAT(2.0f / (3.0f * (float)self->cfg.pole_pairs));
    self->ctrl_mode = MCL_CTRL_CURRENT;
    self->iq_ref = MCL_DIV(MCL_DIV(torque_nm, self->cfg.bemf_const), kt);
    return MCL_OK;
}

/* ============================ 开环指令 ============================ */

#ifndef MCL_DISABLE_OPENLOOP

int mcl_set_openloop_vf(mcl *self, mcl_scalar voltage, mcl_scalar speed_rpm)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 电压幅值限幅到 [-1, 1]（标幺，1.0 = 满母线；定点用满幅表示） */
    if (voltage > MCL_FROM_FLOAT(1.0f)) { voltage = MCL_FROM_FLOAT(1.0f); }
    if (voltage < MCL_FROM_FLOAT(-1.0f)) { voltage = MCL_FROM_FLOAT(-1.0f); }

    self->ctrl_mode = MCL_CTRL_OPENLOOP_VF;
    self->openloop_mag = voltage;
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    /* 定点：rpm_pu == 电气速度_pu（见 mcl_fixed_point.md），直接存，不做 rpm↔rad/s 换算。 */
    self->openloop_speed = speed_rpm;
#else
    self->openloop_speed = MCL_FROM_FLOAT(MCL_RAD_PER_S_PER_RPM * (float)self->cfg.pole_pairs);
    self->openloop_speed = MCL_MUL(speed_rpm, self->openloop_speed);
#endif
    return MCL_OK;
}

int mcl_set_openloop_if(mcl *self, mcl_scalar current, mcl_scalar speed_rpm)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 电流幅值限幅到额定电流 */
    if (current > self->cfg.rated_current) { current = self->cfg.rated_current; }
    if (current < MCL_NEG(self->cfg.rated_current)) { current = MCL_NEG(self->cfg.rated_current); }

    self->ctrl_mode = MCL_CTRL_OPENLOOP_IF;
    self->openloop_mag = current;
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    /* 定点：rpm_pu == 电气速度_pu，直接存。 */
    self->openloop_speed = speed_rpm;
#else
    self->openloop_speed = MCL_FROM_FLOAT(MCL_RAD_PER_S_PER_RPM * (float)self->cfg.pole_pairs);
    self->openloop_speed = MCL_MUL(speed_rpm, self->openloop_speed);
#endif
    return MCL_OK;
}

int mcl_set_openloop_align(mcl *self, mcl_scalar current, mcl_scalar phase_rad)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 电流幅值限幅到额定电流 */
    if (current > self->cfg.rated_current) { current = self->cfg.rated_current; }
    if (current < MCL_NEG(self->cfg.rated_current)) { current = MCL_NEG(self->cfg.rated_current); }

    self->ctrl_mode = MCL_CTRL_OPENLOOP_ALIGN;
    self->openloop_mag = current;
    self->openloop_phase = mcl_wrap_full_turn(phase_rad);
    /* 预定位固定相位，不转动 */
    self->openloop_speed = (mcl_scalar)0;
    return MCL_OK;
}

#endif /* MCL_DISABLE_OPENLOOP */

/* ============================ 查询 ============================ */

int mcl_get_state(mcl *self, mcl_state *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }
    *out = self->state;
    return MCL_OK;
}

int mcl_get_fault(mcl *self, mcl_fault *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }
    *out = self->fault;
    return MCL_OK;
}

int mcl_get_fault_info(mcl *self, mcl_fault_info *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }
    *out = self->fault_info;
    return MCL_OK;
}

int mcl_fault_assert(mcl *self, mcl_fault fault)
{
    if (self == NULL || fault == MCL_FAULT_NONE)
    {
        return MCL_ERR_PARAM;
    }

    self->fault = fault;
    self->state = MCL_STATE_FAULT;

    /* 记录快照（硬件保护在中断里，用最近缓存值） */
    self->fault_info.fault = fault;
    self->fault_info.current = self->iq_now;
    self->fault_info.voltage = self->vbus;
    self->fault_info.speed = self->speed_rad_s;
    self->fault_info.temp = (mcl_scalar)0;
    self->fault_info.tick = self->tick_count;
    self->fault_timer = (mcl_scalar)0;

    /* 立即关断 PWM */
    if (self->hal != NULL && self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    }

    return MCL_OK;
}

int mcl_clear_fault(mcl *self)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    if (self->state == MCL_STATE_RUN)
    {
        return MCL_ERR_STATE;
    }
    self->fault = MCL_FAULT_NONE;
    self->state = MCL_STATE_IDLE;
    self->fault_timer = (mcl_scalar)0;
    return MCL_OK;
}

int mcl_get_telemetry(mcl *self, mcl_telemetry *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* speed_rad_s 为电气角速度 rad/s（float）或归一化速度（定点）。
       转速 rpm = 电气 rad/s ÷ pole_pairs × 60/(2π)。定点下归一化电气速度 == 归一化 rpm。 */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    out->speed_rpm = self->speed_rad_s;   /* 定点：已归一化，数值即归一化 rpm */
#else
    out->speed_rpm = MCL_MUL(self->speed_rad_s,
                             MCL_FROM_FLOAT(MCL_RPM_PER_RAD_S / (float)self->cfg.pole_pairs));
#endif
    out->position_rad = self->phase_rad;
    out->iq = self->iq_now;
    out->id = self->id_now;
    out->vbus = self->vbus;
    out->ibus = (mcl_scalar)0;
    out->duty = self->duty_now;
    out->temp_motor = (mcl_scalar)0;
    out->temp_fet = (mcl_scalar)0;
    out->est_phase = self->phase_rad;
    out->est_speed_rad_s = self->speed_rad_s;
    return MCL_OK;
}

/* ============================ 控制节拍 ============================ */

void mcl_control_tick(mcl *self)
{
    if (self == NULL || self->hal == NULL)
    {
        return;
    }

    /* 故障自动恢复计时（VESC 式：fault_stop_time 后自动恢复） */
    if (self->state == MCL_STATE_FAULT && self->cfg.fault_stop_time > (mcl_scalar)0)
    {
        self->fault_timer = MCL_ADD(self->fault_timer, self->dt);
        if (self->fault_timer >= self->cfg.fault_stop_time)
        {
            self->state = MCL_STATE_IDLE;
            self->fault = MCL_FAULT_NONE;
            self->fault_timer = (mcl_scalar)0;
        }
    }

    if (self->state != MCL_STATE_RUN)
    {
        return;
    }

    if (self->mode == MCL_MODE_FOC_SENSORED || self->mode == MCL_MODE_FOC_SENSORLESS)
    {
        mcl_control_tick_foc(self);
    }
#ifndef MCL_DISABLE_BLDC
    else
    {
        mcl_control_tick_bldc(self);
    }
#endif

    self->tick_count++;
}

/* ============================ 校准封装 ============================ */

#ifndef MCL_DISABLE_CALIBRATION

int mcl_calibrate_offset(mcl *self)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    return mcl_cal_current_offset(self->hal, self->hal_ctx, 256u, self->cfg.current_offset);
}

int mcl_calibrate_align(mcl *self)
{
    mcl_scalar offset;
    int ret;

    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    ret = mcl_cal_encoder_align(self->hal, self->hal_ctx, &self->cfg,
                                      self->cfg.max_duty, &offset);
    if (ret == MCL_OK)
    {
        self->cfg.feedback.encoder_offset = offset;
    }
    return ret;
}

int mcl_calibrate_resistance(mcl *self, mcl_scalar *resistance)
{
    if (self == NULL || resistance == NULL)
    {
        return MCL_ERR_PARAM;
    }
    return mcl_cal_resistance(self->hal, self->hal_ctx, &self->cfg,
                                    self->cfg.max_duty, resistance);
}

int mcl_calibrate_inductance(mcl *self, mcl_scalar *inductance)
{
    if (self == NULL || inductance == NULL)
    {
        return MCL_ERR_PARAM;
    }
    return mcl_cal_inductance(self->hal, self->hal_ctx, &self->cfg,
                                    self->cfg.max_duty, inductance);
}

#endif /* MCL_DISABLE_CALIBRATION */

