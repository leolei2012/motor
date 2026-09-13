#include "rcm.h"
#include <stddef.h>

/** ============================================================
   初始化 RCM 实例，绑定硬件操作接口
   ============================================================ */
void rcm_init(rcm_t *rcm, const rcm_adc_ops_t *ops, void *user_data)
{
    if (rcm == NULL || ops == NULL)
    {
        return;
    }

    rcm->count     = 0;
    rcm->index     = 0;
    rcm->ops       = *ops;
    rcm->user_data = user_data;
    rcm->init_done = false;
}

/** ============================================================
   注册一个通道到轮询队列

   首次注册该 RCM 实例时，自动调用 ops.init() 做硬件初始化。
   返回 true 成功，false 失败（队列满或参数非法）。
   ============================================================ */
bool rcm_register(rcm_t *rcm, rcm_conv_t *conv)
{
    if (rcm == NULL || conv == NULL)
    {
        return false;
    }

    if (rcm->count >= RCM_MAX_CONV)
    {
        return false;
    }

    /** 加入队列 */
    conv->id = rcm->count;
    rcm->slots[rcm->count] = conv;
    rcm->count++;

    /** 首次初始化 */
    if (!rcm->init_done)
    {
        rcm->ops.init(rcm->user_data);
        rcm->init_done = true;
    }

    rcm->ops.set_sample_time(conv->channel, conv->sample_time, rcm->user_data);

    return true;
}

/** ============================================================
   读取上一次转换结果，推进指针

   调用时机：中断开头
   - 检查 EOC：完成 → 写结果到 conv->data，未完成 → 跳过
   - 指针环形 +1
   ============================================================ */
void rcm_read_ongoing(rcm_t *rcm)
{
    if (rcm == NULL || rcm->count == 0)
    {
        return;
    }

    rcm_conv_t *conv = rcm->slots[rcm->index];

    if (rcm->ops.is_conversion_done(rcm->user_data))
    {
        conv->data = rcm->ops.read_result(rcm->user_data);
    }

    /** 环形推进 */
    if (rcm->index >= rcm->count - 1)
    {
        rcm->index = 0;
    }
    else
    {
        rcm->index++;
    }
}

/** ============================================================
   启动下一个转换（非阻塞）

   调用时机：中断里，紧接 rcm_read_ongoing() 之后
   设置通道 → 触发转换 → 立即返回，ADC 在后台跑
   ============================================================ */
void rcm_exec_next(rcm_t *rcm)
{
    if (rcm == NULL || rcm->count == 0)
    {
        return;
    }

    rcm_conv_t *conv = rcm->slots[rcm->index];

    rcm->ops.start_conversion(conv->channel, rcm->user_data);
}

/** ============================================================
   同步阻塞执行一次转换

   与 rcm_exec_next 不同，这里会等待 ADC 完成。
   仅用于初始化验证或调试，不要在中断里调用。
   ============================================================ */
uint16_t rcm_exec_blocking(rcm_t *rcm, rcm_conv_t *conv)
{
    if (rcm == NULL || conv == NULL)
    {
        return 0;
    }

    rcm->ops.start_conversion(conv->channel, rcm->user_data);

    while (!rcm->ops.is_conversion_done(rcm->user_data))
    {
        /** 等待 */
    }

    return rcm->ops.read_result(rcm->user_data);
}

/** ============================================================
   等待当前转换完成（不读结果）
   ============================================================ */
void rcm_wait(rcm_t *rcm)
{
    if (rcm == NULL || rcm->count == 0)
    {
        return;
    }

    while (!rcm->ops.is_conversion_done(rcm->user_data))
    {
        /** 等待 */
    }
}