/** @file modbus_slave.c
 *  @brief Modbus RTU 从机实现：寄存器段查找 / 读写分派 / 异常应答 / IAP 接数据
 */

#include "modbus/modbus_slave.h"

#if MB_SLAVE_EN

#if MB_FC01_EN || MB_FC02_EN || MB_FC05_EN || MB_FC0F_EN
/** @brief 位段是否已配置（data 或任一回调非空） */
static uint8_t bit_seg_valid(const struct mb_bit_seg *seg)
{
    return (uint8_t)(seg->data != 0 || seg->on_read != 0 || seg->on_write != 0);
}
#endif

/** @brief 寄存器段是否已配置（data 或任一回调非空） */
static uint8_t reg_seg_valid(const struct mb_reg_seg *seg)
{
    return (uint8_t)(seg->data != 0 || seg->on_read != 0 || seg->on_write != 0);
}

#if MB_FC01_EN || MB_FC02_EN
/** @brief 读位：回调优先，其次连续数组，否则报错 */
static enum mb_err_t bit_seg_read(const struct mb_bit_seg *seg, uint16_t local, uint8_t *out)
{
    if (seg->on_read)
        return seg->on_read((uint16_t)(seg->start_addr + local), out);
    if (seg->data)
    {
        *out = (uint8_t)(seg->data[local] & 0x01);
        return MB_OK;
    }
    return MB_ERR_ADDR;
}
#endif

#if MB_FC05_EN || MB_FC0F_EN
/** @brief 写位：回调优先，其次连续数组，否则报错 */
static enum mb_err_t bit_seg_write(const struct mb_bit_seg *seg, uint16_t local, uint8_t val)
{
    if (seg->on_write)
        return seg->on_write((uint16_t)(seg->start_addr + local), val);
    if (seg->data)
    {
        seg->data[local] = (uint8_t)(val ? 1 : 0);
        return MB_OK;
    }
    return MB_ERR_ADDR;
}
#endif

#if MB_FC03_EN || MB_FC04_EN
/** @brief 读寄存器：回调优先，其次连续数组，否则报错 */
static enum mb_err_t reg_seg_read(const struct mb_reg_seg *seg, uint16_t local, uint16_t *out)
{
    if (seg->on_read)
        return seg->on_read((uint16_t)(seg->start_addr + local), out);
    if (seg->data)
    {
        *out = seg->data[local];
        return MB_OK;
    }
    return MB_ERR_ADDR;
}
#endif

#if MB_FC06_EN || MB_FC10_EN
/** @brief 写寄存器：回调优先，其次连续数组，否则报错 */
static enum mb_err_t reg_seg_write(const struct mb_reg_seg *seg, uint16_t local, uint16_t val)
{
    if (seg->on_write)
        return seg->on_write((uint16_t)(seg->start_addr + local), val);
    if (seg->data)
    {
        seg->data[local] = val;
        return MB_OK;
    }
    return MB_ERR_ADDR;
}
#endif

#if MB_FC01_EN || MB_FC02_EN || MB_FC05_EN || MB_FC0F_EN
/** @brief 在位段数组中查找包含指定地址的段，返回局部下标 */
static const struct mb_bit_seg *bit_seg_find(const struct mb_bit_seg *segs, uint8_t seg_num,
                                             uint16_t modbus_addr, uint16_t *out_local_idx)
{
    uint8_t i;
    for (i = 0; i < seg_num; i++)
    {
        if (!bit_seg_valid(&segs[i]))
            continue;
        if (modbus_addr >= segs[i].start_addr &&
            modbus_addr < (uint16_t)(segs[i].start_addr + segs[i].num))
        {
            *out_local_idx = (uint16_t)(modbus_addr - segs[i].start_addr);
            return &segs[i];
        }
    }
    return 0;
}
#endif

/** @brief 在寄存器段数组中查找包含指定地址的段，返回局部下标 */
static const struct mb_reg_seg *reg_seg_find(const struct mb_reg_seg *segs, uint8_t seg_num,
                                             uint16_t modbus_addr, uint16_t *out_local_idx)
{
    uint8_t i;
    for (i = 0; i < seg_num; i++)
    {
        if (!reg_seg_valid(&segs[i]))
            continue;
        if (modbus_addr >= segs[i].start_addr &&
            modbus_addr < (uint16_t)(segs[i].start_addr + segs[i].num))
        {
            *out_local_idx = (uint16_t)(modbus_addr - segs[i].start_addr);
            return &segs[i];
        }
    }
    return 0;
}

#if MB_FC01_EN || MB_FC02_EN || MB_FC0F_EN
/** @brief 查找段并校验 [start, start+count) 不越段界 */
static const struct mb_bit_seg *bit_seg_check_range(const struct mb_bit_seg *segs, uint8_t seg_num,
                                                    uint16_t start, uint16_t count,
                                                    uint16_t *out_local_idx)
{
    const struct mb_bit_seg *seg = bit_seg_find(segs, seg_num, start, out_local_idx);
    if (seg == 0)
        return 0;
    if ((uint16_t)(*out_local_idx + count) > seg->num)
        return 0;
    return seg;
}
#endif

/** @brief 查找段并校验 [start, start+count) 不越段界 */
static const struct mb_reg_seg *reg_seg_check_range(const struct mb_reg_seg *segs, uint8_t seg_num,
                                                    uint16_t start, uint16_t count,
                                                    uint16_t *out_local_idx)
{
    const struct mb_reg_seg *seg = reg_seg_find(segs, seg_num, start, out_local_idx);
    if (seg == 0)
        return 0;
    if ((uint16_t)(*out_local_idx + count) > seg->num)
        return 0;
    return seg;
}

/** @brief 将回调错误码映射为 Modbus 异常码（地址错→0x02，其他→0x04） */
static uint8_t err_to_exception(enum mb_err_t e)
{
    return (uint8_t)((e == MB_ERR_ADDR) ? MB_EX_ILLEGAL_DATA_ADDRESS : MB_EX_SLAVE_DEVICE_FAILURE);
}

/** @brief 发送应答帧（广播地址 0x00 不应答，符合 Modbus 规范） */
static void slave_send(struct mb_slave_handle *self, uint8_t addr, const uint8_t *f, uint8_t len)
{
    if (addr == 0x00)
    {
        return;
    }
    if (self->base.send != NULL)
    {
        self->base.send(f, len, self->base.ctx);
    }
}

/** @brief 发送标准 Modbus 异常帧（addr, fc|0x80, ex_code, crc） */
static void send_exception(struct mb_slave_handle *self, uint8_t addr, uint8_t fc, uint8_t ex_code)
{
    uint8_t *f = self->base.tx_frame;
    f[0] = addr;
    f[1] = (uint8_t)(fc | 0x80);
    f[2] = ex_code;
    mb_append_crc(f, 3);
    slave_send(self, addr, f, 5);
}

#if MB_FC41_EN
/** @brief 发送 IAP 状态响应（status 位于数据区首字节） */
static void iap_send_status(struct mb_slave_handle *self, uint8_t addr, uint8_t cmd, uint8_t status)
{
    uint8_t *f = self->base.tx_frame;
    f[0] = addr;
    f[1] = MB_FC_IAP;
    f[2] = cmd;
    f[3] = status;
    mb_append_crc(f, 4);
    slave_send(self, addr, f, 6);
}

/** @brief 处理 IAP 子命令：START / DATA / END / STATUS */
static void iap_slave_process(struct mb_slave_handle *self, const uint8_t *req, uint8_t len)
{
    uint8_t addr = req[0];
    uint8_t cmd = req[2];
    struct mb_iap_slave *iap = &self->iap;

    switch (cmd)
    {
    case MB_IAP_CMD_START:
        if (len < 13) /* addr+fc+cmd+total_size(4)+fw_crc32(4)+crc(2) */
        {
            iap_send_status(self, addr, cmd, MB_IAP_BAD_LEN);
            return;
        }
        iap->total_size = ((uint32_t)req[3] << 24) | ((uint32_t)req[4] << 16) |
                          ((uint32_t)req[5] << 8) | (uint32_t)req[6];
        iap->expected_crc32 = ((uint32_t)req[7] << 24) | ((uint32_t)req[8] << 16) |
                              ((uint32_t)req[9] << 8) | (uint32_t)req[10];
        iap->next_block = 0;
        iap->active = 1;
        if (iap->on_start && iap->on_start(iap->total_size, iap->expected_crc32) != MB_OK)
        {
            iap->active = 0;
            iap_send_status(self, addr, cmd, MB_IAP_INTERNAL);
            return;
        }
        iap_send_status(self, addr, cmd, MB_IAP_OK);
        break;

    case MB_IAP_CMD_DATA:
    {
        uint16_t block_no;
        uint8_t data_len;
        if (!iap->active)
        {
            iap_send_status(self, addr, cmd, MB_IAP_NOT_ACTIVE);
            return;
        }
        if (len < 8) /* addr+fc+cmd+block_no(2)+至少 1 字节数据+crc(2) */
        {
            iap_send_status(self, addr, cmd, MB_IAP_BAD_LEN);
            return;
        }
        block_no = (uint16_t)(((uint16_t)req[3] << 8) | req[4]);
        data_len = (uint8_t)(len - 7); /* 总长 - (addr+fc+cmd+block_no2+crc2) */
        if (data_len > MB_IAP_MAX_DATA)
        {
            iap_send_status(self, addr, cmd, MB_IAP_BAD_LEN);
            return;
        }
        if (block_no != iap->next_block)
        {
            iap_send_status(self, addr, cmd, MB_IAP_BAD_BLOCK);
            return;
        }
        if (iap->on_data == NULL || iap->on_data(block_no, &req[5], data_len) != MB_OK)
        {
            iap_send_status(self, addr, cmd, MB_IAP_INTERNAL);
            return;
        }
        iap->next_block++;
        iap_send_status(self, addr, cmd, MB_IAP_OK);
        break;
    }

    case MB_IAP_CMD_END:
    {
        uint16_t total_blocks;
        if (!iap->active)
        {
            iap_send_status(self, addr, cmd, MB_IAP_NOT_ACTIVE);
            return;
        }
        if (len < 7) /* addr+fc+cmd+total_blocks(2)+crc(2) */
        {
            iap_send_status(self, addr, cmd, MB_IAP_BAD_LEN);
            return;
        }
        total_blocks = (uint16_t)(((uint16_t)req[3] << 8) | req[4]);
        if (total_blocks != iap->next_block)
        {
            iap_send_status(self, addr, cmd, MB_IAP_BAD_BLOCK);
            return;
        }
        iap->active = 0;
        if (iap->on_end && iap->on_end() != MB_OK)
        {
            iap_send_status(self, addr, cmd, MB_IAP_INTERNAL);
            return;
        }
        iap_send_status(self, addr, cmd, MB_IAP_OK);
        break;
    }

    case MB_IAP_CMD_STATUS:
    {
        uint8_t *f = self->base.tx_frame;
        f[0] = addr;
        f[1] = MB_FC_IAP;
        f[2] = cmd;
        f[3] = MB_IAP_OK;
        f[4] = (uint8_t)(iap->next_block >> 8);
        f[5] = (uint8_t)(iap->next_block & 0xFF);
        mb_append_crc(f, 6);
        slave_send(self, addr, f, 8);
        break;
    }

    default:
        iap_send_status(self, addr, cmd, MB_IAP_BAD_LEN);
        break;
    }
}
#endif /* MB_FC41_EN */

/** @brief 按功能码分派处理标准 Modbus 请求 */
static void slave_process(struct mb_slave_handle *self, const uint8_t *req, uint8_t len)
{
    uint8_t fc = req[1];
    uint16_t start = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
    uint16_t count = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);
    uint8_t *f = self->base.tx_frame;
    struct mb_reg_map *rm = &self->reg_map;
    uint8_t addr = req[0];
    uint16_t i;
    uint16_t local;
    uint8_t byte_cnt;
    enum mb_err_t e;
#if MB_FC01_EN || MB_FC02_EN || MB_FC05_EN || MB_FC0F_EN
    const struct mb_bit_seg *bseg;
#endif
    const struct mb_reg_seg *rseg;

    switch (fc)
    {
#if MB_FC01_EN
    case MB_FC_READ_COILS:
    {
        uint8_t bval;
        if (count == 0 || count > MB_MAX_READ_COILS)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        bseg = bit_seg_check_range(rm->coils, rm->coils_num, start, count, &local);
        if (!bseg)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_ADDRESS);
            return;
        }
        byte_cnt = (uint8_t)((count + 7) >> 3);
        f[0] = addr;
        f[1] = fc;
        f[2] = byte_cnt;
        for (i = 0; i < byte_cnt; i++)
            f[3 + i] = 0;
        for (i = 0; i < count; i++)
        {
            e = bit_seg_read(bseg, (uint16_t)(local + i), &bval);
            if (e != MB_OK)
            {
                send_exception(self, addr, fc, err_to_exception(e));
                return;
            }
            if (bval)
                f[3 + (i >> 3)] |= (uint8_t)(1 << (i & 0x07));
        }
        mb_append_crc(f, (uint8_t)(3 + byte_cnt));
        slave_send(self, addr, f, (uint8_t)(3 + byte_cnt + 2));
        break;
    }
#endif

#if MB_FC02_EN
    case MB_FC_READ_DISCRETE_INPUTS:
    {
        uint8_t bval;
        if (count == 0 || count > MB_MAX_READ_COILS)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        bseg = bit_seg_check_range(rm->discrete, rm->discrete_num, start, count, &local);
        if (!bseg)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_ADDRESS);
            return;
        }
        byte_cnt = (uint8_t)((count + 7) >> 3);
        f[0] = addr;
        f[1] = fc;
        f[2] = byte_cnt;
        for (i = 0; i < byte_cnt; i++)
            f[3 + i] = 0;
        for (i = 0; i < count; i++)
        {
            e = bit_seg_read(bseg, (uint16_t)(local + i), &bval);
            if (e != MB_OK)
            {
                send_exception(self, addr, fc, err_to_exception(e));
                return;
            }
            if (bval)
                f[3 + (i >> 3)] |= (uint8_t)(1 << (i & 0x07));
        }
        mb_append_crc(f, (uint8_t)(3 + byte_cnt));
        slave_send(self, addr, f, (uint8_t)(3 + byte_cnt + 2));
        break;
    }
#endif

#if MB_FC03_EN
    case MB_FC_READ_HOLDING_REGS:
    {
        uint16_t val;
        if (count == 0 || count > MB_MAX_READ_REGS)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        rseg = reg_seg_check_range(rm->holding, rm->holding_num, start, count, &local);
        if (!rseg)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_ADDRESS);
            return;
        }
        byte_cnt = (uint8_t)(count * 2);
        f[0] = addr;
        f[1] = fc;
        f[2] = byte_cnt;
        for (i = 0; i < count; i++)
        {
            e = reg_seg_read(rseg, (uint16_t)(local + i), &val);
            if (e != MB_OK)
            {
                send_exception(self, addr, fc, err_to_exception(e));
                return;
            }
            f[3 + i * 2] = (uint8_t)(val >> 8);
            f[3 + i * 2 + 1] = (uint8_t)(val & 0xFF);
        }
        mb_append_crc(f, (uint8_t)(3 + byte_cnt));
        slave_send(self, addr, f, (uint8_t)(3 + byte_cnt + 2));
        break;
    }
#endif

#if MB_FC04_EN
    case MB_FC_READ_INPUT_REGS:
    {
        uint16_t val;
        if (count == 0 || count > MB_MAX_READ_REGS)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        rseg = reg_seg_check_range(rm->input, rm->input_num, start, count, &local);
        if (!rseg)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_ADDRESS);
            return;
        }
        byte_cnt = (uint8_t)(count * 2);
        f[0] = addr;
        f[1] = fc;
        f[2] = byte_cnt;
        for (i = 0; i < count; i++)
        {
            e = reg_seg_read(rseg, (uint16_t)(local + i), &val);
            if (e != MB_OK)
            {
                send_exception(self, addr, fc, err_to_exception(e));
                return;
            }
            f[3 + i * 2] = (uint8_t)(val >> 8);
            f[3 + i * 2 + 1] = (uint8_t)(val & 0xFF);
        }
        mb_append_crc(f, (uint8_t)(3 + byte_cnt));
        slave_send(self, addr, f, (uint8_t)(3 + byte_cnt + 2));
        break;
    }
#endif

#if MB_FC05_EN
    case MB_FC_WRITE_SINGLE_COIL:
        if (count != 0x0000 && count != 0xFF00)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        bseg = bit_seg_find(rm->coils, rm->coils_num, start, &local);
        if (!bseg)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_ADDRESS);
            return;
        }
        e = bit_seg_write(bseg, local, (uint8_t)(count == 0xFF00 ? 1 : 0));
        if (e != MB_OK)
        {
            send_exception(self, addr, fc, err_to_exception(e));
            return;
        }
        f[0] = addr;
        f[1] = fc;
        f[2] = req[2];
        f[3] = req[3];
        f[4] = req[4];
        f[5] = req[5];
        mb_append_crc(f, 6);
        slave_send(self, addr, f, 8);
        break;
#endif

#if MB_FC06_EN
    case MB_FC_WRITE_SINGLE_REG:
        rseg = reg_seg_find(rm->holding, rm->holding_num, start, &local);
        if (!rseg)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_ADDRESS);
            return;
        }
        e = reg_seg_write(rseg, local, count);
        if (e != MB_OK)
        {
            send_exception(self, addr, fc, err_to_exception(e));
            return;
        }
        f[0] = addr;
        f[1] = fc;
        f[2] = req[2];
        f[3] = req[3];
        f[4] = req[4];
        f[5] = req[5];
        mb_append_crc(f, 6);
        slave_send(self, addr, f, 8);
        break;
#endif

#if MB_FC0F_EN
    case MB_FC_WRITE_MULTI_COILS:
    {
        uint8_t data_bytes = req[6];
        if (count == 0 || count > MB_MAX_WRITE_COILS)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        if (data_bytes != (uint8_t)((count + 7) >> 3))
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        if (len < (uint8_t)(7 + data_bytes + 2))
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        bseg = bit_seg_check_range(rm->coils, rm->coils_num, start, count, &local);
        if (!bseg)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_ADDRESS);
            return;
        }
        for (i = 0; i < count; i++)
        {
            uint8_t val = (uint8_t)((req[7 + (i >> 3)] >> (i & 0x07)) & 0x01);
            e = bit_seg_write(bseg, (uint16_t)(local + i), val);
            if (e != MB_OK)
            {
                send_exception(self, addr, fc, err_to_exception(e));
                return;
            }
        }
        f[0] = addr;
        f[1] = fc;
        f[2] = req[2];
        f[3] = req[3];
        f[4] = req[4];
        f[5] = req[5];
        mb_append_crc(f, 6);
        slave_send(self, addr, f, 8);
        break;
    }
#endif

#if MB_FC10_EN
    case MB_FC_WRITE_MULTI_REGS:
    {
        uint8_t data_bytes = req[6];
        if (count == 0 || count > MB_MAX_WRITE_REGS)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        if (data_bytes != (uint8_t)(count * 2))
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        if (len < (uint8_t)(7 + data_bytes + 2))
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }
        rseg = reg_seg_check_range(rm->holding, rm->holding_num, start, count, &local);
        if (!rseg)
        {
            send_exception(self, addr, fc, MB_EX_ILLEGAL_DATA_ADDRESS);
            return;
        }
        for (i = 0; i < count; i++)
        {
            uint16_t val = (uint16_t)(((uint16_t)req[7 + i * 2] << 8) | req[8 + i * 2]);
            e = reg_seg_write(rseg, (uint16_t)(local + i), val);
            if (e != MB_OK)
            {
                send_exception(self, addr, fc, err_to_exception(e));
                return;
            }
        }
        f[0] = addr;
        f[1] = fc;
        f[2] = req[2];
        f[3] = req[3];
        f[4] = req[4];
        f[5] = req[5];
        mb_append_crc(f, 6);
        slave_send(self, addr, f, 8);
        break;
    }
#endif

#if MB_FC41_EN
    case MB_FC_IAP:
        iap_slave_process(self, req, len);
        break;
#endif

    default:
        send_exception(self, addr, fc, MB_EX_ILLEGAL_FUNCTION);
        break;
    }
}

/**
 * @brief 处理一帧请求（验 CRC → 比对地址 → 分派处理；广播仅执行不应答）
 */
uint8_t mb_slave_rx_frame(struct mb_slave_handle *self, const uint8_t *p_data, uint8_t len)
{
    if (len < 5)
        return 0;

    if (!mb_check_crc(p_data, len))
        return 0;

    if (p_data[0] == self->slave_addr || p_data[0] == 0x00)
    {
        slave_process(self, p_data, len);
        return 1;
    }
    return 0;
}

/**
 * @brief 初始化从机（清零句柄后设置地址并拷贝寄存器映射表）
 */
void mb_slave_init(struct mb_slave_handle *self,
                   uint8_t addr,
                   const struct mb_reg_map *reg_map)
{
    if (self == NULL)
    {
        return;
    }

    memset(self, 0, sizeof(*self));
    self->slave_addr = addr;
    if (reg_map != NULL)
    {
        self->reg_map = *reg_map;
    }
}

#endif
