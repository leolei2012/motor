/** @file modbus_master.c
 *  @brief Modbus RTU 主机实现：异步请求、超时处理、响应分派
 */

#include "modbus/modbus_master.h"
#include "modbus/modbus_iap.h"

#if MB_MASTER_EN

/**
 * @brief 初始化主机（memset 清零后设置发送回调）
 */
void mb_master_init(struct mb_master_handle *self, mb_send_func_t send, void *ctx)
{
    if (self == NULL)
    {
        return;
    }

    memset(self, 0, sizeof(*self));

    if (send != NULL)
    {
        self->base.send = send;
        self->base.ctx = ctx;
    }
}

/**
 * @brief 匹配并分派响应：地址相符则清 waiting，按功能码高位判断异常 / 正常回调
 */
static void master_process_rsp(struct mb_master_handle *self, const uint8_t *p_data, uint8_t len)
{
    struct mb_master_req *req = &self->req;

    if (!req->waiting)
        return;

    if (p_data[0] != req->slave_addr)
        return;

    /* 校验功能码：正常回显 req->fc，异常回 req->fc | 0x80 */
    if (p_data[1] != req->fc && p_data[1] != (uint8_t)(req->fc | 0x80))
        return;

    req->waiting = 0;
    req->timeout_cnt = 0;

    if (req->on_rsp)
    {
        if (p_data[1] & 0x80)
        {
            req->on_rsp(MB_ERR_FC, p_data, len, req->rsp_arg);
        }

        else
        {
            req->on_rsp(MB_OK, p_data, len, req->rsp_arg);
        }
    }
}

/**
 * @brief 处理收到的响应帧（验 CRC 后分派）
 */
void mb_master_rx_frame(struct mb_master_handle *self, const uint8_t *p_data, uint8_t len)
{
    if (len < 5)
        return;

    if (!mb_check_crc(p_data, len))
        return;

    master_process_rsp(self, p_data, len);
}

/**
 * @brief 周期调用，驱动请求超时
 */
void mb_master_tick(struct mb_master_handle *self)
{
    struct mb_master_req *req = &self->req;

    if (!req->waiting)
    {
        return;
    }

    req->timeout_cnt++;
    if (req->timeout_cnt >= req->timeout_reload)
    {
        req->waiting = 0;
        req->timeout_cnt = 0;
        if (req->on_rsp)
        {
            req->on_rsp(MB_ERR_TIMEOUT, 0, 0, req->rsp_arg);
        }
    }
}

/**
 * @brief 通用请求发送：记录请求状态、追加 CRC 并发帧
 */
static enum mb_err_t master_send(struct mb_master_handle *self,
                                 uint8_t slave_addr,
                                 uint8_t fc,
                                 uint16_t reg_addr,
                                 uint16_t reg_num,
                                 uint16_t timeout_tick,
                                 mb_master_rsp_cb_t on_rsp,
                                 void *rsp_arg,
                                 uint8_t frame_len)
{
    if (self->req.waiting)
        return MB_ERR_BUSY;

    self->req.slave_addr = slave_addr;
    self->req.fc = fc;
    self->req.reg_addr = reg_addr;
    self->req.reg_num = reg_num;
    self->req.timeout_reload = timeout_tick;
    self->req.timeout_cnt = 0;
    self->req.on_rsp = on_rsp;
    self->req.rsp_arg = rsp_arg;
    self->req.waiting = 1;

    mb_append_crc(self->base.tx_frame, frame_len);

    if (self->base.send != NULL)
        self->base.send(self->base.tx_frame, (uint8_t)(frame_len + 2), self->base.ctx);

    return MB_OK;
}

/**
 * @brief 发起读请求（FC01–FC04）
 */
enum mb_err_t mb_master_read(struct mb_master_handle *self,
                             uint8_t slave_addr,
                             uint8_t fc,
                             uint16_t reg_addr,
                             uint16_t reg_num,
                             uint16_t timeout_tick,
                             mb_master_rsp_cb_t on_rsp,
                             void *rsp_arg)
{
    uint8_t *f = self->base.tx_frame;
    f[0] = slave_addr;
    f[1] = fc;

    f[2] = (uint8_t)((reg_addr >> 8) & 0xFF);
    f[3] = (uint8_t)(reg_addr & 0xFF);
    f[4] = (uint8_t)((reg_num >> 8) & 0xFF);
    f[5] = (uint8_t)(reg_num & 0xFF);

    return master_send(self, slave_addr, fc, reg_addr, reg_num, timeout_tick, on_rsp, rsp_arg, 6);
}

/**
 * @brief 发起写单请求（FC05 / FC06）
 */
enum mb_err_t mb_master_write_single(struct mb_master_handle *self,
                                     uint8_t slave_addr,
                                     uint8_t fc,
                                     uint16_t addr,
                                     uint16_t value,
                                     uint16_t timeout_tick,
                                     mb_master_rsp_cb_t on_rsp,
                                     void *rsp_arg)
{
    uint8_t *f = self->base.tx_frame;
    f[0] = slave_addr;
    f[1] = fc;
    f[2] = (uint8_t)((addr >> 8) & 0xFF);
    f[3] = (uint8_t)(addr & 0xFF);
    f[4] = (uint8_t)((value >> 8) & 0xFF);
    f[5] = (uint8_t)(value & 0xFF);
    return master_send(self, slave_addr, fc, addr, 1, timeout_tick, on_rsp, rsp_arg, 6);
}

/**
 * @brief 发起写多请求（FC0F / FC10）
 */
enum mb_err_t mb_master_write_multi(struct mb_master_handle *self,
                                    uint8_t slave_addr,
                                    uint8_t fc,
                                    uint16_t start_addr,
                                    uint16_t num,
                                    const uint8_t *p_data,
                                    uint8_t data_len,
                                    uint16_t timeout_tick,
                                    mb_master_rsp_cb_t on_rsp,
                                    void *rsp_arg)
{
    uint8_t i;
    uint8_t *f = self->base.tx_frame;

    if ((uint16_t)(7 + data_len + 2) > MB_TX_FRAME_SIZE)
    {
        return MB_ERR_RANGE;
    }

    f[0] = slave_addr;
    f[1] = fc;
    f[2] = (uint8_t)(start_addr >> 8);
    f[3] = (uint8_t)(start_addr & 0xFF);
    f[4] = (uint8_t)(num >> 8);
    f[5] = (uint8_t)(num & 0xFF);
    f[6] = data_len;
    for (i = 0; i < data_len; i++)
        f[7 + i] = p_data[i];
    return master_send(self, slave_addr, fc, start_addr, num, timeout_tick, on_rsp, rsp_arg, (uint8_t)(7 + data_len));
}

#if MB_FC41_EN
/**
 * @brief IAP：开始升级（携带固件总大小与镜像 CRC32）
 */
enum mb_err_t mb_master_iap_start(struct mb_master_handle *self,
                                  uint8_t slave_addr,
                                  uint32_t total_size,
                                  uint32_t fw_crc32,
                                  uint16_t timeout_tick,
                                  mb_master_rsp_cb_t on_rsp,
                                  void *rsp_arg)
{
    uint8_t *f = self->base.tx_frame;
    f[0] = slave_addr;
    f[1] = MB_FC_IAP;
    f[2] = MB_IAP_CMD_START;
    f[3] = (uint8_t)(total_size >> 24);
    f[4] = (uint8_t)(total_size >> 16);
    f[5] = (uint8_t)(total_size >> 8);
    f[6] = (uint8_t)(total_size);
    f[7] = (uint8_t)(fw_crc32 >> 24);
    f[8] = (uint8_t)(fw_crc32 >> 16);
    f[9] = (uint8_t)(fw_crc32 >> 8);
    f[10] = (uint8_t)(fw_crc32);
    return master_send(self, slave_addr, MB_FC_IAP, 0, 0, timeout_tick, on_rsp, rsp_arg, 11);
}

/**
 * @brief IAP：写数据块
 */
enum mb_err_t mb_master_iap_write(struct mb_master_handle *self,
                                  uint8_t slave_addr,
                                  uint16_t block_no,
                                  const uint8_t *p_data,
                                  uint8_t data_len,
                                  uint16_t timeout_tick,
                                  mb_master_rsp_cb_t on_rsp,
                                  void *rsp_arg)
{
    uint8_t i;
    uint8_t *f = self->base.tx_frame;

    if (data_len > MB_IAP_MAX_DATA || (uint16_t)(5 + data_len + 2) > MB_TX_FRAME_SIZE)
    {
        return MB_ERR_RANGE;
    }

    f[0] = slave_addr;
    f[1] = MB_FC_IAP;
    f[2] = MB_IAP_CMD_DATA;
    f[3] = (uint8_t)(block_no >> 8);
    f[4] = (uint8_t)(block_no & 0xFF);
    for (i = 0; i < data_len; i++)
    {
        f[5 + i] = p_data[i];
    }
    return master_send(self, slave_addr, MB_FC_IAP, block_no, 0, timeout_tick, on_rsp, rsp_arg, (uint8_t)(5 + data_len));
}

/**
 * @brief IAP：结束升级（携带总块数）
 */
enum mb_err_t mb_master_iap_end(struct mb_master_handle *self,
                                uint8_t slave_addr,
                                uint16_t total_blocks,
                                uint16_t timeout_tick,
                                mb_master_rsp_cb_t on_rsp,
                                void *rsp_arg)
{
    uint8_t *f = self->base.tx_frame;
    f[0] = slave_addr;
    f[1] = MB_FC_IAP;
    f[2] = MB_IAP_CMD_END;
    f[3] = (uint8_t)(total_blocks >> 8);
    f[4] = (uint8_t)(total_blocks & 0xFF);
    return master_send(self, slave_addr, MB_FC_IAP, total_blocks, 0, timeout_tick, on_rsp, rsp_arg, 5);
}

/**
 * @brief IAP：查询状态（响应携带期望块号）
 */
enum mb_err_t mb_master_iap_status(struct mb_master_handle *self,
                                   uint8_t slave_addr,
                                   uint16_t timeout_tick,
                                   mb_master_rsp_cb_t on_rsp,
                                   void *rsp_arg)
{
    uint8_t *f = self->base.tx_frame;
    f[0] = slave_addr;
    f[1] = MB_FC_IAP;
    f[2] = MB_IAP_CMD_STATUS;
    return master_send(self, slave_addr, MB_FC_IAP, 0, 0, timeout_tick, on_rsp, rsp_arg, 3);
}
#endif /* MB_FC41_EN */

#endif
