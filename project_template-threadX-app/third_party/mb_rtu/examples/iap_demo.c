/** @file iap_demo.c
 *  @brief IAP 内存回环演示：验证正常流程与块号跳变负例
 */

#include <stdio.h>
#include <string.h>

#include "modbus/modbus_slave.h"
#include "modbus/modbus_master.h"

/* ---- 内存回环链路 ---- */
static uint8_t req_buf[256];
static uint8_t req_len;
static uint8_t rsp_buf[256];
static uint8_t rsp_len;
static uint8_t rsp_flag;

static uint8_t master_send_cb(const uint8_t *buf, uint8_t len, void *ctx)
{
    (void)ctx;
    memcpy(req_buf, buf, len);
    req_len = len;
    return 1;
}

static uint8_t slave_send(const uint8_t *buf, uint8_t len, void *ctx)
{
    (void)ctx;
    memcpy(rsp_buf, buf, len);
    rsp_len = len;
    rsp_flag = 1;
    return 1;
}

/* ---- 从机「接数据」状态（模拟 flash）---- */
static uint8_t fw_buf[512];
static uint32_t fw_received;
static uint8_t fw_done;

static enum mb_err_t iap_on_start(uint32_t total_size, uint32_t crc32)
{
    (void)total_size;
    (void)crc32;
    fw_received = 0;
    fw_done = 0;
    return MB_OK;
}

static enum mb_err_t iap_on_data(uint16_t block_no, const uint8_t *data, uint8_t len)
{
    uint16_t off = (uint16_t)(block_no * MB_IAP_MAX_DATA);
    memcpy(&fw_buf[off], data, len);
    fw_received += len;
    return MB_OK;
}

static enum mb_err_t iap_on_end(void)
{
    fw_done = 1;
    return MB_OK;
}

/* ---- 主机响应回调 ---- */
static uint8_t master_rsp_status;

static void on_rsp(enum mb_err_t err, const uint8_t *raw, uint8_t len, void *arg)
{
    (void)arg;
    if (err == MB_OK && len >= 4)
        master_rsp_status = raw[3]; /* status 在 raw[3]（raw[2] 为回显 cmd）*/
    else
        master_rsp_status = 0xEE;
}

/* 一次回环：把主机的请求喂给从机，再把从机的响应喂回主机 */
static void loop_once(struct mb_slave_handle *slave, struct mb_master_handle *master)
{
    if (req_len > 0)
    {
        mb_slave_rx_frame(slave, req_buf, req_len);
        req_len = 0;
    }
    if (rsp_flag)
    {
        mb_master_rx_frame(master, rsp_buf, rsp_len);
        rsp_flag = 0;
    }
}

/** @brief 内存回环测试：正常流程 + 块号跳变负例 */
int main(void)
{
    struct mb_slave_handle slave;
    struct mb_master_handle master;
    struct mb_reg_map map;
    static uint16_t holding[8];
    uint8_t fw[300];
    uint8_t junk[MB_IAP_MAX_DATA];
    uint16_t i;
    uint16_t total_blocks;

    memset(&slave, 0, sizeof(slave));
    memset(&master, 0, sizeof(master));
    memset(&map, 0, sizeof(map));
    memset(junk, 0, sizeof(junk));

    /* 从机 */
    map.holding[0].start_addr = 0;
    map.holding[0].num = 8;
    map.holding[0].data = holding;
    map.holding_num = 1;

    mb_slave_init(&slave, 1, &map);
    slave.base.send = slave_send;
    slave.iap.on_start = iap_on_start;
    slave.iap.on_data = iap_on_data;
    slave.iap.on_end = iap_on_end;

    /* 主机 */
    mb_master_init(&master, master_send_cb, NULL);

    /* 模拟一段固件字节流 */
    for (i = 0; i < sizeof(fw); i++)
    {
        fw[i] = (uint8_t)(i * 7 + 3);
    }
    total_blocks = (uint16_t)((sizeof(fw) + MB_IAP_MAX_DATA - 1) / MB_IAP_MAX_DATA);

    /* ---- 正常流程：START -> DATA x N -> END ---- */
    master_rsp_status = 0xFF;
    mb_master_iap_start(&master, 1, sizeof(fw), 0x12345678u, 100, on_rsp, NULL);
    loop_once(&slave, &master);
    if (master_rsp_status != MB_IAP_OK)
    {
        printf("FAIL: START status=0x%02X\n", master_rsp_status);
        return 1;
    }

    for (i = 0; i < total_blocks; i++)
    {
        uint16_t off = (uint16_t)(i * MB_IAP_MAX_DATA);
        uint8_t blen = (uint8_t)((off + MB_IAP_MAX_DATA <= sizeof(fw)) ? MB_IAP_MAX_DATA : (sizeof(fw) - off));
        master_rsp_status = 0xFF;
        mb_master_iap_write(&master, 1, i, &fw[off], blen, 100, on_rsp, NULL);
        loop_once(&slave, &master);
        if (master_rsp_status != MB_IAP_OK)
        {
            printf("FAIL: DATA block %d status=0x%02X\n", i, master_rsp_status);
            return 1;
        }
    }

    master_rsp_status = 0xFF;
    mb_master_iap_end(&master, 1, total_blocks, 100, on_rsp, NULL);
    loop_once(&slave, &master);
    if (master_rsp_status != MB_IAP_OK)
    {
        printf("FAIL: END status=0x%02X\n", master_rsp_status);
        return 1;
    }

    if (!fw_done || fw_received != sizeof(fw) || memcmp(fw_buf, fw, sizeof(fw)) != 0)
    {
        printf("FAIL: firmware mismatch (received=%lu)\n", (unsigned long)fw_received);
        return 1;
    }

    /* ---- 负例：块号跳变 -> BAD_BLOCK ---- */
    master_rsp_status = 0xFF;
    mb_master_iap_start(&master, 1, sizeof(fw), 0x12345678u, 100, on_rsp, NULL);
    loop_once(&slave, &master);

    master_rsp_status = 0xFF;
    mb_master_iap_write(&master, 1, 0, &fw[0], MB_IAP_MAX_DATA, 100, on_rsp, NULL);
    loop_once(&slave, &master);

    master_rsp_status = 0xFF;
    mb_master_iap_write(&master, 1, 2, junk, MB_IAP_MAX_DATA, 100, on_rsp, NULL);
    loop_once(&slave, &master);
    if (master_rsp_status != MB_IAP_BAD_BLOCK)
    {
        printf("FAIL: expected BAD_BLOCK, got 0x%02X\n", master_rsp_status);
        return 1;
    }

    printf("ALL TESTS PASSED (blocks=%d, block_size=%d)\n", total_blocks, MB_IAP_MAX_DATA);
    return 0;
}
