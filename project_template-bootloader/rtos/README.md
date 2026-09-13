# rtos/

本目录为 RTOS 占位目录。模板默认**裸机**运行，不绑定具体 RTOS。

如需引入 RTOS（RT-Thread / FreeRTOS / ThreadX …），把内核源码放到本目录，
并按所选 RTOS 的方式调整 `bsp/src/main.c` 的启动流程与各层初始化注册方式。

> 分层初始化约定见 `docs/规范/architecture.md` §5。
