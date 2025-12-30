E (6094) task_wdt: Task watchdog got triggered. The following tasks/users did not reset the watchdog in time:
E (6094) task_wdt:  - IDLE0 (CPU 0)
E (6094) task_wdt: Tasks currently running:
E (6094) task_wdt: CPU 0: lv_tick
E (6094) task_wdt: CPU 1: IDLE1
E (6094) task_wdt: Print CPU 0 (current core) backtrace


Backtrace: 0x42030282:0x3FC976C0 0x42030698:0x3FC976E0 0x40376F2D:0x3FC97710 0x4037E84A:0x3FCEB460 0x4200A441:0x3FCEB480 0x4037CC99:0x3FCEB4A0
--- 0x42030282: task_wdt_timeout_handling at /home/denxwan/esp/v5.5.2/esp-idf/components/esp_system/task_wdt/task_wdt.c:436
--- 0x42030698: task_wdt_isr at /home/denxwan/esp/v5.5.2/esp-idf/components/esp_system/task_wdt/task_wdt.c:509
--- 0x40376f2d: _xt_lowint1 at /home/denxwan/esp/v5.5.2/esp-idf/components/xtensa/xtensa_vectors.S:1240
--- 0x4037e84a: vTaskDelay at /home/denxwan/esp/v5.5.2/esp-idf/components/freertos/FreeRTOS-Kernel/tasks.c:1611
--- 0x4200a441: lv_tick_task at /home/denxwan/Downloads/2.1inch_RGB_LCD_pic/main/main.c:153
--- 0x4037cc99: vPortTaskWrapper at /home/denxwan/esp/v5.5.2/esp-idf/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:139
