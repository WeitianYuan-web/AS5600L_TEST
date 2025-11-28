#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"

/*==================== 配置宏 ====================*/

#define I2C_PORT_NUM           0           /**< 使用 I2C_NUM_0 */
#define I2C_SDA_GPIO           4           /**< SDA = GPIO4 */
#define I2C_SCL_GPIO           5           /**< SCL = GPIO5 */
#define I2C_CLK_SPEED_HZ       400000      /**< I2C 400kHz */

/* AS5600L 相关 */
#define AS5600L_DEFAULT_ADDR   0x40        /**< AS5600L 默认 7 位 I2C 地址 */
#define AS5600L_REG_I2CADDR    0x20        /**< I2CADDR 寄存器地址 */
#define AS5600L_REG_BURN       0xFF        /**< BURN 寄存器地址 */
#define AS5600L_BURN_SETTING   0x40        /**< BURN_SETTING 命令值 */

/* 目标 I2C 地址，可按需修改 */
#define AS5600L_TARGET_ADDR    0x50

static const char *TAG = "AS5600L_ADDR";

#define I2C_PROBE_TIMEOUT_MS   50          /**< I2C 地址探测超时 */

/*==================== 全局 I2C 句柄 ====================*/

static i2c_master_bus_handle_t g_i2c_bus = NULL;

/*==================== I2C 基础函数 ====================*/

/**
 * @brief 初始化 I2C 主机总线
 *
 * @return
 *  - ESP_OK: 初始化成功
 *  - 其他:   I2C 驱动初始化失败
 */
static esp_err_t i2c_master_init(void)
{
    esp_err_t ret;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ret = i2c_new_master_bus(&bus_cfg, &g_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief 创建一个 I2C 设备句柄（对应一个 7 位从机地址）
 *
 * @param addr       7 位 I2C 地址
 * @param handle_out 输出设备句柄指针
 *
 * @return
 *  - ESP_OK: 创建成功
 *  - 其他:   失败
 */
static esp_err_t i2c_get_device(uint8_t addr, i2c_master_dev_handle_t *handle_out)
{
    if (g_i2c_bus == NULL) {
        return ESP_FAIL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_CLK_SPEED_HZ,
    };

    return i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, handle_out);
}

/*==================== AS5600L 寄存器读写封装 ====================*/

/**
 * @brief 写 AS5600L 单字节寄存器
 *
 * @param i2c_addr AS5600L 当前 7 位 I2C 地址
 * @param reg      寄存器地址
 * @param value    要写入的数值
 *
 * @return
 *  - ESP_OK: 写入成功
 *  - 其他:   写入失败
 */
static esp_err_t as5600l_write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t value)
{
    esp_err_t ret;
    i2c_master_dev_handle_t dev;

    ret = i2c_get_device(i2c_addr, &dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_get_device(0x%02X) failed: %s", i2c_addr, esp_err_to_name(ret));
        return ret;
    }

    uint8_t buf[2] = { reg, value };
    ret = i2c_master_transmit(dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }

    i2c_master_bus_rm_device(dev);
    return ret;
}

/**
 * @brief 读 AS5600L 单字节寄存器
 *
 * @param i2c_addr  AS5600L 当前 7 位 I2C 地址
 * @param reg       寄存器地址
 * @param value_out 输出读取的数值
 *
 * @return
 *  - ESP_OK: 读取成功
 *  - 其他:   读取失败
 */
static esp_err_t as5600l_read_reg(uint8_t i2c_addr, uint8_t reg, uint8_t *value_out)
{
    esp_err_t ret;
    i2c_master_dev_handle_t dev;

    ret = i2c_get_device(i2c_addr, &dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_get_device(0x%02X) failed: %s", i2c_addr, esp_err_to_name(ret));
        return ret;
    }

    /* 先写寄存器地址 */
    ret = i2c_master_transmit(dev, &reg, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write reg addr 0x%02X failed: %s", reg, esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev);
        return ret;
    }

    /* 再读 1 字节数据 */
    ret = i2c_master_receive(dev, value_out, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }

    i2c_master_bus_rm_device(dev);
    return ret;
}

/*==================== AS5600L 地址检测与修改 ====================*/

/**
 * @brief 使用 i2c_master_probe() 检测指定 I2C 地址上是否存在 AS5600L 设备
 *
 * @param addr 7 位 I2C 地址
 *
 * @return
 *  - ESP_OK: 该地址有设备响应
 *  - 其他:   无响应或通信出错
 */
static esp_err_t as5600l_check_present(uint8_t addr)
{
    if (g_i2c_bus == NULL) {
        return ESP_FAIL;
    }

    return i2c_master_probe(g_i2c_bus, addr, I2C_PROBE_TIMEOUT_MS);
}

/**
 * @brief 将 AS5600L 的 I2C 地址从 old_addr 永久修改为 new_addr
 *
 * 流程：
 *  1. 使用 old_addr 检测设备是否存在；
 *  2. 将 new_addr 写入 I2CADDR(0x20)；
 *  3. 向 BURN(0xFF) 写 0x40 触发 BURN_SETTING；
 *  4. 延时等待烧录完成；
 *  5. 可选：用 new_addr 再次访问寄存器进行验证。
 *
 * @warning BURN_SETTING 为一次性可编程操作，烧录次数有限，请谨慎多次执行。
 *
 * @param old_addr 旧的 7 位 I2C 地址（例如 0x40）
 * @param new_addr 期望的 7 位 I2C 地址（例如 0x50，内部会转换为 8 位写入 I2CADDR）
 *
 * @return
 *  - ESP_OK: 修改流程执行成功（不保证硬件一定成功烧录，请断电后用新地址验证）
 *  - 其他:   过程中出现错误
 */
static esp_err_t as5600l_change_address(uint8_t old_addr, uint8_t new_addr)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Check AS5600L at old addr 0x%02X ...", old_addr);
    ret = as5600l_check_present(old_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AS5600L not found at 0x%02X, err=%s", old_addr, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "AS5600L found at 0x%02X", old_addr);

    /* 1. 写 I2CADDR 寄存器
     * 根据 AS5600L 寄存器定义，I2CADDR 存放的是 8 位 I2C 地址（最低位为 0：写位），
     * 实际 7 位设备地址 = I2CADDR[7:1]。
     * 因此这里需要将 7 位地址左移 1 位，并清零最低位后写入寄存器。
     */
    uint8_t i2caddr_reg_val = (uint8_t)((new_addr << 1) & 0xFE);
    ESP_LOGI(TAG, "Write new I2C address 0x%02X (reg=0x%02X) to I2CADDR(0x20)",
             new_addr, i2caddr_reg_val);
    ret = as5600l_write_reg(old_addr, AS5600L_REG_I2CADDR, i2caddr_reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write I2CADDR failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. 执行 BURN_SETTING 命令 */
    ESP_LOGI(TAG, "Burn I2C address with BURN_SETTING (0x40) to BURN(0xFF)");
    ret = as5600l_write_reg(old_addr, AS5600L_REG_BURN, AS5600L_BURN_SETTING);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write BURN reg failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 3. 等待烧录完成（数据手册建议的时间，一般几毫秒，这里取 20ms 较为保险） */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 不能直接验证新地址，提醒用户断电重启后用新地址自行验证 */
    ESP_LOGI(TAG,
             "Address change procedure finished. 请断电重启后，用新地址 0x%02X 与 AS5600L 通信验证。",
             new_addr);
    return ESP_OK;
}

/**
 * @brief 扫描默认地址并在找到 AS5600L 时修改为目标地址
 *
 * @param target_addr 目标 7 位 I2C 地址
 *
 * @return
 *  - ESP_OK: 找到设备并成功执行修改流程
 *  - 其他:   未找到设备或修改失败
 */
static esp_err_t as5600l_scan_and_change(uint8_t target_addr)
{
    return as5600l_change_address(AS5600L_DEFAULT_ADDR, target_addr);
}

/*==================== I2C 总线扫描（类似 i2cdetect） ====================*/

/**
 * @brief 扫描当前 I2C 总线上的设备并打印表格
 *
 * @param found_as5600l_default 输出标志位，若在 0x40 发现设备则置为 true
 */
static void i2c_scan_bus(bool *found_as5600l_default)
{
    if (found_as5600l_default != NULL) {
        *found_as5600l_default = false;
    }

    if (g_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return;
    }

    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            uint8_t address = i + j;
            esp_err_t ret = i2c_master_probe(g_i2c_bus, address, I2C_PROBE_TIMEOUT_MS);
            if (ret == ESP_OK) {
                printf("%02x ", address);
                if (found_as5600l_default != NULL && address == AS5600L_DEFAULT_ADDR) {
                    *found_as5600l_default = true;
                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }
}

/*==================== 使用 esp_console + argtable3 的命令行交互 ====================*/

/**
 * @brief AS5600L 地址修改命令行参数结构
 *
 * 命令格式示例：
 *   as5600 -n 0x50        （第一步：只显示将要执行的操作，不真正烧录）
 *   as5600 -n 0x50 -y     （第二步：确认后执行永久烧录）
 */
static struct {
    struct arg_str *new_addr;   /**< 新地址参数 */
    struct arg_lit *yes;        /**< 确认标志 -y / --yes */
    struct arg_end *end;
} as5600_args;

/**
 * @brief 注册 as5600 命令
 */
static void register_as5600_cmd(void)
{
    as5600_args.new_addr = arg_str1("n", "new", "<addr>",
                                    "新的 7 位 I2C 地址（十六进制或十进制，如 0x50 或 80）");
    as5600_args.yes = arg_lit0("y", "yes", "确认执行永久烧录（BURN_SETTING，次数有限）");
    as5600_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "as5600",
        .help = "修改 AS5600L I2C 地址（两步确认，避免误烧录）",
        .hint = NULL,
        .func = NULL,      /* 先置空，下面再赋值 */
        .argtable = &as5600_args,
    };

    /* 由于 C 语言限制，这里先声明，再在后面实现真正的处理函数并注册 */
    extern int as5600_cmd_handler(int argc, char **argv);
    ((esp_console_cmd_t *)&cmd)->func = &as5600_cmd_handler;

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief as5600 控制台命令实现
 *
 * @param argc 参数个数
 * @param argv 参数数组
 *
 * @return
 *  - 0: 执行成功
 *  - 1: 解析或执行出错
 */
int as5600_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&as5600_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, as5600_args.end, argv[0]);
        return 1;
    }

    const char *addr_str = as5600_args.new_addr->sval[0];
    char *endptr = NULL;
    long val = strtol(addr_str, &endptr, 0);
    if (endptr == addr_str || val < 0 || val > 0x7F) {
        ESP_LOGE(TAG, "非法地址: %s（期望 0~0x7F）", addr_str);
        return 1;
    }

    uint8_t new_addr = (uint8_t)val;

    if (!as5600_args.yes->count) {
        /* 第一步：只提示，不真正烧录 */
        printf("\r\n即将把 AS5600L 地址从 0x%02X 修改为 0x%02X。\r\n", AS5600L_DEFAULT_ADDR, new_addr);
        printf("注意：此操作将执行 BURN_SETTING（一次性可编程，烧录次数有限且不可恢复）。\r\n");
        printf("如确认，请再次执行：as5600 -n 0x%02X -y\r\n\r\n", new_addr);
        return 0;
    }

    /* 第二步：带 -y 确认，真正执行烧录 */
    ESP_LOGI(TAG, "开始执行地址修改流程: 0x%02X -> 0x%02X", AS5600L_DEFAULT_ADDR, new_addr);
    esp_err_t ret = as5600l_change_address(AS5600L_DEFAULT_ADDR, new_addr);
    if (ret == ESP_OK) {
        printf("地址已尝试从 0x%02X 修改为 0x%02X。\r\n", AS5600L_DEFAULT_ADDR, new_addr);
        printf("请断电重启后，用新地址 0x%02X 与 AS5600L 通信以确认。\r\n", new_addr);
        return 0;
    }

    ESP_LOGE(TAG, "地址修改失败: %s", esp_err_to_name(ret));
    return 1;
}

/*==================== app_main 示例 ====================*/

/**
 * @brief 应用入口：初始化 I2C，总线扫描，并启动 ESP-IDF Console REPL
 *
 * 上电流程：
 *  1. 初始化 I2C 总线；
 *  2. 扫描 I2C 总线（使用 i2c_master_probe，类似 i2cdetect 输出）；
 *  3. 如果发现 0x40 设备，则提示用户在 REPL 中使用 `as5600` 命令进行地址修改；
 *  4. 启动 ESP-IDF Console REPL，等待用户输入命令。
 */
void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "AS5600L I2C address tool start");

    /* 1. 初始化 I2C 总线 */
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* 2. 扫描 I2C 总线 */
    bool found_default_addr = false;
    ESP_LOGI(TAG, "开始扫描 I2C 总线上的设备：");
    i2c_scan_bus(&found_default_addr);

    if (!found_default_addr) {
        ESP_LOGI(TAG, "未在 0x%02X 发现 AS5600L 默认地址设备。", AS5600L_DEFAULT_ADDR);
    } else {
        ESP_LOGI(TAG, "在 0x%02X 发现 AS5600L 设备。", AS5600L_DEFAULT_ADDR);
        ESP_LOGI(TAG, "如需修改地址，请稍后在控制台输入命令，例如：");
        ESP_LOGI(TAG, "  as5600 -n 0x50        （第一步：查看即将执行的操作）");
        ESP_LOGI(TAG, "  as5600 -n 0x50 -y     （第二步：确认并执行永久烧录）");
    }

    /* 3. 安装并启动 console REPL 环境 */
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "as5600>";

#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif

    /* 4. 注册 as5600 命令 */
    register_as5600_cmd();

    printf("\n ==============================================================\n");
    printf(" |             AS5600L 地址配置工具使用步骤                   |\n");
    printf(" |                                                            |\n");
    printf(" |  1. 查看上电日志，确认是否在 0x40 发现 AS5600L             |\n");
    printf(" |  2. 在提示符下输入: as5600 -n 0x50                         |\n");
    printf(" |  3. 再次确认后，输入: as5600 -n 0x50 -y                    |\n");
    printf(" |  4. 命令提示成功后，请断电重启，再用新地址通信            |\n");
    printf(" |                                                            |\n");
    printf(" ==============================================================\n\n");

    ESP_LOGI(TAG, "启动控制台 REPL，可输入 'help' 查看可用命令（如支持）。");

    /* 5. 启动 REPL（此调用不会返回） */
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
