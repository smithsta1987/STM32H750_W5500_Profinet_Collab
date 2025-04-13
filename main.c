/** Full Working Version 1.0
  ******************************************************************************
  * @file           : main.c
  * @brief          : Program with IP config, PLC TCP server, LED, and DAC
  ******************************************************************************
  * Copyright (c) 2025 Orion MIS. All rights reserved.
  * Licensed under ST terms—see LICENSE file. Provided AS-IS if no LICENSE.
  ******************************************************************************
  */
#include "main.h"
#include "wizchip_conf.h"
#include <string.h>
#include <stdio.h>
#include "w5500/socket.h"
#include <stdlib.h>

/* Private defines */
#define W5500_CS_PIN GPIO_PIN_0
#define W5500_CS_PORT GPIOA
#define W5500_RST_PIN GPIO_PIN_0
#define W5500_RST_PORT GPIOB
#define USER_LED_PIN GPIO_PIN_3
#define USER_LED_PORT GPIOE
#define CONFIG_MAGIC 0xDEADBEEF
#define FLASH_CONFIG_ADDR 0x0800C000  // Sector 6
#define FLASH_SECTOR_SIZE 0x20000     // 128 KB
#define ENTRY_SIZE 32                 // sizeof(StoredConfig), aligned
#define MAX_ENTRIES (FLASH_SECTOR_SIZE / ENTRY_SIZE)  // ~4096
#define HTTP_PORT 80                  // HTTP server port
#define PLC_PORT 5000                 // PLC TCP server port
#define HTTP_SOCKET 1                 // HTTP on Socket 1
#define PLC_SOCKET 0                  // PLC on Socket 0
#define CONNECTION_TIMEOUT_MS 3000    // PLC timeout

/* Private typedefs */
typedef struct {
    uint32_t magic;         // 0xDEADBEEF
    uint32_t counter;       // Increment per update
    wiz_NetInfo netInfo;    // 24 bytes
    uint8_t padding[4];     // 4 bytes to make total 32 bytes
} StoredConfig;

/* Private variables */
SPI_HandleTypeDef hspi1;
DAC_HandleTypeDef hdac1;
HAL_StatusTypeDef debug_status;  // Flash debug
uint8_t debug_socket_state = 0;  // Socket debug
uint8_t debug_w5500_version = 0; // W5500 version debug
uint8_t debug_w5500_chip_id = 0; // W5500 chip ID debug
uint8_t flash_debug_code = 0;    // Flash failure debug
uint8_t plc_buffer[1024];        // PLC input buffer
uint8_t plc_input_index = 0;     // PLC buffer index
uint8_t plc_last_was_cr = 0;     // PLC carriage return flag
uint8_t plc_new_input = 1;       // PLC new input flag
uint8_t plc_was_connected = 0;   // PLC connection state
uint8_t plc_is_connected = 0;    // PLC current state
uint32_t plc_last_data_time = 0; // PLC timeout tracker

/* Private function prototypes */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_DAC1_Init(void);
void Error_Handler(void);
void wizchip_select(void);
void wizchip_deselect(void);
uint8_t wizchip_read(void);
void wizchip_write(uint8_t wb);
void wizchip_spi_init(void);
void wizchip_hardware_reset(void);
HAL_StatusTypeDef save_network_config(wiz_NetInfo* config);
void load_network_config(wiz_NetInfo* config);
uint8_t check_w5500(void);
void http_server(wiz_NetInfo* netInfo);
void plc_server(void);

/* SPI Functions */
void wizchip_select(void) { HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET); }
void wizchip_deselect(void) { HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET); }
uint8_t wizchip_read(void) { uint8_t rb; HAL_SPI_Receive(&hspi1, &rb, 1, 1000); return rb; }
void wizchip_write(uint8_t wb) { HAL_SPI_Transmit(&hspi1, &wb, 1, 1000); }
void wizchip_spi_init(void) { wizchip_deselect(); reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect); reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write); }
void wizchip_hardware_reset(void) { HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_RESET); HAL_Delay(10); HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET); HAL_Delay(200); }

/* W5500 Check Function */
uint8_t check_w5500(void) {
    uint8_t chip_id = WIZCHIP_READ(0x0000);  // Chip ID
    debug_w5500_chip_id = chip_id;
    uint8_t version = WIZCHIP_READ(0x0039);  // Version
    debug_w5500_version = version;
    return (version == 0x04);  // W5500 version should be 0x04
}

/* Flash Functions */
HAL_StatusTypeDef save_network_config(wiz_NetInfo* config) {
    StoredConfig storedConfig __attribute__((aligned(32)));
    uint32_t last_counter = 0;
    uint32_t next_offset = 0;

    // Find last valid entry
    for (uint32_t i = 0; i < MAX_ENTRIES; i++) {
        StoredConfig temp;
        uint32_t addr = FLASH_CONFIG_ADDR + (i * ENTRY_SIZE);
        memcpy(&temp, (void*)addr, ENTRY_SIZE);
        if (temp.magic != CONFIG_MAGIC) {
            next_offset = i * ENTRY_SIZE;
            break;
        }
        last_counter = temp.counter;
        if (i == MAX_ENTRIES - 1) {
            debug_status = HAL_FLASH_Unlock();
            if (debug_status != HAL_OK) return HAL_ERROR;

            while (FLASH->SR1 & FLASH_FLAG_BSY) {}
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);

            FLASH_EraseInitTypeDef eraseInit = {
                .TypeErase = FLASH_TYPEERASE_SECTORS,
                .Banks = FLASH_BANK_1,
                .Sector = 6,
                .NbSectors = 1
            };
            uint32_t sectorError = 0;
            debug_status = HAL_FLASHEx_Erase(&eraseInit, sectorError);
            if (debug_status != HAL_OK) {
                HAL_GPIO_TogglePin(USER_LED_PORT, USER_LED_PIN);
                HAL_Delay(50);
                HAL_FLASH_Lock();
                return HAL_ERROR;
            }
            next_offset = 0;
            last_counter = 0;
        }
    }

    storedConfig.magic = CONFIG_MAGIC;
    storedConfig.counter = last_counter + 1;
    storedConfig.netInfo = *config;

    debug_status = HAL_FLASH_Unlock();
    if (debug_status != HAL_OK) return HAL_ERROR;

    while (FLASH->SR1 & FLASH_FLAG_BSY) {}
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);

    uint32_t address = FLASH_CONFIG_ADDR + next_offset;
    uint64_t data[4];
    memcpy(data, &storedConfig, ENTRY_SIZE);
    debug_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)&data);
    if (debug_status != HAL_OK) {
        HAL_GPIO_TogglePin(USER_LED_PORT, USER_LED_PIN);
        HAL_Delay(50);
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    while (FLASH->SR1 & FLASH_FLAG_BSY) {}
    debug_status = HAL_FLASH_Lock();
    if (debug_status != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

void load_network_config(wiz_NetInfo* config) {
    StoredConfig storedConfig;
    uint32_t highest_counter = 0;
    uint32_t last_valid_index = 0;
    uint8_t found_valid = 0;

    for (uint32_t i = 0; i < MAX_ENTRIES; i++) {
        uint32_t addr = FLASH_CONFIG_ADDR + (i * ENTRY_SIZE);
        memcpy(&storedConfig, (void*)addr, ENTRY_SIZE);
        if (storedConfig.magic == CONFIG_MAGIC) {
            if (storedConfig.counter > highest_counter) {
                highest_counter = storedConfig.counter;
                last_valid_index = i;
                found_valid = 1;
            }
        } else {
            break;
        }
    }

    if (found_valid) {
        uint32_t addr = FLASH_CONFIG_ADDR + (last_valid_index * ENTRY_SIZE);
        memcpy(&storedConfig, (void*)addr, ENTRY_SIZE);
        *config = storedConfig.netInfo;
    }
}

/* HTTP Server */
const char* html_page =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Connection: close\r\n"
    "\r\n"
    "<!DOCTYPE html>"
    "<html>"
    "<head><title>Orion Controller IP Configuration</title></head>"
    "<body>"
    "<h1>Orion Controller IP Configuration</h1>"
    "<p>Current IP: %d.%d.%d.%d</p>"
    "<form method='POST' action='/update'>"
    "<label>New IP Address: </label>"
    "<input type='text' name='ip1' size='3' value='%d'>."
    "<input type='text' name='ip2' size='3' value='%d'>."
    "<input type='text' name='ip3' size='3' value='%d'>."
    "<input type='text' name='ip4' size='3' value='%d'> "
    "<input type='submit' value='Update IP'>"
    "</form>"
    "</body>"
    "</html>";

void http_server(wiz_NetInfo* netInfo) {
    uint8_t socket_state;
    uint16_t len;
    uint8_t buf[1024];

    socket_state = getSn_SR(HTTP_SOCKET);
    debug_socket_state = socket_state;

    if (socket_state == SOCK_CLOSED || socket_state == SOCK_CLOSE_WAIT || socket_state == SOCK_INIT) {
        close(HTTP_SOCKET);
        HAL_Delay(1);
        socket(HTTP_SOCKET, Sn_MR_TCP, HTTP_PORT, 0);
        HAL_Delay(1);
        listen(HTTP_SOCKET);
        HAL_Delay(1);
    }
    else if (socket_state == SOCK_ESTABLISHED) {
        len = getSn_RX_RSR(HTTP_SOCKET);
        if (len > 0) {
            if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
            recv(HTTP_SOCKET, buf, len);
            HAL_Delay(1);
            buf[len] = '\0';

            if (strncmp((char*)buf, "POST /update", 12) == 0) {
                char* ip1_str = strstr((char*)buf, "ip1=");
                char* ip2_str = strstr((char*)buf, "ip2=");
                char* ip3_str = strstr((char*)buf, "ip3=");
                char* ip4_str = strstr((char*)buf, "ip4=");
                if (ip1_str && ip2_str && ip3_str && ip4_str) {
                    uint8_t new_ip[4];
                    new_ip[0] = atoi(ip1_str + 4);
                    new_ip[1] = atoi(ip2_str + 4);
                    new_ip[2] = atoi(ip3_str + 4);
                    new_ip[3] = atoi(ip4_str + 4);

                    netInfo->ip[0] = new_ip[0];
                    netInfo->ip[1] = new_ip[1];
                    netInfo->ip[2] = new_ip[2];
                    netInfo->ip[3] = new_ip[3];

                    char error_page[512];
                    snprintf(error_page, sizeof(error_page),
                        "HTTP/1.1 500 Internal Server Error\r\n"
                        "Content-Type: text/html\r\n"
                        "Connection: close\r\n"
                        "\r\n"
                        "<!DOCTYPE html>"
                        "<html>"
                        "<head><title>Error</title></head>"
                        "<body>"
                        "<h1>Error</h1>"
                        "<p>Failed to save new IP address. Please try again.</p>"
                        "<p>Debug code: %d</p>"
                        "<a href='/'>Go back</a>"
                        "</body>"
                        "</html>",
                        flash_debug_code);

                    if (save_network_config(netInfo) == HAL_OK) {
                        close(HTTP_SOCKET);
                        HAL_Delay(100);
                        HAL_Delay(500);
                        NVIC_SystemReset();
                    } else {
                        send(HTTP_SOCKET, (uint8_t*)error_page, strlen(error_page));
                        close(HTTP_SOCKET);
                        HAL_Delay(10);
                    }
                }
            }

            int len = snprintf((char*)buf, sizeof(buf), html_page,
                               netInfo->ip[0], netInfo->ip[1], netInfo->ip[2], netInfo->ip[3],
                               netInfo->ip[0], netInfo->ip[1], netInfo->ip[2], netInfo->ip[3]);
            if (len >= sizeof(buf) - 1) {
                const char* error_msg = "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\n\r\nBuffer overflow";
                send(HTTP_SOCKET, (uint8_t*)error_msg, strlen(error_msg));
            } else {
                send(HTTP_SOCKET, buf, strlen((char*)buf));
            }
            close(HTTP_SOCKET);
            HAL_Delay(10);
        }
    }
}

/* PLC Server */
void plc_server(void) {
    uint8_t status = getSn_SR(PLC_SOCKET);
    if (status == SOCK_CLOSED || status == SOCK_CLOSE_WAIT || status == SOCK_INIT) {
        close(PLC_SOCKET);
        socket(PLC_SOCKET, Sn_MR_TCP, PLC_PORT, 0);
        listen(PLC_SOCKET);
        HAL_Delay(100);
    }
    else if (status == SOCK_ESTABLISHED) {
        if (!plc_was_connected) {
            uint8_t response[] = "Connected\r\n";
            send(PLC_SOCKET, response, strlen((char*)response));
            plc_was_connected = 1;
            plc_is_connected = 1;
            plc_last_data_time = HAL_GetTick();
        }
        int len = getSn_RX_RSR(PLC_SOCKET);
        while (len > 0) {
            uint8_t received_byte;
            len = recv(PLC_SOCKET, &received_byte, 1);
            if (len > 0) {
                plc_last_data_time = HAL_GetTick();
                if (plc_new_input) {
                    plc_input_index = 0;
                    memset(plc_buffer, 0, sizeof(plc_buffer));
                    plc_new_input = 0;
                }
                if (received_byte == 13) {
                    plc_last_was_cr = 1;
                    if (plc_input_index > 0) {
                        plc_buffer[plc_input_index] = '\0';
                        int value = atoi((char*)plc_buffer);
                        if (value < 0) value = 0;
                        if (value > 255) value = 255;
                        uint32_t dac_value = (value * 4095) / 255; // Scale 0-255 to 0-3.3V
                        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
                        uint8_t echo_response[50];
                        snprintf((char*)echo_response, sizeof(echo_response), "Set value: %d\r\n", value);
                        send(PLC_SOCKET, echo_response, strlen((char*)echo_response));
                    }
                    plc_input_index = 0;
                    plc_new_input = 1;
                    break;
                }
                else if (received_byte >= '0' && received_byte <= '9') {
                    if (plc_input_index < sizeof(plc_buffer) - 1) {
                        plc_buffer[plc_input_index++] = received_byte;
                    }
                }
                else {
                    plc_last_was_cr = 0;
                }
            }
            len = getSn_RX_RSR(PLC_SOCKET);
        }
    }
    if (plc_was_connected && (status != SOCK_ESTABLISHED || HAL_GetTick() - plc_last_data_time > CONNECTION_TIMEOUT_MS)) {
        close(PLC_SOCKET);
        plc_was_connected = 0;
        plc_is_connected = 0;
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); // Reset DAC to 0V
    }
}

/* Main */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    __HAL_RCC_SPI1_CLK_ENABLE();
    MX_SPI1_Init();
    MX_DAC1_Init();

    // Default network config
    wiz_NetInfo netInfo = {
        .mac = {0x00, 0x08, 0xDC, 0xAB, 0xCD, 0xEF},
        .ip = {192, 168, 0, 100},
        .sn = {255, 255, 255, 0},
        .gw = {192, 168, 0, 1},
        .dns = {8, 8, 8, 8},
        .dhcp = NETINFO_STATIC
    };

    // Initialize W5500
    wizchip_hardware_reset();
    HAL_Delay(50);
    wizchip_spi_init();
    HAL_Delay(50);
    uint8_t txsize[] = {2, 2, 0, 0, 0, 0, 0, 0}; // 2KB for Sockets 0, 1
    uint8_t rxsize[] = {2, 2, 0, 0, 0, 0, 0, 0};
    wizchip_init(txsize, rxsize);
    HAL_Delay(50);

    // Load config from flash
    load_network_config(&netInfo);
    wizchip_setnetinfo(&netInfo);
    HAL_Delay(50);

    if (!check_w5500()) {
        // W5500 not responding, but proceed for now
    }

    while (1) {
        plc_server();
        http_server(&netInfo);
        HAL_GPIO_TogglePin(USER_LED_PORT, USER_LED_PIN); // Blink LED
        HAL_Delay(500);
    }
}

/* System Config */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 16;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET);   // CS
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET); // RST
    HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_RESET); // LED
    GPIO_InitStruct.Pin = W5500_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(W5500_CS_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = W5500_RST_PIN;
    HAL_GPIO_Init(W5500_RST_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = USER_LED_PIN;
    HAL_GPIO_Init(USER_LED_PORT, &GPIO_InitStruct);
}

void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 0x0;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

void MX_DAC1_Init(void) {
    DAC_ChannelConfTypeDef sConfig = {0};
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) Error_Handler();
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
}

void Error_Handler(void) {
    while (1) {
        HAL_GPIO_TogglePin(USER_LED_PORT, USER_LED_PIN);
        HAL_Delay(50);
    }
}

