/*
 * Copyright (c) 2026 Area
 * SPDX-License-Identifier: MIT
 *
 * STM32 INMP441 硬件驱动模块头文件
 * 包含所有外设头文件引用、全局变量声明和硬件接口函数
 */

#ifndef __HARDWARE_H_
#define __HARDWARE_H_

// ==================== 标准库和HAL库头文件 ====================
// STM32F4系列HAL库主头文件
#include "stm32f4xx_hal.h"
// C标准输入输出库（用于printf调试）
#include <stdio.h>
// C标准字符串操作库
#include <string.h>
// CMSIS-DSP数学库（用于FFT和三角函数计算）
#include <arm_math.h>
// C标准库
#include <stdlib.h>
// 1.69寸SPI LCD显示屏驱动
#include "lcd_spi_169.h"
// C标准数学库
#include <math.h>
// STM32Cube生成的主头文件
#include "main.h"
// DMA外设配置头文件
#include "dma.h"
// I2S外设配置头文件
#include "i2s.h"
// 定时器配置头文件
#include "tim.h"
// 串口配置头文件
#include "usart.h"
// GPIO配置头文件
#include "gpio.h"
// FFT处理模块头文件
#include "fft.h"
// CMSIS-DSP常量结构体头文件
#include "arm_const_structs.h"

// ==================== 硬件配置宏 ====================
// I2S DMA单次接收缓冲区大小（单位：16位半字）
// 每次DMA传输4个字节（两个16位半字），对应一个24位音频样本
#define buffer_size 4

// ==================== 全局变量声明 ====================
// printf调试标志（保留未使用）
extern volatile uint8_t printf_flag;
// DMA传输完成计数器（调试用，统计I2S DMA中断次数）
extern volatile unsigned dma_cnt;
// 拼接并符号扩展后的32位音频样本
extern volatile int val32;
// I2S DMA原始接收缓冲区（16位格式）
extern volatile uint32_t audio_buf[buffer_size];
// DMA传输计数器（保留未使用）
extern volatile uint32_t dma_count;
// 串口接收缓冲区索引
extern uint16_t uart_rx_index;
// 串口接收数据缓冲区（最大256字节）
extern uint8_t uart_rx_buffer[256];
// 200-500Hz频段能量占比（应用层使用）
extern float target_ratio;
// 频谱峰值频率（保留未使用）
extern float freq;

// ==================== 对外接口函数 ====================
/**
 * @brief  串口接收超时处理函数（调试用）
 * @note   主循环中调用，实现串口数据按行接收和打印
 */
void uart_proc(void);

#endif