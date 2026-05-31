/*
 * Copyright (c) 2026 Area
 * SPDX-License-Identifier: MIT
 *
 * STM32 INMP441 实时音频FFT处理模块头文件
 * 对外提供FFT初始化、处理和结果获取接口
 */

#ifndef __FFT_H_
#define __FFT_H_

#include "hardware.h"

// ==================== 系统配置参数 ====================
// FFT变换点数，必须为2的整数次幂
#define FFT_SIZE 1024
// I2S时钟配置的采样率（实际使用4000Hz，双声道采集仅取左声道）
#define SAMPLE_RATE 8000
// 目标检测频段的中心频率
#define TARGET_FREQ 500
// FFT输出的频率点数量 = FFT点数 / 2
#define FFT_OUTPUT_SIZE (FFT_SIZE/2)

// ==================== I2S中断共享全局变量 ====================
// 24位音频样本缓冲区（int32_t格式，已正确符号扩展）
// 由I2S中断写入，FFT处理函数读取
extern volatile int32_t fft_buf[FFT_SIZE];
// 缓冲区写入索引，由I2S中断递增，满1024后清零
extern volatile uint16_t fft_buf_idx;
// FFT计算就绪标志，缓冲区满时置1，处理完成后清0
extern volatile uint8_t fft_ready;

/**
 * @brief FFT计算结果结构体
 */
typedef struct{
    // 各频率点的幅值谱（归一化到[0.0, 1.0]）
	float magnitude[FFT_OUTPUT_SIZE];
    // 各频率点对应的实际频率值 (Hz)
    float frequency[FFT_OUTPUT_SIZE];
    // 频谱峰值对应的频率 (Hz)
    float peak_freq;
    // 频谱峰值的幅度
    float peak_magnitude;
    // 频谱峰值在magnitude数组中的索引
    uint32_t peak_index;
}FFT_RESULT_T;

// ==================== 对外接口函数 ====================
/**
 * @brief  初始化FFT处理模块
 * @note   系统启动时调用一次，完成所有预计算和变量初始化
 */
void fft_init(void);

/**
 * @brief  处理一帧完整的音频数据
 * @note   必须在fft_ready标志为1时调用，调用后自动清除标志
 */
void fft_process(void);

/**
 * @brief  获取FFT计算结果结构体指针
 * @return FFT_RESULT_T* 结果结构体指针
 */
FFT_RESULT_T* FFT_GetResult(void);

/**
 * @brief  计算200-500Hz频段的能量占比
 * @return float 占比百分比（0~100%）
 */
float FFT_calculateTargetRatio(void);

/**
 * @brief  计算标准A加权声压级
 * @return float 分贝值（35~120dB）
 */
float FFT_CalculateSPL(void);

#endif