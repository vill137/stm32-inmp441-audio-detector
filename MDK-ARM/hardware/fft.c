/*
 * Copyright (c) 2026 Area
 * SPDX-License-Identifier: MIT
 *
 * STM32 INMP441 实时音频FFT处理模块
 * 支持标准A加权分贝测量和200-500Hz频段能量占比检测
 */

#include "fft.h"

// CMSIS-DSP 快速实数FFT实例
static arm_rfft_fast_instance_f32 fft_instance;
// FFT输入缓冲区（时域信号，归一化到[-1.0, 1.0]）
static float fft_input[FFT_SIZE];
// FFT输出缓冲区（频域复数信号，长度为FFT_SIZE）
static float fft_output[FFT_SIZE];
// Hanning窗函数系数数组，用于减少频谱泄漏
static float hanning_window[FFT_SIZE];
// 时域信号原始RMS值（未加权）
static float rms_value = 0.0f;
// FFT计算结果结构体，包含幅值谱、频率和峰值信息
static FFT_RESULT_T fft_result;

// ==================== I2S中断共享缓冲区 ====================
// 24位音频样本缓冲区（int32_t格式，已正确符号扩展）
volatile int32_t fft_buf[FFT_SIZE];
// 缓冲区写入索引，由I2S中断递增
volatile uint16_t fft_buf_idx=0;
// FFT计算就绪标志，缓冲区满时置1，处理完成后清0
volatile uint8_t fft_ready=0;

// ==================== 后处理参数 ====================
// A加权系数数组（每个频率bin对应的能量域增益）
static float a_weighting[FFT_OUTPUT_SIZE];  // A加权系数数组
// 频段占比滑动平均缓存，用于平滑输出
static float last_ratio = 0.0f;             // ratio滑动平均
// 分贝值滑动平均缓存，用于平滑输出
static float last_spl = 35.0f;              // 分贝滑动平均
// A加权后的总能量，专门用于分贝计算
static float a_weighted_total_energy = 0.0f;

// 频率分辨率 = 采样率 / FFT点数 ≈ 3.90625Hz/bin
static const float freq_resolution=(float)SAMPLE_RATE/(float)FFT_SIZE;

/**
 * @brief  初始化FFT处理模块
 * @note   系统启动时调用一次，完成所有预计算和变量初始化
 */
void fft_init(void){
	// 初始化CMSIS-DSP RFFT实例
	arm_rfft_fast_init_f32(&fft_instance,FFT_SIZE);
	
	// 预计算Hanning窗函数系数
	for(uint32_t i=0;i<FFT_SIZE;i++){
		hanning_window[i]=0.5f*(1.0f-arm_cos_f32(2.0f*PI*(float)i/(float)(FFT_SIZE-1)));
	}
	
	// 预计算每个频率bin的A加权系数（ANSI S1.42标准）
	for(uint32_t i=0;i<FFT_OUTPUT_SIZE;i++){
		fft_result.frequency[i]=(float)i*freq_resolution;
		float f = fft_result.frequency[i];
		if(f < 20.0f) f = 20.0f; // 低于人耳可听下限按20Hz计算
		
		// A加权标准公式
		float f2 = f * f;
		float f4 = f2 * f2;
		float numerator = 12194.0f * 12194.0f * f4;
		float denominator = (f2 + 20.6f*20.6f) * (f2 + 12194.0f*12194.0f) * sqrtf((f2 + 107.7f*107.7f) * (f2 + 737.9f*737.9f));
		float a_db = 20.0f * log10f(numerator / denominator) + 2.0f;
		// 转换为能量域增益
		a_weighting[i] = powf(10.0f, a_db / 10.0f);
	}
	
	// 初始化结果数组
	memset(fft_result.magnitude,0,sizeof(fft_result.magnitude));
	fft_result.peak_freq=0.0f;
	fft_result.peak_index=0;
	fft_result.peak_magnitude=0.0f;
	
	// 初始化所有状态变量
	rms_value=0.0f;
	last_ratio=0.0f;
	last_spl=35.0f;
}

/**
 * @brief  处理一帧完整的音频数据
 * @note   必须在fft_ready标志为1时调用，调用后自动清除标志
 */
void fft_process(void){
	float dc_offset=0.0f;
	float sum_squares=0.0f;
	
	// 第一步：计算并去除直流偏置
	for(uint32_t i=0;i<FFT_SIZE;i++){
		dc_offset+=(float)fft_buf[i];
	}
	dc_offset/=(float)FFT_SIZE;
	
	// 第二步：去直流并计算时域均方值
	for(uint32_t i=0;i<FFT_SIZE;i++){
		float sample=(float)fft_buf[i]-dc_offset;
		sum_squares+=sample*sample;
		// 归一化到[-1.0, 1.0]范围（INMP441输出24位有符号整数）
		fft_input[i]=sample/8388608.0f;
	}
	
	// 计算时域RMS值
	rms_value=sqrtf(sum_squares/(float)FFT_SIZE);
	
	// 第三步：加Hanning窗减少频谱泄漏
	arm_mult_f32(fft_input,hanning_window,fft_input,FFT_SIZE);
	
	// 第四步：执行快速实数FFT变换
	arm_rfft_fast_f32(&fft_instance,fft_input,fft_output,0);
	
	// 第五步：计算幅值谱
	// DC分量（0Hz）
	fft_result.magnitude[0]=fabsf(fft_output[0])/(float)FFT_SIZE;
	// Nyquist分量（采样率/2 = 2000Hz）
	fft_result.magnitude[FFT_OUTPUT_SIZE-1]=fabsf(fft_output[1])/(float)FFT_SIZE;
	// 其他频率分量
	for(uint32_t i=1;i<FFT_OUTPUT_SIZE;i++){
		float real=fft_output[2*i];
		float imag=fft_output[2*i+1];
		fft_result.magnitude[i]=sqrtf(real*real+imag*imag)/(float)FFT_SIZE;
	}
	
	// 第六步：查找频谱峰值
	float max_mag=0.0f;
	uint32_t max_index=1;
	for(uint32_t i=1;i<FFT_OUTPUT_SIZE;i++){
		if(fft_result.magnitude[i]>max_mag){
			max_mag=fft_result.magnitude[i];
			max_index=i;
		}
	}
	fft_result.peak_index=max_index;
	fft_result.peak_magnitude=max_mag;
	fft_result.peak_freq=(float)max_index*4000.0f/(float)FFT_SIZE;
	
	// 第七步：计算A加权总能量（用于分贝计算）
	a_weighted_total_energy = 0.0f;
	for(int i=1; i<FFT_OUTPUT_SIZE-1; i++){
		float power = fft_result.magnitude[i] * fft_result.magnitude[i];
		a_weighted_total_energy += power * a_weighting[i];
	}
	a_weighted_total_energy *= 2.0f; // Hanning窗能量补偿
}

/**
 * @brief  获取FFT计算结果结构体指针
 * @return FFT_RESULT_T* 结果结构体指针
 */
FFT_RESULT_T* FFT_GetResult(void){
	return &fft_result;
}

/**
 * @brief  计算200-500Hz频段的能量占比
 * @return float 占比百分比（0~100%）
 */
float FFT_calculateTargetRatio(void){
	float total_energy=0.0f;
	float target_energy=0.0f;
	//BIN计算公式BIN=TARGET/3.90625
	// 目标范围：199Hz~512Hz（bin50~bin131）
	const int target_bin_start = 50;
	const int target_bin_end = 131;
	
	// 计算全频段总能量和目标频段能量
	for(int i=1;i<FFT_OUTPUT_SIZE-1;i++){
		float power = fft_result.magnitude[i] * fft_result.magnitude[i];
		total_energy += power;
		
		if(i >= target_bin_start && i <= target_bin_end){
			target_energy += power;
		}
	}
	
	// Hanning窗能量补偿
	total_energy *= 2.0f;
	target_energy *= 2.0f;
	
	// 无有效信号时平滑衰减到0
	if(total_energy < 1e-8f){
		last_ratio = last_ratio * 0.8f + 0.0f * 0.2f;
		return last_ratio;
	}
	
	// 计算当前占比
	float current_ratio = (target_energy / total_energy) * 100.0f;
	
	// 一阶滑动平均滤波：新值占60%，旧值占40%
	last_ratio = last_ratio * 0.4f + current_ratio * 0.6f;
	
	// 限制合理范围
	if(last_ratio < 0.01f) last_ratio = 0.01f;
	if(last_ratio > 100.0f) last_ratio = 100.0f;
	
	return last_ratio;
}

/**
 * @brief  计算标准A加权声压级
 * @return float 分贝值（35~120dB）
 */
float FFT_CalculateSPL(void){
    // INMP441官方参考值：94dB SPL @ 1kHz时的RMS输出
    const float ref_rms = 297285.0f;
	//会有10-5db的误差，可以通过偏移调节，不要调太多
    const float calibration_offset =-2.0f; 
	
	// 无有效信号时平滑衰减到35dB（系统本底噪声）
    if(a_weighted_total_energy < 1e-8f){
        last_spl = last_spl * 0.8f + 35.0f * 0.2f;
        return last_spl;
    }
    
    // 由频域A加权能量反推时域RMS值
    float signal_rms = sqrtf(a_weighted_total_energy) * 8388608.0f;
    
    // 声压级标准计算公式
    float raw_spl = 20.0f * log10f(signal_rms / ref_rms) + 94.0f + calibration_offset;
    
	// 一阶滑动平均滤波：新值占50%，旧值占50%
    last_spl = last_spl * 0.5f + raw_spl * 0.5f;
    
	// 限制合理范围
    if(last_spl < 35.0f) last_spl = 35.0f;
    if(last_spl > 120.0f) last_spl = 120.0f;
    
    return last_spl;
}