/*
 * Copyright (c) 2026 Area
 * SPDX-License-Identifier: MIT
 *
 * STM32 INMP441 硬件驱动模块
 * 包含I2S音频采集、串口调试和定时器中断处理
 */

#include "hardware.h"

/**
 * @brief  串口接收超时处理函数（调试用）
 * @note   主循环中调用，实现串口数据的按行接收和打印
 *         超时时间：50ms（TIM4时钟1MHz，计数到50000）
 */
//串口，调试用
void uart_proc(void){
	// 无数据接收时直接返回
	if(uart_rx_index==0){
		return ;
	}
	// 超过50ms没有新数据，认为一帧接收完成
	if(__HAL_TIM_GET_COUNTER(&htim4)>=50000){
		// 打印接收到的完整数据
		printf("%s\r\n",uart_rx_buffer);
		// 清空缓冲区和索引
		uart_rx_index=0;
		memset(uart_rx_buffer,0,sizeof(uart_rx_buffer));
		// 重置HAL库接收指针，准备下一次接收
		huart1.pRxBuffPtr=uart_rx_buffer;
	}
}

/**
 * @brief  I2S接收完成中断回调函数
 * @param  hi2s: I2S句柄
 * @note   1. STM32F4的I2S外设只能以16位格式接收24位数据
 *         2. 24位数据会被拆分成两个16位半字传输
 *         3. 必须手动拼接并正确进行符号扩展
 */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if(hi2s->Instance==SPI2)
    {
			// DMA传输完成计数器（调试用）
			dma_cnt++;
			// 拼接24位数据：高16位在audio_buf[0]，低8位在audio_buf[1]的高8位
			uint32_t val24=(audio_buf[0]<<8)+(audio_buf[1]>>8);
			// 正确的符号扩展：如果是负数（最高位为1），高8位补1
			if(val24&0x800000){
				val32=0xFF000000|val24;
			}
			else{
				val32=val24;
			}
			// 将处理好的32位样本存入FFT缓冲区
			fft_buf[fft_buf_idx++]=val32;
			// 缓冲区满时，置位FFT就绪标志，准备下一帧采集
			if(fft_buf_idx>=FFT_SIZE){
				fft_buf_idx=0;
				fft_ready=1;
			}
    }
}

/**
 * @brief  串口接收完成中断回调函数
 * @param  huart: 串口句柄
 * @note   实现单字节中断接收，每次接收一个字节后重置超时定时器
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART1){
		// 重置超时定时器，重新开始计时
		__HAL_TIM_SET_COUNTER(&htim4,0);
		// 索引递增，准备接收下一个字节
		uart_rx_index++;
		// 启动下一次单字节接收
		HAL_UART_Receive_IT(&huart1,&uart_rx_buffer[uart_rx_index],1);
	}
}

/**
 * @brief  定时器周期溢出中断回调函数
 * @param  htim: 定时器句柄
 * @note   TIM4用于串口接收超时，TIM6保留未使用
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// TIM4：串口接收超时定时器（1MHz时钟，50ms溢出）
	if(htim->Instance==TIM4){
		
	}
	// TIM6：保留，可用于其他定时任务
	if(htim->Instance==TIM6){
		
	}
}