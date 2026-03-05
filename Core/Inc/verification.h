#ifndef __VERIFICATION_H
#define __VERIFICATION_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 计算校验和 (Checksum)
 * @param data 数据指针
 * @param len 数据长度
 * @return uint8_t 校验位
 */
uint8_t Verification_CalculateChecksum(uint8_t *data, uint16_t len);

/**
 * @brief 添加校验和校验位到数据末尾
 * @param data 数据指针（需要足够空间存放校验位）
 * @param len 原始数据长度
 */
void Verification_AddChecksum(uint8_t *data, uint16_t len);

/**
 * @brief 验证校验和
 * @param data 数据指针（最后一位是校验位）
 * @param len 整个数组的总长度（含末尾校验位）
 * @return uint8_t 1:校验成功, 0:校验失败
 */
uint8_t Verification_CheckChecksum(uint8_t *data, uint16_t len);

/**
 * @brief 计算异或校验位 (XOR / BCC)
 * @param data 数据指针
 * @param len 数据长度
 * @return uint8_t 校验位
 */
uint8_t Verification_CalculateXOR(uint8_t *data, uint16_t len);

/**
 * @brief 添加异或校验位到数据末尾
 * @param data 数据指针
 * @param len 原始数据长度
 */
void Verification_AddXOR(uint8_t *data, uint16_t len);

/**
 * @brief 验证异或校验
 * @param data 数据指针（最后一位是校验位）
 * @param len 整个数组的总长度（含末尾校验位）
 * @return uint8_t 1:校验成功, 0:校验失败
 */
uint8_t Verification_CheckXOR(uint8_t *data, uint16_t len);

#endif /* __VERIFICATION_H */
