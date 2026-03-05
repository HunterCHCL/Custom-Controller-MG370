#include "verification.h"
#include "main.h"
/**
 * @brief 计算校验和 (Checksum): 将所有数据累加，并返回其低8位。
 */
uint8_t Verification_CalculateChecksum(uint8_t *data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 在末位添加校验和
 * @param data 原始数据，data[len] 必须有额外的一个字节空间
 */
void Verification_AddChecksum(uint8_t *data, uint16_t len) {
    if (data == NULL) return;
    data[len] = Verification_CalculateChecksum(data, len);
}

/**
 * @brief 校验累加和 (数据+最后一位校验位)
 * @param len 数组总长度 (含校验位)
 * @return 1:成功, 0:失败
 */
uint8_t Verification_CheckChecksum(uint8_t *data, uint16_t len) {
    if (data == NULL || len < 2) return 0;
    uint8_t calculated = Verification_CalculateChecksum(data, len - 1);
    return (calculated == data[len - 1]) ? 1 : 0;
}

/**
 * @brief 计算异或校验位 (XOR/BCC)
 */
uint8_t Verification_CalculateXOR(uint8_t *data, uint16_t len) {
    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}

/**
 * @brief 添加异或校验位
 */
void Verification_AddXOR(uint8_t *data, uint16_t len) {
    if (data == NULL) return;
    data[len] = Verification_CalculateXOR(data, len);
}

/**
 * @brief 校验异或校验 (数据+最后一位校验位)
 * @param len 数组总长度 (含校验位)
 * @return 1:成功, 0:失败
 */
uint8_t Verification_CheckXOR(uint8_t *data, uint16_t len) {
    if (data == NULL || len < 2) return 0;
    uint8_t calculated = Verification_CalculateXOR(data, len - 1);
    return (calculated == data[len - 1]) ? 1 : 0;
}
