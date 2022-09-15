#ifndef FOTA_H
#define FOTA_H

#define S32_VCU_FOTA_DL_SIZE        (0x50000U)
#define S32_BMS_FOTA_DL_SIZE        (0x20000U)

/* Refer MAP file */
#define S32_VCU_FOTA_DL_ADDR        (0x90800U)     
#define S32_BMS_FOTA_DL_ADDR        (S32_VCU_FOTA_DL_ADDR + S32_VCU_FOTA_DL_SIZE + 32U)

#define BMS_FOTA_BLOCK_SIZE         (40U)

#endif /* FOTA_H */
