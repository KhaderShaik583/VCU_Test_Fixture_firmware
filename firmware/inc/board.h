#ifndef BOARD_H
#define BOARD_H

#include "status.h"
#include "msdi.h"

/* PORT A Block */
#define AUX_5V_EN_PIN           0U
#define AUX_CURRENT_SNS_PIN     1U
#define S32K_DBG_RX_PIN         2U    
#define S32K_DBG_TX_PIN         3U    
#define LEFT_IND_IO_PIN         6U
#define RTC_CLK_HZ_PIN          7U
#define IMX_S32K_RX_PIN         8U    
#define IMX_S32K_TX_PIN         9U    
#define RTC_IRQ_PIN             11U
#define BMS_CAN_RXD_PIN         12U
#define BMS_CAN_TXD_PIN         13U
#define SHUTDOWN_MCU_N_PIN      14U
#define USB_VBUS_CTL_PIN        15U
#define VBAT_EN1_PIN            16U
#define REAR_BRAKE_LED_EN_PIN   17U
#define KEY_WAKE_SIG_PIN        25U
#define WAKEUP_IN_MCU_PIN       26U  
#define MOTOR_CAN_TX_PIN        27U
#define MOTOR_CAN_RX_PIN        28U
#define RESET_MCU_PIN           29U    
#define IMU_CS_PIN              30U
#define HORN_EN_PIN             31U

/* PORT B Block */
#define SWIF_CS_PIN         0U
#define SWIF_SI_PIN         1U
#define SWIF_CLK_PIN        2U
#define SWIF_SO_PIN         3U
#define MII_RMII_MDIO_PIN   4U
#define MII_RMII_MDC_PIN    5U
#define ABS_OFF_MC_PIN      8U
#define ETH_SW_RST_PIN      9U
#define TACHY_MC_PIN        10U
#define NETWRK_STAT_PIN     11U  
#define DBA_CAN_RX_PIN      12U
#define DBA_CAN_TX_PIN      13U
#define IMU_SCL_PIN         14U
#define IMU_SDO_PIN         15U
#define IMU_SDI_PIN         16U
#define PAC9129_IRQ_PIN     17U
#define HORN_VCC_EN_PIN     18U   
#define SWIF_AMUX_PIN       20U
#define REV_EN_PIN          21U
#define HORN_IS_PIN         22U
#define DRL_TL_IS_PIN       23U
#define DEN_PIN             25U
#define ENET_MOSI_PIN       27U
#define ENET_MISO_PIN       28U
#define SWIF_WAKE_UP_PIN    29U

/* PORT C Block */
#define MII_RMII_RX1_PIN    0U
#define MII_RMII_RX0_PIN    1U
#define MII_RMII_TXD0_PIN   2U
#define ENET_CS_PIN         3U
#define LTE_TX_PIN          6U
#define LTC_RX_PIN          7U
#define MODE_GPIO_SW2_PIN   8U
#define HIGH_BEAM_PIN       9U   
#define IMX6_PMIC_EN_PIN    10U
#define MODE_GPIO_SW_PIN    11U
#define DRL_LED_EN_PIN      12U 
#define SWIF_IRQ_PIN        13U
#define BR_NP_IS_PIN        14U
#define HL_IS_PIN           15U
#define MII_RMII_RX_ER_PIN  16U
#define MII_RMII_RX_DV_PIN  17U
#define TAIL_LAMP_EN_PIN    19U
#define AUX_EN_PIN          23U  
#define THRT_VDD_MONIT_PIN  27U  
#define MAIN_VDC_MONIT_PIN  28U   
#define RIGHT_IND_IO_PIN    29U  
#define PG_4G_PIN           30U  
#define PG_5V_PIN           31U  

/* PORT D Block */
#define ETM_TRACE_D0_PIN        0U   
#define LTE_IRQ_PIN             1U
#define THROTTLE1_PIN           2U
#define DBA_CAN_ERR_PIN         3U
#define DBA_CAN_EN_PIN          4U
#define MODE1_EN_PIN            5U    
#define IMX6_RESET_PIN          6U    
#define MII_RMII_TXD1_PIN       7U
#define MOTOR_CON_ON_OFF_PIN    8U   
#define IO_EN_PIN               9U
#define MII_RX_CLK_PIN          10U
#define MII_RMII_TX_CLK_PIN     11U
#define MII_RMII_TX_EN_PIN      12U
#define VDD_MCU_EN_PIN          13U
#define IMU_IRQ_PIN             14U
#define ETM_TRACE_D3_PIN        15U
#define ETM_TRACE_D2_PIN        16U
#define ON_OFF_MCU_PIN          17U
#define HANDLE_LED_IS_PIN       18U
#define LTE_SPI_IRQ_PIN         19U
#define DBA_STB_N_PIN           22U
#define EXT_WDT_WDI_PIN         23U
#define EXT_WDT_EN_PIN          24U
#define ABS_EN_PIN              27U
#define NBR_PLT_LED_EN_PIN      28U
#define MOTOR_CAN_ERR_PIN       29U
#define MOTOR_STB_N_PIN         30U

/* PORT E Block */
#define RTC_SDA_PIN             0U
#define RTC_SCL_PIN             1U
#define IMX6_GPIO_01_PIN        2U
#define IMX6_GP32_PIN           3U   
#define ETM_TRACE_D1_PIN        4U   
#define RIGHT_IND_PIN           5U
#define LOW_BEAM_PIN            6U
#define HEAD_LAMP_12V_EN_PIN    7U
#define FWD_EN_PIN              8U
#define IMX6_GP34_PIN           9U  
#define LEFT_IND_PIN            10U   
#define LTE_MODULE_STATUS_PIN   11U  
#define MOTOR_CAN_EN_PIN        12U
#define IMU_VDD_EN_PIN          13U
#define IMX6_VDD_EN_PIN         14U
#define VBAT_EN2_PIN            16U
#define ABS_LED_IS_PIN          19U
#define THROTTLE2_PIN           20U
#define LED_RED_PIN             21U
#define LED_GREEN_PIN           22U
#define BMS_STB_N_PIN           23U
#define BMS_CAN_EN_PIN          24U
#define BMS_CAN_ERR_PIN         25U
#define ENET_SPI_SCK_PIN        15U

/* Port Definitions */

#define AUX_5V_EN_PORT          	PORTA
#define AUX_CURRENT_SNS_PORT     	PORTA
#define S32K_DBG_RX_PORT         	PORTA
#define S32K_DBG_TX_PORT        	PORTA
#define LEFT_IND_IO_PORT     	    PORTA
#define RTC_CLK_HZ_PORT          	PORTA
#define IMX_S32K_RX_PORT         	PORTA
#define IMX_S32K_TX_PORT         	PORTA
#define RTC_IRQ_PORT             	PORTA
#define BMS_CAN_RXD_PORT         	PORTA
#define BMS_CAN_TXD_PORT         	PORTA
#define SHUTDOWN_MCU_N_PORT      	PORTA
#define USB_VBUS_CTL_PORT        	PORTA
#define VBAT_EN1_PORT            	PORTA
#define REAR_BRAKE_LED_EN_PORT   	PORTA
#define KEY_WAKE_SIG_PORT        	PORTA
#define WAKEUP_IN_MCU_PORT       	PORTA
#define MOTOR_CAN_TX_PORT        	PORTA
#define MOTOR_CAN_RX_PORT        	PORTA
#define RESET_MCU_PORT           	PORTA
#define IMU_CS_PORT              	PORTA
#define HORN_EN_PORT             	PORTA


#define SWIF_CS_PORT         	PORTB
#define SWIF_SI_PORT         	PORTB
#define SWIF_CLK_PORT        	PORTB
#define SWIF_SO_PORT         	PORTB
#define MII_RMII_MDIO_PORT   	PORTB
#define MII_RMII_MDC_PORT    	PORTB
#define ABS_OFF_MC_PORT      	PORTB
#define ETH_SW_RST_PORT      	PORTB
#define TACHY_MC_PORT        	PORTB
#define NETWRK_STAT_PORT     	PORTB
#define DBA_CAN_RX_PORT      	PORTB
#define DBA_CAN_TX_PORT      	PORTB
#define IMU_SCL_PORT      	    PORTB
#define IMU_SDO_PORT      	    PORTB
#define IMU_SDI_PORT      	    PORTB
#define PAC9129_IRQ_PORT     	PORTB
#define HORN_VCC_EN_PORT      	PORTB
#define SWIF_AMUX_PORT       	PORTB
#define REV_EN_PORT          	PORTB
#define HORN_IS_PORT         	PORTB
#define DRL_TL_IS_PORT       	PORTB
#define DEN_PORT             	PORTB
#define ENET_MOSI_PORT          PORTB
#define ENET_MISO_PORT          PORTB
#define SWIF_WAKE_UP_PORT       PORTB

#define MII_RMII_RX1_PORT    	PORTC
#define MII_RMII_RX0_PORT    	PORTC
#define MII_RMII_TXD0_PORT   	PORTC
#define ENET_CS_PORT         	PORTC
#define LTE_TX_PORT          	PORTC
#define LTC_RX_PORT          	PORTC
#define MODE_GPIO_SW2_PORT   	PORTC
#define HIGH_BEAM_PORT       	PORTC
#define IMX6_PMIC_EN_PORT    	PORTC
#define MODE_GPIO_SW_PORT    	PORTC
#define DRL_LED_EN_PORT      	PORTC
#define SWIF_IRQ_PORT        	PORTC
#define BR_NP_IS_PORT        	PORTC
#define HL_IS_PORT           	PORTC
#define MII_RMII_RX_ER_PORT  	PORTC
#define MII_RMII_RX_DV_PORT  	PORTC
#define TAIL_LAMP_EN_PORT    	PORTC
#define AUX_EN_PORT         	PORTC
#define THRT_VDD_MONIT_PORT    	PORTC
#define MAIN_VDC_MONIT_PORT   	PORTC
#define RIGHT_IND_IO_PORT    	PORTC
#define PG_4G_PORT           	PORTC
#define PG_5V_PORT           	PORTC

#define LTE_IRQ_PORT             	PORTD
#define THROTTLE1_PORT           	PORTD
#define DBA_CAN_ERR_PORT         	PORTD
#define DBA_CAN_EN_PORT          	PORTD
#define MODE1_EN_PORT            	PORTD
#define IMX6_RESET_PORT         	PORTD
#define MII_RMII_TXD1_PORT       	PORTD
#define MOTOR_CON_ON_OFF_PORT     	PORTD
#define IO_EN_PORT               	PORTD
#define MII_RX_CLK_PORT          	PORTD
#define MII_RMII_TX_CLK_PORT     	PORTD
#define MII_RMII_TX_EN_PORT      	PORTD
#define VDD_MCU_EN_PORT          	PORTD
#define IMU_IRQ_PORT             	PORTD
#define ON_OFF_MCU_PORT          	PORTD
#define HANDLE_LED_IS_PORT       	PORTD
#define LTE_SPI_IRQ_PORT         	PORTD
#define DBA_STB_N_PORT           	PORTD
#define EXT_WDT_WDI_PORT         	PORTD
#define EXT_WDT_EN_PORT          	PORTD
#define ABS_EN_PORT              	PORTD
#define NBR_PLT_LED_EN_PORT         PORTD
#define MOTOR_CAN_ERR_PORT          PORTD
#define MOTOR_STB_N_PORT            PORTD
#define ETM_TRACE_D0_PORT           PORTD
#define ETM_TRACE_D2_PORT           PORTD
#define ETM_TRACE_D3_PORT           PORTD

#define RTC_SDA_PORT             	PORTE
#define RTC_SCL_PORT             	PORTE
#define IMX6_GPIO_01_PORT        	PORTE
#define IMX6_GP32_PORT           	PORTE
#define RIGHT_IND_PORT          	PORTE
#define LOW_BEAM_PORT            	PORTE
#define HEAD_LAMP_12V_EN_PORT       PORTE
#define FWD_EN_PORT              	PORTE
#define IMX6_GP34_PORT           	PORTE
#define LEFT_IND_PORT            	PORTE
#define LTE_MODULE_STATUS_PORT   	PORTE
#define MOTOR_CAN_EN_PORT        	PORTE
#define IMU_VDD_EN_PORT          	PORTE
#define IMX6_VDD_EN_PORT         	PORTE
#define VBAT_EN2_PORT            	PORTE
#define ABS_LED_IS_PORT          	PORTE
#define THROTTLE2_PORT           	PORTE
#define LED_RED_PORT             	PORTE
#define LED_GREEN_PORT           	PORTE
#define BMS_STB_N_PORT              PORTE
#define BMS_CAN_EN_PORT             PORTE
#define BMS_CAN_ERR_PORT            PORTE
#define ENET_SPI_SCK_PORT           PORTE
#define ETM_TRACE_D1_PORT           PORTE

/* GPIO Definitions */
#define AUX_5V_EN_GPIO          	PTA
#define AUX_CURRENT_SNS_GPIO     	PTA
#define S32K_DBG_RX_GPIO         	PTA
#define S32K_DBG_TX_GPIO        	PTA
#define LEFT_IND_IO_GPIO     	    PTA
#define RTC_CLK_HZ_GPIO          	PTA
#define IMX_S32K_RX_GPIO         	PTA
#define IMX_S32K_TX_GPIO         	PTA
#define RTC_IRQ_GPIO             	PTA
#define BMS_CAN_RXD_GPIO         	PTA
#define BMS_CAN_TXD_GPIO         	PTA
#define SHUTDOWN_MCU_N_GPIO      	PTA
#define USB_VBUS_CTL_GPIO        	PTA
#define VBAT_EN1_GPIO            	PTA
#define REAR_BRAKE_LED_EN_GPIO   	PTA
#define KEY_WAKE_SIG_GPIO        	PTA
#define WAKEUP_IN_MCU_GPIO       	PTA
#define MOTOR_CAN_TX_GPIO        	PTA
#define MOTOR_CAN_RX_GPIO        	PTA
#define RESET_MCU_GPIO           	PTA
#define IMU_CS_GPIO              	PTA
#define HORN_EN_GPIO             	PTA

#define SWIF_CS_GPIO         	PTB
#define SWIF_SI_GPIO         	PTB
#define SWIF_CLK_GPIO        	PTB
#define SWIF_SO_GPIO         	PTB
#define MII_RMII_MDIO_GPIO   	PTB
#define MII_RMII_MDC_GPIO    	PTB
#define ABS_OFF_MC_GPIO      	PTB
#define ETH_SW_RST_GPIO      	PTB
#define TACHY_MC_GPIO        	PTB
#define NETWRK_STAT_GPIO     	PTB
#define DBA_CAN_RX_GPIO      	PTB
#define DBA_CAN_TX_GPIO      	PTB
#define IMU_SCL_GPIO      	    PTB
#define IMU_SDO_GPIO      	    PTB
#define IMU_SDI_GPIO      	    PTB
#define PAC9129_IRQ_GPIO     	PTB
#define HORN_VCC_EN_GPIO      	PTB
#define SWIF_AMUX_GPIO       	PTB
#define REV_EN_GPIO          	PTB
#define HORN_IS_GPIO         	PTB
#define DRL_TL_IS_GPIO          PTB
#define DEN_GPIO                PTB
#define ENET_MOSI_GPIO          PTB
#define ENET_MISO_GPIO          PTB
#define SWIF_WAKE_UP_GPIO       PTB

#define MII_RMII_RX1_GPIO    	PTC
#define MII_RMII_RX0_GPIO    	PTC
#define MII_RMII_TXD0_GPIO   	PTC
#define ENET_CS_GPIO         	PTC
#define LTE_TX_GPIO          	PTC
#define LTC_RX_GPIO          	PTC
#define MODE_GPIO_SW2_GPIO   	PTC
#define HIGH_BEAM_GPIO       	PTC
#define IMX6_PMIC_EN_GPIO    	PTC
#define MODE_GPIO_SW_GPIO    	PTC
#define DRL_LED_EN_GPIO      	PTC
#define SWIF_IRQ_GPIO        	PTC
#define BR_NP_IS_GPIO        	PTC
#define HL_IS_GPIO           	PTC
#define MII_RMII_RX_ER_GPIO  	PTC
#define MII_RMII_RX_DV_GPIO  	PTC
#define TAIL_LAMP_EN_GPIO    	PTC
#define AUX_EN_GPIO         	PTC
#define THRT_VDD_MONIT_GPIO    	PTC
#define MAIN_VDC_MONIT_GPIO   	PTC
#define RIGHT_IND_IO_GPIO       PTC
#define PG_4G_GPIO              PTC
#define PG_5V_GPIO              PTC

#define LTE_IRQ_GPIO             	PTD
#define THROTTLE1_GPIO           	PTD
#define DBA_CAN_ERR_GPIO         	PTD
#define DBA_CAN_EN_GPIO          	PTD
#define MODE1_EN_GPIO            	PTD
#define IMX6_RESET_GPIO         	PTD
#define MII_RMII_TXD1_GPIO       	PTD
#define MOTOR_CON_ON_OFF_GPIO     	PTD
#define IO_EN_GPIO               	PTD
#define MII_RX_CLK_GPIO          	PTD
#define MII_RMII_TX_CLK_GPIO     	PTD
#define MII_RMII_TX_EN_GPIO      	PTD
#define VDD_MCU_EN_GPIO          	PTD
#define IMU_IRQ_GPIO             	PTD
#define ON_OFF_MCU_GPIO          	PTD
#define HANDLE_LED_IS_GPIO       	PTD
#define LTE_SPI_IRQ_GPIO         	PTD
#define DBA_STB_N_GPIO           	PTD
#define EXT_WDT_WDI_GPIO         	PTD
#define EXT_WDT_EN_GPIO          	PTD
#define ABS_EN_GPIO                 PTD
#define NBR_PLT_LED_EN_GPIO         PTD
#define MOTOR_CAN_ERR_GPIO          PTD
#define MOTOR_STB_N_GPIO            PTD
#define ETM_TRACE_D0_GPIO           PTD
#define ETM_TRACE_D2_GPIO           PTD
#define ETM_TRACE_D3_GPIO           PTD

#define RTC_SDA_GPIO             	PTE
#define RTC_SCL_GPIO             	PTE
#define IMX6_GPIO_01_GPIO        	PTE
#define IMX6_GP32_GPIO           	PTE
#define RIGHT_IND_GPIO          	PTE
#define LOW_BEAM_GPIO            	PTE
#define HEAD_LAMP_12V_EN_GPIO       PTE
#define FWD_EN_GPIO              	PTE
#define IMX6_GP34_GPIO           	PTE
#define LEFT_IND_GPIO            	PTE
#define LTE_MODULE_STATUS_GPIO   	PTE
#define MOTOR_CAN_EN_GPIO        	PTE
#define IMU_VDD_EN_GPIO          	PTE
#define IMX6_VDD_EN_GPIO         	PTE
#define VBAT_EN2_GPIO            	PTE
#define ABS_LED_IS_GPIO          	PTE
#define THROTTLE2_GPIO           	PTE
#define LED_RED_GPIO             	PTE
#define LED_GREEN_GPIO           	PTE
#define BMS_STB_N_GPIO              PTE
#define BMS_CAN_EN_GPIO             PTE
#define BMS_CAN_ERR_GPIO            PTE
#define ENET_SPI_SCK_GPIO           PTE
#define ETM_TRACE_D1_GPIO           PTE

#define RTC_I2C_ADDR             (0xA2U >> 1U)
#define RTC_I2C_BAUD             100U
#define RTC_I2C_IF               1U
#define RTC_IF_INST              LPI2C1

#define PAC1921_I2C_ADDR 		 (0x98U >> 1U)
#define PAC1921_I2C_BAUD 		 (100U)
#define PAC1921_I2C_IF  		 (1U)

/* FTM for PWM signal measurement at Right and left ind GPIO */
#define FLEX_TIMER_IC0_RIGHT_IND    (1U)
#define FLEX_TIMER_IC0_LEFT_IND  	(0U)
#define SYS_FTM_INST				(0U)
#define INST_LPUART1 				(1U)
#define CAN_IF_BMS                  (1U)
#define CAN_IF_MOTOR				(0U)
#define CAN_IF_ABS				    (2U)
#define SYS_LTE_UART_IF             (1U)
#define SYS_LTE_UART_BAUD           (115200U)
#define SYS_ETH_IF                  (0U)
#define SW_DEBOUNCE_DELAY_US        (50000U)

status_t board_init(void);
msdi_status_t init_CD10x0(void);
void ec25_pwr_init(void);

#endif /* BOARD_H */

