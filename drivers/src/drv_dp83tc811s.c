 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2020] - [2021] Ultraviolette Automotive Pvt. Ltd.
 * All Rights Reserved.
 * 
 * NOTICE: All information contained herein is, and remains the 
 * property of Ultraviolette Automotive Pvt. Ltd. and its suppliers, if 
 * any. The intellectual and technical concepts contained herein are 
 * proprietary to Ultraviolette Automotive  and its suppliers and may 
 * be covered by U.S. and Foreign Patents, patents in process,  and 
 * are protected by trade secret or copyright law. Dissemination of 
 * this information or reproduction of this material is strictly 
 * forbidden unless prior written permission is obtained from 
 * Ultraviolette Automotive Pvt. Ltd.
 * 
 *
 * Author : Rishi F. [011]
 *
 */
 
#include "drv_dp83tc811s.h"
#include "netifapi.h"

static uint8_t instance = 0U;

void drv_mdio_reg_read()
{
    
}



void drv_mdio_reg_write(uint8_t phy_address, uint8_t phy_reg, uint16_t phy_data, uint32_t timeout_ms)
{

}

void drv_mdio_platform_init(struct netif *netif)
{
    volatile uint16_t phy_data = 0U;
    
    instance = netif->num;
    
    ENET_DRV_EnableMDIO(instance, false);
    
    (void)ENET_DRV_MDIOWrite(instance, DP83TC811_PHY_ADDR, DP83TC811_BMCR, DP83TC811_BMCR_RESET, 50U);
    while(ENET_DRV_MDIORead(instance, DP83TC811_PHY_ADDR, DP83TC811_BMCR, (uint16_t *)&phy_data, 50U))
    {
        if(phy_data == DP83TC811_BMCR_RESET)
        {
            break;
        }
    }
    
    (void)ENET_DRV_MDIORead(netif->num, DP83TC811_PHY_ADDR, DP83TC811_PHYID2, (uint16_t *)&phy_data, 50U);
}
