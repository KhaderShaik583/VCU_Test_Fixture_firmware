#include "lte_util.h"


uint8_t is_atc_ok(uint8_t *rxbuffer, uint32_t len)
{
    /* Check rxbuffer for OK */
    uint32_t i = 0U;
    uint32_t flag = 0U;
    

    for(i = 0; i < len - 2U; i++)
    {
        if(rxbuffer[i] == 'O' && rxbuffer[i + 1] == 'K')
        {
            flag = 1U;
            break;
        }
    }
    
    return flag;
}


static uint32_t qmt_open_rsp_parse(void)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'Q') &&
           (saveBuffer[i + 2] == 'M') && (saveBuffer[i + 3] == 'T') &&
           (saveBuffer[i + 4] == 'O') && (saveBuffer[i + 5] == 'P') &&           
           (saveBuffer[i + 6] == 'E') && (saveBuffer[i + 7] == 'N') &&
           (saveBuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            if(saveBuffer[i] == '-')
            {
                return 6;
            }
            else
            {
                ret = saveBuffer[i] - '0';
            }
        }
    }
    
    return ret;
}

static uint32_t qmt_close_rsp_parse(void)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'Q') &&
           (saveBuffer[i + 2] == 'M') && (saveBuffer[i + 3] == 'T') &&
           (saveBuffer[i + 4] == 'C') && (saveBuffer[i + 5] == 'L') &&           
           (saveBuffer[i + 6] == 'O') && (saveBuffer[i + 7] == 'S') &&
           (saveBuffer[i + 8] == 'E') && (saveBuffer[i + 9] == ':'))
        {    
            i = i + 9U + 4U;
            if(saveBuffer[i] == '-')
            {
                return 6;
            }
            else
            {
                ret = saveBuffer[i] - '0';
            }
        }
    }
    
    return ret;
}


static uint8_t qmt_disconnect_rsp_parse(void)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'Q') &&
           (saveBuffer[i + 2] == 'M') && (saveBuffer[i + 3] == 'T') &&
           (saveBuffer[i + 4] == 'D') && (saveBuffer[i + 5] == 'I') &&           
           (saveBuffer[i + 6] == 'S') && (saveBuffer[i + 7] == 'C') &&
           (saveBuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            if(saveBuffer[i] == '0')
            {
                ret = 0;
            }
            else
            {
                ret = 1;
            }
        }
    }
    
    return ret;
}

static uint32_t qmt_publish_rsp_parse(void)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'Q') &&
           (saveBuffer[i + 2] == 'M') && (saveBuffer[i + 3] == 'T') &&
           (saveBuffer[i + 4] == 'P') && (saveBuffer[i + 5] == 'U') &&           
           (saveBuffer[i + 6] == 'B') && (saveBuffer[i + 7] == ':'))
        {    
            i = i + 7U + 6U;
            ret = (saveBuffer[i] - '0');
            
        }
    }
    
    return ret;
}

static uint32_t qmt_sub_rsp_parse(void)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'Q') &&
           (saveBuffer[i + 2] == 'M') && (saveBuffer[i + 3] == 'T') &&
           (saveBuffer[i + 4] == 'S') && (saveBuffer[i + 5] == 'U') &&           
           (saveBuffer[i + 6] == 'B') && (saveBuffer[i + 7] == ':'))
        {    
            i = i + 7U + 6U;
            ret = (saveBuffer[i] - '0');
            
        }
    }
    
    return ret;
}

static uint32_t qmt_cgreg_rsp_parse(void)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'C') &&
           (saveBuffer[i + 2] == 'G') && (saveBuffer[i + 3] == 'R') &&
           (saveBuffer[i + 4] == 'E') && (saveBuffer[i + 5] == 'G') &&           
           (saveBuffer[i + 6] == ':'))
        {    
            i = i + 10U;
            ret = (saveBuffer[i] - '0');
            
        }
    }
    
    return ret;
}

static uint32_t qmt_qgpsloc_rsp_parse(void)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'Q') &&
           (saveBuffer[i + 2] == 'M') && (saveBuffer[i + 3] == 'T') &&
           (saveBuffer[i + 4] == 'S') && (saveBuffer[i + 5] == 'T') &&           
           (saveBuffer[i + 6] == 'A') && (saveBuffer[i + 7] == 'T') &&
           (saveBuffer[i + 8] == ':'))
        {    
            
            ret = 9U;
            
        }
        
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'Q') &&
           (saveBuffer[i + 2] == 'G') && (saveBuffer[i + 3] == 'P') &&
           (saveBuffer[i + 4] == 'S') && (saveBuffer[i + 5] == 'L') &&           
           (saveBuffer[i + 6] == 'O') && (saveBuffer[i + 7] == 'C') &&
           (saveBuffer[i + 8] == ':'))
        {    
            ret = 1U; 
        }
        else
        {
            ret = 0U;
        }
    }
    
    return ret;    
}


static uint32_t qmt_conn_rsp_parse(void)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((saveBuffer[i] == '+') && (saveBuffer[i + 1] == 'Q') &&
           (saveBuffer[i + 2] == 'M') && (saveBuffer[i + 3] == 'T') &&
           (saveBuffer[i + 4] == 'C') && (saveBuffer[i + 5] == 'O') &&           
           (saveBuffer[i + 6] == 'N') && (saveBuffer[i + 7] == 'N') &&
           (saveBuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            ret = (saveBuffer[i] - '0') & 0xFF;
            ret |= ((saveBuffer[i + 2U] - '0') & 0xFF) << 4U;

        }
    }
    
    return ret;
}


static uint8_t buffer_contains_spcl_char(void)
{
    /* Check rxbuffer for '>' */
    uint32_t i = 0U;
    uint32_t flag = 0U;
    
    for(i = 0; i < bufferIdx; i++)
    {
        if(saveBuffer[i] == '>')
        {
            flag = 1U;
            break;
        }
    }
    
    return flag;    
}