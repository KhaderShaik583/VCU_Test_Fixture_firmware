#include <stdio.h>
#include "version.h"

int main(void)
{
    printf("%d.%d\n", get_fw_max_ver(),get_fw_min_ver());
    
    return 0;
}
