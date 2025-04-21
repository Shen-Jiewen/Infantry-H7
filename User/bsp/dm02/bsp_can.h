#ifndef BSP_CAN_H_
#define BSP_CAN_H_

#include "main.h"
#include "dji_3508.h"
#include "dji_6020.h"
#include "dm_4310.h"
#include "dm_4310_v41.h"
#include "supercap.h"
#include "dji_2006.h"

extern void fdcan1_filter_init(void);
extern void fdcan2_filter_init(void);
extern void fdcan3_filter_init(void);

#endif //BSP_CAN_H_
