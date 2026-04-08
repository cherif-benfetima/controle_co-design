#include <stdint.h>
#include <stdio.h>

#define SENSOR_CONTROL_BASE 0x00001020u
#define SENSOR_STATUS_BASE  0x00001030u
#define SENSOR_DATA6_BASE   0x000010A0u
#define KP_BASE             0x000010B0u
#define START_SL_BASE       0x00001110u
#define KD_BASE             0x00001120u

#define LEDS_BASE           0x00002010u

#define CTRL_OP_MASK        0x03u
#define CTRL_SRC_SENSOR     0x04u 

#define STATUS_DONE_MASK    0x01u
#define STATUS_OVF_MASK     0x02u
#define STATUS_SENSOR_RDY   0x04u

#define IOWR(base, data) (*((volatile uint32_t*)(base)) = (uint32_t)(data))
#define IORD(base)       (*((volatile uint32_t*)(base)))















  return 0;
}