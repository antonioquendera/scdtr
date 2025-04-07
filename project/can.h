#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>
#include "commands.h"
#include "CircularBuffer.hpp"
#include "mcp2515.h"


typedef unsigned char __u8;
typedef unsigned short __u16;
typedef unsigned long __u32;


/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000UL /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000UL /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000UL /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFUL /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFUL /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFUL /* omit EFF, RTR, ERR flags */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28 : CAN identifier (11/29 bit)
 * bit 29   : error message frame flag (0 = data frame, 1 = error message)
 * bit 30   : remote transmission request flag (1 = rtr frame)
 * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef __u32 canid_t;

#define CAN_SFF_ID_BITS     11
#define CAN_EFF_ID_BITS     29

/* CAN payload length and DLC definitions according to ISO 11898-1 */
#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

struct can_frame {
    canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    __u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    __u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));

};

uint32_t bytes_to_msg(uint8_t *b);
uint32_t can_frame_to_msg(can_frame *frm);
uint32_t error_flags_to_msg(uint8_t canintf, uint8_t eflg);
void msg_to_bytes(uint32_t msg, uint8_t *bytes);
void print_message(int number, int node, int id, int val);
void print_can_errors(uint8_t canintf, uint8_t eflg);
void read_interrupt(uint gpio, uint32_t events);
#endif /* CAN_H_ */
