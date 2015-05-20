//
// Created by Texot Qexoq on 5/16/15.
//

#ifndef __CAN_PACKET_H__
#define __CAN_PACKET_H__

/* aaa bbbc dddd

 * aaa:  目标
 * bbb:  数据包类型
 * c:    数据放置类型
 * dddd: 目标地址/源地址
 */
#define CAN_PACKET_TYPE_MASK_DESTTYPE 0x700				        /* 111 0000	0000 */
#define CAN_PACKET_TYPE_MASK_DATATYPE 0x0e0				        /* 000 1110	0000 */
#define CAN_PACKET_TYPE_MASK_PLACETYPE 0x010			        /* 000 0001	0000 */
#define CAN_PACKET_TYPE_MASK_DESTADDR_STRICT 0xf
#define CAN_PACKET_TYPE_MASK_DESTADDR_ALL 0
#define CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT 0xf
#define CAN_PACKET_TYPE_MASK_SOURCEADDR_ALL 0

#define CAN_PACKET_DESTTYPE_DRIVER \
	(0x0 << 8)									/* 000 xxxx xxxx */
#define CAN_PACKET_DESTTYPE_CENTER \
	(0x1 << 8)									/* 001 xxxx xxxx */
#define CAN_PACKET_DESTTYPE_GIMBAL \
	(0x2 << 8)									/* 010 xxxx xxxx */
#define CAN_PACKET_DESTTYPE_ALL \
	(0x2 << 8)									/* 111 xxxx xxxx */

// driver fifo0
#define CAN_PACKET_DRIVER_DATATYPE_ENABLE \
	(0x0 << 5)									/* xxx 000x xxxx */
#define CAN_PACKET_DRIVER_DATATYPE_CONFIG \
	(0x1 << 5)									/* xxx 001x xxxx */
#define CAN_PACKET_DRIVER_DATATYPE_CONTROL \
	(0x2 << 5)									/* xxx 010x xxxx */

// center fifo0
#define CAN_PACKET_CENTER_DATATYPE_DRIVER_STDOUT \
	(0x0 << 5)									/* xxx 000x xxxx */
#define CAN_PACKET_CENTER_DATATYPE_DRIVER_STATUS \
	(0x1 << 5)									/* xxx 001x xxxx */
// center fifo1
#define CAN_PACKET_CENTER_DATATYPE_GIMBAL_OTHERS \
    (0x0 << 5)									/* xxx 000x xxxx */
#define CAN_PACKET_CENTER_DATATYPE_GIMBAL_ACCEL \
    (0x1 << 5)									/* xxx 001x xxxx */
#define CAN_PACKET_CENTER_DATATYPE_GIMBAL_GYRO \
    (0x2 << 5)									/* xxx 010x xxxx */
#define CAN_PACKET_CENTER_DATATYPE_GIMBAL_YAW_PITCH \
    (0x3 << 5)									/* xxx 011x xxxx */
#define CAN_PACKET_CENTER_DATATYPE_GIMBAL_ROLL \
    (0x4 << 5)									/* xxx 100x xxxx */

// gimbal fifo0
#define CAN_PACKET_GIMBAL_DATATYPE_GIMBAL_MOTOR_STATUS \
    (0x0 << 5)                                  /* xxx 000x xxxx */
// gimbal fifo1
#define CAN_PACKET_GIMBAL_DATATYPE_SET_SPEED \
	(0x0 << 5)									/* xxx 000x xxxx */
#define CAN_PACKET_GIMBAL_DATATYPE_SET_FRICTION \
	(0x1 << 5)									/* xxx 001x xxxx */
#define CAN_PACKET_GIMBAL_DATATYPE_SET_SHOOTER \
	(0x2 << 5)									/* xxx 010x xxxx */
#define CAN_PACKET_GIMBAL_DATATYPE_SET_LASER \
    (0x3 << 5)                                  /* xxx 011x xxxx */
#define CAN_PACKET_GIMBAL_DATATYPE_SET_YAW_PITCH \
    (0x4 << 5)                                  /* xxx 100x xxxx */


#define CAN_PACKET_PLACETYPE_SPECIFIC \
	(0x0 << 4)									/* xxx xxx0 xxxx */
#define CAN_PACKET_PLACETYPE_ALLINONE \
	(0x1 << 4)									/* xxx xxx1 xxxx */


#endif //__CAN_PACKET_H__
