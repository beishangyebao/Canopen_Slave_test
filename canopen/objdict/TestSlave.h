
#ifndef __TESTSLAVE_H
#define __TESTSLAVE_H

#include "stm32f10x.h"
#include "canfestival.h"

/* 节点ID定义 */
#define NODE_ID 0x7F

/* 对象字典索引定义 */
#define INDEX_OBJ6040 0x6040
#define INDEX_OBJ6041 0x6041
#define INDEX_OBJ605A 0x605A
#define INDEX_OBJ605B 0x605B
#define INDEX_OBJ605C 0x605C
#define INDEX_OBJ605D 0x605D
#define INDEX_OBJ605E 0x605E
#define INDEX_OBJ6060 0x6060
#define INDEX_OBJ6061 0x6061
#define INDEX_OBJ6062 0x6062
#define INDEX_OBJ6064 0x6064
#define INDEX_OBJ6065 0x6065
#define INDEX_OBJ6066 0x6066
#define INDEX_OBJ606B 0x606B
#define INDEX_OBJ606C 0x606C
#define INDEX_OBJ6071 0x6071
#define INDEX_OBJ6072 0x6072
#define INDEX_OBJ6073 0x6073
#define INDEX_OBJ6074 0x6074
#define INDEX_OBJ6077 0x6077
#define INDEX_OBJ6078 0x6078
#define INDEX_OBJ607A 0x607A
#define INDEX_OBJ607D 0x607D
#define INDEX_OBJ607E 0x607E
#define INDEX_OBJ6080 0x6080
#define INDEX_OBJ6081 0x6081
#define INDEX_OBJ6083 0x6083
#define INDEX_OBJ6084 0x6084
#define INDEX_OBJ6085 0x6085
#define INDEX_OBJ6087 0x6087
#define INDEX_OBJ6098 0x6098
#define INDEX_OBJ6099 0x6099
#define INDEX_OBJ609A 0x609A
#define INDEX_OBJ60E0 0x60E0
#define INDEX_OBJ60E1 0x60E1
#define INDEX_OBJ60C0 0x60C0
#define INDEX_OBJ60C1 0x60C1
#define INDEX_OBJ60FF 0x60FF

/* PDO映射参数索引定义 */
#define INDEX_OBJ1600 0x1600  // RPDO1映射参数
#define INDEX_OBJ1601 0x1601  // RPDO2映射参数
#define INDEX_OBJ1602 0x1602  // RPDO3映射参数
#define INDEX_OBJ1A00 0x1A00  // TPDO1映射参数
#define INDEX_OBJ1A01 0x1A01  // TPDO2映射参数
#define INDEX_OBJ1A02 0x1A02  // TPDO3映射参数

/* 全局变量声明 */
extern CO_Data TestSlave_Data;
extern UNS8 TestSlave_bDeviceNodeId;
extern volatile uint8_t CANopenTimerFlag;

/* 函数声明 */
void TestSlave_init(void);
void TestSlave_loop(void);
void TestSlave_setNodeId(UNS8 nodeId);
UNS8 TestSlave_getNodeId(void);

/* CANopen回调函数声明 */
void heartbeatError(CO_Data* d, UNS8 heartbeatID);
void initialisation(CO_Data* d);
void preOperational(CO_Data* d);
void operational(CO_Data* d);
void stopped(CO_Data* d);
void post_sync(CO_Data* d);
void post_TPDO(CO_Data* d);
UNS32 storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex);
void post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5]);

#endif /* __TESTSLAVE_H */
