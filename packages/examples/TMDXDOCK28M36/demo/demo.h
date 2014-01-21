/*
 *  ======== demo.h ========
 */
 
#ifndef __TMDXDOCK28M36_DEMO_H
#define __TMDXDOCK28M36_DEMO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/ipc/MessageQ.h>

#define M3QUEUENAME "M3s_queue"
#define C28QUEUENAME "C28s_queue"

#define HEAPID      0

#define TEMPERATURE_CONVERSION 1

typedef struct TempMsg {
    MessageQ_MsgHeader hdr;
    Float temperatureF;
    Float temperatureC;
} TempMsg;

#define RECORDING_OPEN    1
#define RECORDING_CLOSE   2
#define RECORDING_WRITING 3
#define RECORDING_CLOSED  4

#ifdef __cplusplus
}
#endif

#endif /* __TMDXDOCK28M36_DEMO_H */
