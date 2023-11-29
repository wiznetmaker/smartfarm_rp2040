#ifndef _LOOPBACK_H_
#define _LOOPBACK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Loopback test debug message printout enable */
#define	_LOOPBACK_DEBUG_

/* DATA_BUF_SIZE define for Loopback example */
#ifndef DATA_BUF_SIZE
	#define DATA_BUF_SIZE			2048
#endif

/************************/
/* Select LOOPBACK_MODE */
/************************/
#define LOOPBACK_MAIN_NOBLOCK    0
#define LOOPBACK_MODE   LOOPBACK_MAIN_NOBLOCK


static uint8_t ai_buf[2048];
static int send_cnt = 0;

/* TCP server Loopback test example */
int32_t tcps(uint8_t sn, uint8_t* buf, uint16_t port, uint16_t* out_temp, uint16_t* out_humid, uint16_t* out_brightness, uint16_t* water_flag);

/* TCP client Loopback test example */
int32_t loopback_tcpc(uint8_t sn, uint8_t* buf, uint8_t* destip, uint16_t destport);

/* UDP Loopback test example */
int32_t loopback_udps(uint8_t sn, uint8_t* buf, uint16_t port);

void parseAiBuf(const uint8_t *ai_buf, uint16_t *temp, uint16_t *humi, uint16_t *bright, uint16_t* wf);

#ifdef __cplusplus
}
#endif

#endif
