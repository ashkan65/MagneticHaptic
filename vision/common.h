#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// ROI Dimensions
#define ROI_WIDTH	152
#define ROI_HEIGHT	152

#define L_ROI_WIDTH	 252
#define L_ROI_HEIGHT 252

// Full Image Dimensions
#define IMG_WIDTH	4096
#define IMG_HEIGHT	3072

// The port we'll use
#define UDP_PORT	2368

// The magic code
#define CAM_MAGIC	(0xCEDAC0DE)

// The maximum size of the UDP Packets we're sending
// IP max len - IP header - UDP header
#define MAX_UDP_SIZE	(65535 - 20 - 8)

// ROI Type
typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t x;
	uint16_t y;
} ROI_t;

typedef struct {
	// Magic code
	uint32_t magic;
	// ROI Dimensions
	ROI_t roi;
} frame_t;

// Frame functions
void sendROI(const int sock, const struct sockaddr *sa, volatile ROI_t *roi);
void sendStop(int sock, struct sockaddr *sa);
bool rx_frame(const int sock, const struct sockaddr_in *si_server, volatile ROI_t *roi, uint8_t *buffer);

#endif
