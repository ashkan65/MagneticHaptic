#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "common.h"

// Global done signal
extern volatile bool done;

// Send an ROI command
inline __attribute__((always_inline)) void sendROI(const int sock, const struct sockaddr *sa, volatile ROI_t *roi)  {
	// Set up the frame to send
	frame_t f;
	f.magic = htonl(CAM_MAGIC);
	f.roi.x = roi->x;
	f.roi.y = roi->y;
	f.roi.width = roi->width;
	f.roi.height = roi->height;

	// These packets are short and should always fit in the buffer
	ssize_t status = sendto(sock, &f, sizeof(frame_t), 0, sa, sizeof(struct sockaddr_in));
	if (status < 0) {
		printf("Unable to send ROI packet, error code: %zd - %s!\n", status, strerror(errno));
		exit(-1);
	} else if (status != sizeof(frame_t)) {
		printf("Unable to send full frame_t: %zd bytes of %zu\n", status, sizeof(frame_t));
	}
}

// Send a stop command
inline __attribute__((always_inline)) void sendStop(int sock, struct sockaddr *sa)  {
	// Set up the frame to send
	frame_t f;
	f.magic = htonl(CAM_MAGIC);
	memset(&(f.roi), 0, sizeof(ROI_t));

	// These packets are short and should always fit in the buffer
	ssize_t status = sendto(sock, &f, sizeof(frame_t), 0, sa, sizeof(struct sockaddr_in));
	if (status < 0) {
		printf("Unable to send STOP packet, error code: %zd - %s!\n", status, strerror(errno));
		exit(-1);
	} else if (status != sizeof(frame_t)) {
		printf("Unable to send full frame_t: %zd bytes of %zu\n", status, sizeof(frame_t));
	}
}

// Receive function
inline __attribute__((always_inline)) bool rx_frame(const int sock, const struct sockaddr_in *si_server, volatile ROI_t *roi, uint8_t *buffer)  {
	struct timeval to;
	// Receive the data
	bool rcvd = false;
	int to_rx = sizeof(frame_t), rxd = 0, rx_len, to_rx_now;
	socklen_t sock_len = sizeof(struct sockaddr);
	frame_t *f = (frame_t *)buffer;

	to_rx += roi->width * roi->height;

	while ((!rcvd) && (!done)) {
		fd_set readfds;
		FD_ZERO(&readfds);
		FD_SET(sock, &readfds);

		// Set up the time out timeval, we will wait for 3 seconds
		to.tv_sec = 3;
		to.tv_usec = 0;

		// Check to see if there's data available
		int status = select(sock + 1, &readfds, nullptr, nullptr, &to);
		if (-1 == status) {
			if (EINTR == errno)
				continue;
			printf("Select failed\n");
			exit(-1);
		// We timed out
		} else if (0 == status) {
			printf("RX Frame timeout, got %d bytes\n", rxd);
			return false;
		}

		// Recieve data...
		if (to_rx > MAX_UDP_SIZE)
			to_rx_now = MAX_UDP_SIZE;
		else
			to_rx_now = to_rx;

		rx_len = recvfrom(sock, &(buffer[rxd]), to_rx_now, 0, (struct sockaddr *)&si_server, &sock_len);
		if (rx_len < 0) {
			printf("Error receiving: %d!\n" , rx_len);
			exit(-1);
		}

		// Figure out how much more data we have to rx
		to_rx -= rx_len;
		rxd += rx_len;

		// If we have received the whole packet, we will exit the loop
		if (to_rx == 0) {
			// Verify that the packet has the correct magic, if not, reset the packet
			if (!(ntohl(f->magic) == CAM_MAGIC))  {
				printf("Packet has incorrect magic, expected 0xCEDAC0DE, got %X\n", ntohl(f->magic));
				return false;
			} else {
				rcvd = true;
			}
		}
	}
	return true;
}
