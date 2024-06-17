// Copyright 2024, rcelyte
// SPDX-License-Identifier: BSL-1.0

#include "websocket.h"
#include "os/os_time.h"
#include <assert.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <sched.h>

static bool
ValidateDigest(const char base64[28], const uint8_t key[16])
{
	const uint8_t map[0x80] = {
	    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  62, 0,  0,  0,  63, 52, 53, 54, 55,
	    56, 57, 58, 59, 60, 61, 0,  0,  0,  0,  0,  0,  0,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12,
	    13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 0,  0,  0,  0,  0,  0,  26, 27, 28, 29, 30, 31, 32,
	    33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 0,  0,  0,  0,  0,
	};
	assert(base64[27] == '=');
	const char *base64_it = base64;
	uint8_t digest[21], *digest_it = digest;
	while (base64_it < &base64[28]) {
		uint32_t word = 0;
		for (const char *end = &base64_it[4]; base64_it < end; ++base64_it) {
			word = word << 6 | map[*base64_it & 0x7f];
		}
		for (const uint8_t *end = &digest_it[2]; digest_it <= end; ++digest_it) {
			*digest_it = (uint8_t)(word >> (end - digest_it) * 8);
		}
	}
	uint8_t source[16 + 36];
	memcpy(source, key, 16);
	memcpy(&source[16], "258EAFA5-E914-47DA-95CA-C5AB0DC85B11", 36);
	uint32_t W[80], H[5] = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0};
	const uint64_t bitlen = ((uint64_t)sizeof(source)) * 8;
	const uint32_t loopcount = (sizeof(source) + 8) / 64 + 1, tailLen = 64 * loopcount - sizeof(source);
	uint8_t tail[128] = {0x80};
	tail[tailLen - 8] = (uint8_t)(bitlen >> 56);
	tail[tailLen - 7] = (uint8_t)(bitlen >> 48);
	tail[tailLen - 6] = (uint8_t)(bitlen >> 40);
	tail[tailLen - 5] = (uint8_t)(bitlen >> 32);
	tail[tailLen - 4] = (uint8_t)(bitlen >> 24);
	tail[tailLen - 3] = (uint8_t)(bitlen >> 16);
	tail[tailLen - 2] = (uint8_t)(bitlen >> 8);
	tail[tailLen - 1] = (uint8_t)(bitlen >> 0);
	for (uint32_t lidx = 0, didx = 0; lidx < loopcount; lidx++) {
		memset(W, 0, 80 * sizeof(uint32_t));
		for (uint32_t widx = 0; widx < 16; widx++) {
			int32_t wcount = 24;
			while (didx < sizeof(source) && wcount >= 0) {
				W[widx] += (((uint32_t)source[didx]) << wcount);
				didx++;
				wcount -= 8;
			}
			while (wcount >= 0) {
				W[widx] += (((uint32_t)tail[didx - sizeof(source)]) << wcount);
				didx++;
				wcount -= 8;
			}
		}
		for (uint32_t widx = 16; widx < 32; widx++) {
			const uint32_t value = W[widx - 3] ^ W[widx - 8] ^ W[widx - 14] ^ W[widx - 16];
			W[widx] = value << 1 | value >> 31;
		}
		for (uint32_t widx = 32; widx < 80; widx++) {
			const uint32_t value = W[widx - 6] ^ W[widx - 16] ^ W[widx - 28] ^ W[widx - 32];
			W[widx] = value << 2 | value >> 30;
		}
		uint32_t a = H[0], b = H[1], c = H[2], d = H[3], e = H[4], f = 0, k = 0;
		for (uint32_t stage = 0, idx = 0; stage < 4; ++stage) {
			for (uint32_t end = idx + 20; idx < end; ++idx) {
				// clang-format off
				switch (stage) {
					case 0: f = (b & c) | ((~b) & d); k = 0x5a827999; break;
					case 1: f = b ^ c ^ d; k = 0x6ed9eba1; break;
					case 2: f = (b & c) | (b & d) | (c & d); k = 0x8f1bbcdc; break;
					case 3: f = b ^ c ^ d; k = 0xca62c1d6; break;
				}
				// clang-format on
				const uint32_t temp = (a << 5 | a >> 27) + f + e + k + W[idx];
				e = d;
				d = c;
				c = b << 30 | b >> 2;
				b = a;
				a = temp;
			}
		}
		H[0] += a;
		H[1] += b;
		H[2] += c;
		H[3] += d;
		H[4] += e;
	}
	for (uint32_t i = 0; i < 20; ++i) {
		if (digest[i] != (uint8_t)(H[i / 4] >> (24 - (i % 4) * 8))) {
			return false;
		}
	}
	return true;
}

struct PacketHeader
{
	uint8_t opcode : 4, rsv3 : 1, rsv2 : 1, rsv1 : 1, fin : 1, len : 7, mask : 1, optional[12];
};

bool
WebSocket_init(struct WebSocket *const state, const enum u_logging_level log_level)
{
	state->sockfd = socket(AF_INET, SOCK_STREAM, 0);
	state->log_level = log_level;
	state->timestamp = (int64_t)os_monotonic_get_ns();
	state->opcode = 0;
	state->finished = true;
	state->end = state->head = state->buffer;
	memset(state->mask, 0, sizeof(state->mask));
	if (state->sockfd == -1) {
		U_LOG_IFL_E(log_level, "socket() failed");
		return false;
	}
	return true;
}

void
WebSocket_destroy(struct WebSocket *const state)
{
	const int sockfd = atomic_exchange(&state->sockfd, -1);
	if (sockfd == -1) {
		return;
	}
	shutdown(sockfd, SHUT_RDWR); // unblock `WebSocket_wait()`
	while (atomic_load(&state->sockfd_pin) != 0)
		sched_yield();
	close(sockfd);
}

bool
WebSocket_handshake(const struct WebSocket *const state)
{
	const struct sockaddr_in localhost = {AF_INET, htons(21110), {htonl(INADDR_LOOPBACK)}, {0}};
	if (connect(state->sockfd, (const struct sockaddr *)&localhost, sizeof(struct sockaddr_in)) != 0) {
		U_LOG_IFL_E(state->log_level, "connect() failed: %s", strerror(errno));
		return false;
	}
	uint8_t buffer[0x2000];
	uint32_t buffer_len = 0;
	uint8_t key[16];
	{
		__uint128_t keyValue = (__uint128_t)(unsigned)rand() | (__uint128_t)(unsigned)rand() << 32u |
		                       (__uint128_t)(unsigned)rand() << 64u | (__uint128_t)(unsigned)rand() << 96u;
		for (uint32_t i = 0; i < sizeof(key); ++i) {
			key[i] = (keyValue % 96) + 32;
			keyValue /= 96;
		}
		buffer_len = snprintf((char *)buffer, sizeof(buffer), "%s%.*s%s",
		                      "GET / HTTP/1.1\r\n"
		                      "Host: localhost:8080\r\n"
		                      "Connection: Upgrade\r\n"
		                      "Upgrade: websocket\r\n"
		                      "Sec-Websocket-Key: ",
		                      (int)sizeof(key), (const char *)key,
		                      "\r\n"
		                      "Sec-Websocket-Version: 13\r\n"
		                      "\r\n");
		assert(buffer_len == 145);
		if (send(state->sockfd, buffer, buffer_len, MSG_NOSIGNAL) != buffer_len) {
			U_LOG_IFL_E(state->log_level, "send() failed");
			return false;
		}
	}
	for (buffer_len = 0; buffer_len < 4 || memcmp(&buffer[buffer_len - 4], "\r\n\r\n", 4) != 0;) {
		const ssize_t fragmentLength =
		    recv(state->sockfd, &buffer[buffer_len], sizeof(buffer) - 1 - buffer_len, 0);
		if (fragmentLength <= 0) {
			U_LOG_IFL_E(state->log_level, "recv() failed: %s", strerror(errno));
			return false;
		}
		buffer_len += fragmentLength;
	}
	if (buffer_len < 40 || memcmp(buffer, "HTTP/1.1 101", 12) != 0) {
		U_LOG_IFL_E(state->log_level, "Bad HTTP response");
		return false;
	}
	buffer[buffer_len] = 0;
	char *acceptBase64 = strstr((char *)buffer, "\r\nSec-WebSocket-Accept: ");
	if (acceptBase64 == NULL) {
		U_LOG_IFL_E(state->log_level, "Missing HTTP 'Sec-WebSocket-Accept' header");
		return false;
	}
	acceptBase64 += 24;
	if (strstr(acceptBase64, "\r\n") != &acceptBase64[28] || !ValidateDigest(acceptBase64, key)) {
		U_LOG_IFL_E(state->log_level, "Bad WebSocket accept nonce");
		return false;
	}
	return true;
}

bool
WebSocket_wait(struct WebSocket *const state)
{
	atomic_fetch_add(&state->sockfd_pin, 1);
	struct pollfd sockfd = {atomic_load(&state->sockfd), POLLIN, 0};
	bool result = false;
	if (sockfd.fd != -1) {
		result = poll(&sockfd, 1, -1) != -1 || errno == EINTR;
	}
	atomic_fetch_sub(&state->sockfd_pin, 1);
	return result;
}

bool
WebSocket_sendWithOpcode(struct WebSocket *const state, uint8_t packet[], const size_t packet_len, const uint8_t opcode)
{
	atomic_fetch_add(&state->sockfd_pin, 1);
	const int sockfd = atomic_load(&state->sockfd);
	if (sockfd == -1) {
		atomic_fetch_sub(&state->sockfd_pin, 1);
		return false;
	}
	struct PacketHeader header = {
	    .opcode = opcode,
	    .fin = true,
	    .mask = true,
	};
	uint8_t payloadLength_len;
	if (packet_len < 126) {
		header.len = (uint8_t)packet_len;
		payloadLength_len = 0;
	} else if (packet_len < 0x10000) {
		header.len = 126;
		payloadLength_len = sizeof(uint16_t);
		header.optional[0] = (uint8_t)(packet_len >> 8);
		header.optional[1] = (uint8_t)packet_len;
	} else {
		header.len = 127;
		payloadLength_len = sizeof(uint64_t);
		header.optional[0] = (uint8_t)(packet_len >> 56);
		header.optional[1] = (uint8_t)(packet_len >> 48);
		header.optional[2] = (uint8_t)(packet_len >> 40);
		header.optional[3] = (uint8_t)(packet_len >> 32);
		header.optional[4] = (uint8_t)(packet_len >> 24);
		header.optional[5] = (uint8_t)(packet_len >> 16);
		header.optional[6] = (uint8_t)(packet_len >> 8);
		header.optional[7] = (uint8_t)packet_len;
	}
	uint8_t *mask = &header.optional[payloadLength_len];
	memcpy(mask, (uint32_t[1]){rand()}, sizeof(uint32_t));
	for (size_t i = 0; i < packet_len; ++i) {
		packet[i] ^= mask[i % sizeof(uint32_t)];
	}
	const ssize_t result = sendmsg(sockfd,
	                               &(const struct msghdr){
	                                   .msg_iov =
	                                       (struct iovec[2]){
	                                           {&header, 2 + payloadLength_len + sizeof(uint32_t)},
	                                           {packet, packet_len},
	                                       },
	                                   .msg_iovlen = 2,
	                               },
	                               MSG_NOSIGNAL);
	atomic_fetch_sub(&state->sockfd_pin, 1);
	return ((size_t)result == 2 + payloadLength_len + sizeof(uint32_t) + packet_len);
}

size_t
WebSocket_receive(struct WebSocket *const state)
{
	atomic_fetch_add(&state->sockfd_pin, 1);
	const int sockfd = atomic_load(&state->sockfd);
	if (sockfd == -1) {
		atomic_fetch_sub(&state->sockfd_pin, 1);
		return 0;
	}
	while (true) {
		if (state->head == state->end) {
			struct PacketHeader header = {0};
			ssize_t length = recv(sockfd, &header, sizeof(header), MSG_PEEK | MSG_DONTWAIT);
			const uint32_t payloadLength_len =
			    sizeof(uint16_t) * (header.len == 126) + sizeof(uint64_t) * (header.len == 127);
			const uint32_t header_len = 2 + payloadLength_len + sizeof(uint32_t) * header.mask;
			if (length < 0 && errno != EAGAIN) {
				U_LOG_IFL_E(state->log_level, "recv() failed: %s", strerror(errno));
				break;
			}
			if (length < header_len) {
				atomic_fetch_sub(&state->sockfd_pin, 1);
				return 0;
			}
			length = recv(sockfd, &header, header_len, MSG_DONTWAIT);
			if (length != header_len) {
				U_LOG_IFL_E(state->log_level, "recv() failed: %s",
				            (length < 0) ? strerror(errno) : "bad length");
				break;
			}
			const uint64_t fragmentLength =
			    (header.len < 126) ? (uint64_t)header.len
			    : (header.len == 126)
			        ? (uint64_t)header.optional[0] << 8 | (uint64_t)header.optional[1]
			        : (uint64_t)header.optional[0] << 56 | (uint64_t)header.optional[1] << 48 |
			              (uint64_t)header.optional[2] << 40 | (uint64_t)header.optional[3] << 32 |
			              (uint64_t)header.optional[4] << 24 | (uint64_t)header.optional[5] << 16 |
			              (uint64_t)header.optional[6] << 8 | (uint64_t)header.optional[7];
			if (fragmentLength > (uint64_t)(size_t)(&state->buffer[sizeof(state->buffer)] - state->end)) {
				U_LOG_IFL_E(state->log_level, "Packet too large");
				break;
			}
			if (state->finished) {
				state->timestamp = (int64_t)os_monotonic_get_ns();
				state->opcode = header.opcode;
				state->end = state->head = state->buffer;
			}
			if (!header.mask) {
				memset(state->mask, 0, sizeof(state->mask));
			} else {
				for (size_t i = 0, shift = (size_t)(state->head - state->buffer); i < 4; ++i) {
					state->mask[(shift + i) % sizeof(state->mask)] =
					    header.optional[payloadLength_len + i];
				}
			}
			state->finished = header.fin;
			state->end += fragmentLength;
		} else {
			const ssize_t length =
			    recv(sockfd, state->head, (size_t)(state->end - state->head), MSG_DONTWAIT);
			if (length < 0 && errno != EAGAIN) {
				U_LOG_IFL_E(state->log_level, "recv() failed: %s", strerror(errno));
				break;
			}
			if (length <= 0) {
				atomic_fetch_sub(&state->sockfd_pin, 1);
				return 0;
			}
			if (memcmp(state->mask, (const uint8_t[sizeof(state->mask)]){0}, sizeof(state->mask)) != 0) {
				for (size_t i = 0, shift = (size_t)(state->head - state->buffer); i < (size_t)length;
				     ++i) {
					state->head[i] ^= state->mask[(shift + i) % sizeof(state->mask)];
				}
			}
			state->head += (size_t)length;
		}
		if (state->head == state->end && state->finished) {
			switch (state->opcode) {
			case 0x1:
				U_LOG_IFL_D(state->log_level, "TEXT - %.*s",
				            (unsigned)(size_t)(state->head - state->buffer), state->buffer);
				break;
			case 0x2: {
				atomic_fetch_sub(&state->sockfd_pin, 1);
				return (size_t)(state->head - state->buffer);
			}
			case 0x8: goto fail;
			case 0x9:
				if (!WebSocket_sendWithOpcode(state, state->buffer,
				                              (size_t)(state->head - state->buffer), 0xa)) {
					U_LOG_IFL_E(state->log_level, "WebSocket_sendWithOpcode(pong) failed");
					goto fail;
				}
			case 0xa: break;
			default: {
				U_LOG_IFL_E(state->log_level, "Unrecognized opcode: 0x%x", state->opcode);
				goto fail;
			}
			}
		}
	}
fail:
	atomic_fetch_sub(&state->sockfd_pin, 1);
	WebSocket_destroy(state);
	return 0;
}
