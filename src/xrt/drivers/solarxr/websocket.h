// Copyright 2024, rcelyte
// SPDX-License-Identifier: BSL-1.0

#pragma once
#include "util/u_logging.h"

struct WebSocket
{
	_Atomic(int) sockfd;
	_Atomic(uint32_t) sockfd_pin;
	enum u_logging_level log_level;
	int64_t timestamp;
	uint8_t opcode;
	bool finished;
	uint8_t *head, *end, mask[4], buffer[0x8000];
};

bool
WebSocket_init(struct WebSocket *state, enum u_logging_level log_level);
void
WebSocket_destroy(struct WebSocket *state); // threadsafe
bool
WebSocket_handshake(const struct WebSocket *state);
bool
WebSocket_wait(struct WebSocket *state);
bool
WebSocket_sendWithOpcode(struct WebSocket *state, uint8_t packet[], size_t packet_len, uint8_t opcode); // threadsafe
size_t
WebSocket_receive(struct WebSocket *state);

static inline bool
WebSocket_send(struct WebSocket *const state, uint8_t packet[], const size_t packet_len)
{
	return WebSocket_sendWithOpcode(state, packet, packet_len, 0x2);
}
