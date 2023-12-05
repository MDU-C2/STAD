#pragma once

#include <stddef.h>

enum class UDP_Type {
    Server,
    Client
};

bool udp_create(UDP_Type type, void(*recv_callback)(void*, size_t));

void udp_send_msg(const void* buffer, size_t size);

void udp_destroy();