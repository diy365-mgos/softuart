/*
 * Copyright (c) 2023 DIY356
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MGOS_SOFTUART_H_
#define MGOS_SOFTUART_H_

#include <stdbool.h>
#include <stdlib.h>
#include "mgos_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Generic and opaque SoftUART instance */
struct mg_softuart;
typedef struct mg_softuart *mgos_softuart_t;

typedef void (*mgos_softuart_dispatcher_t)(mgos_softuart_t uart, void *arg);

mgos_softuart_t mgos_softuart_create(int rx_pin, enum mgos_gpio_pull_type rx_pin_pull,
                                     int tx_pin, struct mgos_uart_config* cfg);

bool mgos_softuart_config_get(mgos_softuart_t uart, struct mgos_uart_config *cfg);

void mgos_softuart_set_dispatcher(mgos_softuart_t uart, mgos_softuart_dispatcher_t cb, void *arg);

size_t mgos_softuart_read(mgos_softuart_t uart, void *buf, size_t len);
size_t mgos_softuart_read_mbuf(mgos_softuart_t uart, struct mbuf *mb, size_t len);

size_t mgos_softuart_read_avail(mgos_softuart_t uart);

void mgos_softuart_set_rx_enabled(mgos_softuart_t uart, bool enabled);

bool mgos_softuart_is_rx_enabled(mgos_softuart_t uart);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MGOS_SOFTUART_H_ */