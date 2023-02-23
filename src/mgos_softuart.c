#include "mgos.h"
#include "mgos_softuart.h"

#define MG_SOFTUART_MAX_COUNT 5
#define MG_SOFTUART_RX_TIMER_TIMEOUT 1 // milliseconds

static struct mg_softuart* s_uarts[MG_SOFTUART_MAX_COUNT];
static int8_t s_uarts_len = 0;
static mgos_timer_id s_rx_timer_id = MGOS_INVALID_TIMER_ID;

struct mg_softuart {
  int rx_pin;
  enum mgos_gpio_pull_type rx_pin_pull;
  bool rx_enabled;
  bool rx_started;
  struct mbuf rx_buf;
  int32_t rx_word_data; 
  int32_t rx_word_mask; 
  int8_t rx_word_bit;
  int64_t rx_first_int_us;
  int64_t rx_last_int_us;
  int64_t rx_timeout_us;
  int tx_pin;
  struct mbuf tx_buf;
  mgos_softuart_dispatcher_t dispatcher_cb;
  void* dispatcher_data;
  struct mgos_uart_config cfg;
  int64_t bit_duration_us;
};

bool mg_softuart_rx_try_dispatch_ex(struct mg_softuart* uart, int64_t uptime, bool gpio_val) {
  if (!uart->rx_started) return false;
  // save received bits into the rx_word_data
  int64_t last_dur = (uptime - uart->rx_last_int_us);
  int8_t num_bits = (last_dur / uart->bit_duration_us);
  for (; (num_bits > 0) && (uart->rx_word_bit < 32); --num_bits, ++uart->rx_word_bit) {
    uart->rx_word_data += gpio_val << uart->rx_word_bit;
  }

  bool dispatched = false;
  if ((uptime - uart->rx_first_int_us) >= uart->rx_timeout_us) {
    // read timeout detected
    if (gpio_val) {
      // STOP bit detected

      // TODO: check PARITY consinstency if the parity bit is present
    
      // remove start-bit
      uart->rx_word_data >>= 1;
      // remove parity and stop bits
      uart->rx_word_data &= uart->rx_word_mask;
      // append the received word to the buffer
      int8_t rx_data = uart->rx_word_data;
      mbuf_append(&uart->rx_buf, &rx_data, 1);
      LOG(LL_INFO, ("RX RECEIVED %ld", uart->rx_word_data));

      if (uart->dispatcher_cb) {
        uart->dispatcher_cb(uart, uart->dispatcher_data);
      }
      dispatched = true;
    } else {
      // something went wrong, and no STOP bit/s have been received,
      // so received bits must be abandoned.
      LOG(LL_ERROR, ("RX ABORTED"));
      dispatched = false;
    }

    uart->rx_started = false;
    uart->rx_word_data = 0;
    uart->rx_word_bit = 0;
    uart->rx_first_int_us = 0;
    uart->rx_last_int_us = 0;
    //LOG(LL_INFO, ("RX END"));
  }
  return dispatched;
}

bool mg_softuart_rx_try_dispatch(struct mg_softuart* uart, int64_t uptime) {
  return mg_softuart_rx_try_dispatch_ex(uart, uptime, mgos_gpio_read(uart->rx_pin));
}

static void mg_softuart_rx_timer_cb(void *arg) {
  int64_t uptime = mgos_uptime_micros();
  for (int8_t i = 0; i < s_uarts_len; ++i) {
    mg_softuart_rx_try_dispatch(s_uarts[i], uptime);
  }
  (void) arg;
}

void mgos_softuart_rx_int_handler(int pin, void *arg) {
  struct mg_softuart* uart = (struct mg_softuart*)arg;
  int64_t uptime = mgos_uptime_micros();
  bool gpio_val = mgos_gpio_read(uart->rx_pin);

  if (gpio_val) {
    // transition from LOW to HIGH
    if (!uart->rx_started) return;
  } else {
    // transition from HIGH to LOW
    if (!uart->rx_started) {
      // START bit detected 
      uart->rx_started = true;
      uart->rx_word_data = 0;
      uart->rx_word_bit = 0;
      uart->rx_first_int_us = uptime;
      uart->rx_last_int_us = uptime;
      return;
    }
  }

  if (!mg_softuart_rx_try_dispatch_ex(uart, uptime, gpio_val)) {
    uart->rx_last_int_us = uptime;
  }
}

mgos_softuart_t mgos_softuart_create(int rx_pin, enum mgos_gpio_pull_type rx_pin_pull,
                                     int tx_pin, struct mgos_uart_config* cfg) {
  if (!cfg) return NULL;
  if (s_uarts_len >= MG_SOFTUART_MAX_COUNT) return NULL;
  if (cfg->num_data_bits > 8) return NULL;

  struct mg_softuart* uart = calloc(1, sizeof(struct mg_softuart));
  if (!uart) return NULL;
  
  // copy the configuration
  memcpy(&uart->cfg, cfg, sizeof(*cfg));
  // set bit_duration (microseconds)
  uart->bit_duration_us = (1000000 / uart->cfg.baud_rate);
  // set word bit mask according word length (5-8 bits)
  uart->rx_word_mask = 0;
  for (int8_t i = 0; i < uart->cfg.num_data_bits; ++i) {
    uart->rx_word_mask += 1 << i;
  }
  // set rx_timeout_us (from the START bit to the PARITY bit)
  // START bit + num_data_bits + PARITY bit (optional)
  uart->rx_timeout_us = (uart->bit_duration_us * (1 + uart->cfg.num_data_bits + (uart->cfg.parity == MGOS_UART_PARITY_NONE ? 0 : 1)));

  // configure RX
  
  uart->rx_pin = rx_pin;
  uart->rx_pin_pull = rx_pin_pull;
  if (uart->rx_pin >= 0) {
    mbuf_init(&uart->rx_buf, uart->cfg.rx_buf_size);
    if (s_rx_timer_id == MGOS_INVALID_TIMER_ID) {
      s_rx_timer_id = mgos_set_timer(MG_SOFTUART_RX_TIMER_TIMEOUT, MGOS_TIMER_REPEAT,
                                     mg_softuart_rx_timer_cb, NULL);
    }
    bool gpio_ok = mgos_gpio_setup_input(uart->rx_pin, uart->rx_pin_pull);
    bool handler_ok = mgos_gpio_set_int_handler(uart->rx_pin, MGOS_GPIO_INT_EDGE_ANY,
                                                mgos_softuart_rx_int_handler, uart);
    if ((s_rx_timer_id == MGOS_INVALID_TIMER_ID) || !gpio_ok || !handler_ok) {
      if (s_uarts_len == 0 && s_rx_timer_id != MGOS_INVALID_TIMER_ID) {
        mgos_clear_timer(s_rx_timer_id);
        s_rx_timer_id = MGOS_INVALID_TIMER_ID;
      }
      mbuf_free(&uart->rx_buf);
      free(uart);
      if (!gpio_ok)
        LOG(LL_ERROR, ("Unable to set pin %d as RX", uart->tx_pin));
      if (!handler_ok)
        LOG(LL_ERROR, ("Unable to interrupt handler on pin %d", uart->tx_pin));
      if (s_rx_timer_id == MGOS_INVALID_TIMER_ID)
        LOG(LL_ERROR, ("Unable to start RX timer (every %dms)", MG_SOFTUART_RX_TIMER_TIMEOUT));
      return NULL;
    }
  } else {
    mbuf_init(&uart->rx_buf, 0);
  }

  // configure TX
  
  uart->tx_pin = tx_pin;
  if (uart->tx_pin >= 0) {
    mbuf_init(&uart->tx_buf, uart->cfg.tx_buf_size);
    if (!mgos_gpio_setup_output(uart->tx_pin, true)) {
      mbuf_free(&uart->tx_buf);
      free(uart);
      LOG(LL_ERROR, ("Unable to set pin %d as TX", uart->tx_pin));
      return NULL;
    }
  } else {
    mbuf_init(&uart->tx_buf, 0);
  }

  LOG(LL_INFO, ("SOFT-UART %d successfully created (TX=%d, RX=%d)", 
      s_uarts_len, uart->tx_pin, uart->rx_pin));

  s_uarts[s_uarts_len++] = uart;
  return uart;
}

bool mgos_softuart_config_get(mgos_softuart_t uart, struct mgos_uart_config *cfg) {
  if (!uart || !cfg) return false;
  /* A way of telling if the UART has been configured. */
  if (((struct mg_softuart *)uart)->cfg.rx_buf_size == 0 && 
      ((struct mg_softuart *)uart)->cfg.tx_buf_size == 0) return false;
  memcpy(cfg, &((struct mg_softuart *)uart)->cfg, sizeof(*cfg));
  return true;
}

void mgos_softuart_set_dispatcher(mgos_softuart_t uart, mgos_softuart_dispatcher_t cb, void *arg) {
  if (uart == NULL) return;
  ((struct mg_softuart *)uart)->dispatcher_cb = cb;
  ((struct mg_softuart *)uart)->dispatcher_data = arg;
}

size_t mgos_softuart_read(mgos_softuart_t uart, void *buf, size_t len) {
  if (uart == NULL || !((struct mg_softuart *)uart)->rx_enabled) return 0;
  size_t tr = MIN(len, ((struct mg_softuart *)uart)->rx_buf.len);
  memcpy(buf, ((struct mg_softuart *)uart)->rx_buf.buf, tr);
  mbuf_remove(&((struct mg_softuart *)uart)->rx_buf, tr);
  return tr;
}

size_t mgos_softuart_read_mbuf(mgos_softuart_t uart, struct mbuf *mb, size_t len) {
  if (uart == NULL || !((struct mg_softuart *)uart)->rx_enabled) return 0;
  size_t nr = MIN(len, mgos_softuart_read_avail(uart));
  if (nr > 0) {
    size_t free_bytes = mb->size - mb->len;
    if (free_bytes < nr) {
      mbuf_resize(mb, mb->len + nr);
    }
    nr = mgos_softuart_read(uart, mb->buf + mb->len, nr);
    mb->len += nr;
  }
  return nr;
}

size_t mgos_softuart_read_avail(mgos_softuart_t uart) {
  return (uart ? ((struct mg_softuart *)uart)->rx_buf.len : 0);
}

void mgos_softuart_set_rx_enabled(mgos_softuart_t uart, bool enabled) {
  if (uart == NULL) return;
  if (enabled) {
    if (!mgos_gpio_enable_int(uart->rx_pin)) return;
  } else {
    if (!mgos_gpio_disable_int(uart->rx_pin)) return;
  }
  ((struct mg_softuart *)uart)->rx_enabled = enabled;
}

bool mgos_softuart_is_rx_enabled(mgos_softuart_t uart) {
  return (uart ? ((struct mg_softuart *)uart)->rx_enabled : false);
}


bool mgos_softuart_init(void) {
  return true;
}