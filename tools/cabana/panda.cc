#include "tools/cabana/panda.h"

#include <unistd.h>
#include <cassert>
#include <stdexcept>
#include <vector>
#include <memory>

#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/util.h"

static libusb_context *init_usb_ctx() {
  libusb_context *context = nullptr;
  int err = libusb_init(&context);
  if (err != 0) {
    LOGE("libusb initialization error");
    return nullptr;
  }

#if LIBUSB_API_VERSION >= 0x01000106
  libusb_set_option(context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
#else
  libusb_set_debug(context, 3);
#endif
  return context;
}

Panda::Panda(std::string serial, uint32_t bus_offset) : bus_offset(bus_offset) {
  if (!init_usb_connection(serial)) {
    throw std::runtime_error("Error connecting to panda");
  }

  LOGW("connected to %s over USB", serial.c_str());
  hw_type = get_hw_type();
  can_reset_communications();
}

Panda::~Panda() {
  cleanup_usb();
}

bool Panda::connected() {
  return connected_flag;
}

bool Panda::comms_healthy() {
  return comms_healthy_flag;
}

std::string Panda::hw_serial() {
  return hw_serial_str;
}

bool Panda::is_flexray() {
  return util::starts_with(hw_serial_str, PICO_FLEXRAY_DONGLE_ID_PREFIX);
}

std::vector<std::string> Panda::list(bool usb_only) {
  static std::unique_ptr<libusb_context, decltype(&libusb_exit)> context(init_usb_ctx(), libusb_exit);

  ssize_t num_devices;
  libusb_device **dev_list = NULL;
  std::vector<std::string> serials;
  if (!context) { return serials; }

  num_devices = libusb_get_device_list(context.get(), &dev_list);
  if (num_devices < 0) {
    LOGE("libusb can't get device list");
    goto finish;
  }

  for (size_t i = 0; i < num_devices; ++i) {
    libusb_device *device = dev_list[i];
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(device, &desc);
    if (desc.idVendor == 0x3801 && desc.idProduct == 0xddcc) {
      libusb_device_handle *handle = NULL;
      int ret = libusb_open(device, &handle);
      if (ret < 0) { goto finish; }

      unsigned char desc_serial[26] = { 0 };
      ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      libusb_close(handle);
      if (ret < 0) { goto finish; }

      serials.push_back(std::string((char *)desc_serial, ret));
    }
  }

finish:
  if (dev_list != NULL) {
    libusb_free_device_list(dev_list, 1);
  }
  return serials;
}

void Panda::set_safety_model(cereal::CarParams::SafetyModel safety_model, uint16_t safety_param) {
  control_write(0xdc, (uint16_t)safety_model, safety_param);
}


cereal::PandaState::PandaType Panda::get_hw_type() {
  unsigned char hw_query[1] = {0};

  control_read(0xc1, 0, 0, hw_query, 1);
  return (cereal::PandaState::PandaType)(hw_query[0]);
}




void Panda::send_heartbeat(bool engaged) {
  control_write(0xf3, engaged, 0);
}

void Panda::set_can_speed_kbps(uint16_t bus, uint16_t speed) {
  control_write(0xde, bus, (speed * 10));
}


void Panda::set_data_speed_kbps(uint16_t bus, uint16_t speed) {
  control_write(0xf9, bus, (speed * 10));
}



bool Panda::can_receive(std::vector<can_frame>& out_vec) {
  // Check if enough space left in buffer to store RECV_SIZE data
  assert(receive_buffer_size + RECV_SIZE <= sizeof(receive_buffer));

  int recv = bulk_read(0x81, &receive_buffer[receive_buffer_size], RECV_SIZE);
  if (!comms_healthy()) {
    return false;
  }

  bool ret = true;
  if (recv > 0) {
    receive_buffer_size += recv;
    if (is_flexray()) {
      ret = unpack_flexray_buffer(receive_buffer, receive_buffer_size, out_vec);
    } else {
      ret = unpack_can_buffer(receive_buffer, receive_buffer_size, out_vec);
    }
  }
  return ret;
}

void Panda::can_reset_communications() {
  control_write(0xc0, 0, 0);
}

bool Panda::unpack_can_buffer(uint8_t *data, uint32_t &size, std::vector<can_frame> &out_vec) {
  int pos = 0;

  while (pos <= size - sizeof(can_header)) {
    can_header header;
    memcpy(&header, &data[pos], sizeof(can_header));

    const uint8_t data_len = dlc_to_len[header.data_len_code];
    if (pos + sizeof(can_header) + data_len > size) {
      // we don't have all the data for this message yet
      break;
    }

    if (calculate_checksum(&data[pos], sizeof(can_header) + data_len) != 0) {
      LOGE("Panda CAN checksum failed");
      size = 0;
      can_reset_communications();
      return false;
    }

    can_frame &canData = out_vec.emplace_back();
    canData.address = header.addr;
    canData.src = header.bus + bus_offset;
    if (header.rejected) {
      canData.src += CAN_REJECTED_BUS_OFFSET;
    }
    if (header.returned) {
      canData.src += CAN_RETURNED_BUS_OFFSET;
    }

    canData.dat.assign((char *)&data[pos + sizeof(can_header)], data_len);

    pos += sizeof(can_header) + data_len;
  }

  // move the overflowing data to the beginning of the buffer for the next round
  memmove(data, &data[pos], size - pos);
  size -= pos;

  return true;
}

uint8_t Panda::calculate_checksum(uint8_t *data, uint32_t len) {
  uint8_t checksum = 0U;
  for (uint32_t i = 0U; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// USB implementation methods
bool Panda::init_usb_connection(const std::string& serial) {
  ssize_t num_devices;
  libusb_device **dev_list = NULL;
  int err = 0;

  ctx = init_usb_ctx();
  if (!ctx) { goto fail; }

  // connect by serial
  num_devices = libusb_get_device_list(ctx, &dev_list);
  if (num_devices < 0) { goto fail; }

  for (size_t i = 0; i < num_devices; ++i) {
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev_list[i], &desc);
    if (desc.idVendor == 0x3801 && desc.idProduct == 0xddcc) {
      int ret = libusb_open(dev_list[i], &dev_handle);
      if (dev_handle == NULL || ret < 0) { goto fail; }

      unsigned char desc_serial[26] = { 0 };
      ret = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      if (ret < 0) { goto fail; }

      hw_serial_str = std::string((char *)desc_serial, ret);
      if (serial.empty() || serial == hw_serial_str) {
        break;
      }
      libusb_close(dev_handle);
      dev_handle = NULL;
    }
  }
  if (dev_handle == NULL) goto fail;
  libusb_free_device_list(dev_list, 1);

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    libusb_detach_kernel_driver(dev_handle, 0);
  }

  err = libusb_set_configuration(dev_handle, 1);
  if (err != 0) { goto fail; }

  err = libusb_claim_interface(dev_handle, 0);
  if (err != 0) { goto fail; }

  return true;

fail:
  if (dev_list != NULL) {
    libusb_free_device_list(dev_list, 1);
  }
  cleanup_usb();
  return false;
}

void Panda::cleanup_usb() {
  if (dev_handle) {
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    dev_handle = nullptr;
  }

  if (ctx) {
    libusb_exit(ctx);
    ctx = nullptr;
  }
  connected_flag = false;
}

void Panda::handle_usb_issue(int err, const char func[]) {
  LOGE_100("usb error %d \"%s\" in %s", err, libusb_strerror((enum libusb_error)err), func);
  if (err == LIBUSB_ERROR_NO_DEVICE) {
    LOGE("lost connection");
    connected_flag = false;
  }
}

int Panda::control_write(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned int timeout) {
  int err;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;

  if (!connected_flag) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, NULL, 0, timeout);
    if (err < 0) handle_usb_issue(err, __func__);
  } while (err < 0 && connected_flag);

  return err;
}

int Panda::control_read(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout) {
  int err;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;

  if (!connected_flag) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout);
    if (err < 0) handle_usb_issue(err, __func__);
  } while (err < 0 && connected_flag);

  return err;
}

int Panda::bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected_flag) {
    return 0;
  }

  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
    if (err == LIBUSB_ERROR_TIMEOUT) {
      LOGW("Transmit buffer full");
      break;
    } else if (err != 0 || length != transferred) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected_flag);

  return transferred;
}

int Panda::bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected_flag) {
    return 0;
  }

  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
    if (err == LIBUSB_ERROR_TIMEOUT) {
      break; // timeout is okay to exit, recv still happened
    } else if (err == LIBUSB_ERROR_OVERFLOW) {
      comms_healthy_flag = false;
      LOGE_100("overflow got 0x%x", transferred);
    } else if (err != 0) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected_flag);

  return transferred;
}

static uint32_t calculate_flexray_header_crc(const flexray_frame_t &frame) {
  uint32_t data_word = 0;
  data_word |= (uint32_t)frame.payload_length_words;
  data_word |= (uint32_t)frame.frame_id << 7;
  data_word |= (uint32_t)frame.indicators << (7+11+0);

  uint32_t crc = 0x1A;
  const uint32_t poly = 0x385;

  for (int i = 19; i >= 0; --i) {
    bool data_bit = (data_word >> i) & 1;
    bool crc_msb = (crc >> 10) & 1;

    crc <<= 1;
    if (data_bit ^ crc_msb) {
      crc ^= poly;
    }
  }

  return crc & 0x7FF;
}

static uint32_t calculate_flexray_payload_crc(const flexray_frame_t &frame) {
  uint32_t payload_bits = frame.payload_length_words * 16;
  uint32_t total_bits_to_crc = 40 + payload_bits;

  uint32_t crc = 0xFEDCBA;
  const uint32_t poly = 0x5D6DCB;

  auto get_bit = [&](int bit_pos) {
    int byte_pos = bit_pos / 8;
    int bit_in_byte = 7 - (bit_pos % 8);

    if (byte_pos < 5) {
      uint8_t header_buf[5];
      header_buf[0] =
        (frame.indicators << 3) |
        ((frame.frame_id >> 8) & 0b111);
      header_buf[1] = frame.frame_id & 0xFF;
      header_buf[2] = ((frame.payload_length_words << 1) | ((frame.header_crc >> 10) & 0b1));
      header_buf[3] = (frame.header_crc >> 2) & 0xFF;
      header_buf[4] = ((frame.header_crc & 0b11) << 6) | (frame.cycle_count & 0b111111);
      return (header_buf[byte_pos] >> bit_in_byte) & 1;
    } else {
      return (frame.payload[byte_pos - 5] >> bit_in_byte) & 1;
    }
  };

  for (uint32_t i = 0; i < total_bits_to_crc; ++i) {
    bool data_bit = get_bit(i);
    bool crc_msb = (crc >> 23) & 1;
    crc <<= 1;
    if (data_bit ^ crc_msb) {
      crc ^= poly;
    }
  }

  return crc & 0xFFFFFF;
}

bool Panda::unpack_flexray_buffer(uint8_t *data, uint32_t &size, std::vector<can_frame> &out_vec) {
  int pos = 0;
  // New variable-length format per USB stream:
  // [len_lo][len_hi] | [source:1] [header:5] [payload:N] [crc24_be:3]
  // where len = 1 + 5 + N + 3, and N == payload_length_words * 2
  while (pos + 2 <= (int)size) {
    uint16_t body_len = (uint16_t)(data[pos] | (data[pos + 1] << 8));
    uint32_t record_len = (uint32_t)body_len + 2U; // include length field itself

    // Basic sanity on body_len
    if (body_len < (uint16_t)(1 + 5 + 3) || body_len > (uint16_t)(1 + 5 + MAX_FRAME_PAYLOAD_BYTES + 3)) {
      // Invalid length, attempt resync by skipping one byte
      pos += 1;
      continue;
    }

    if (pos + (int)record_len > (int)size) {
      // Partial record; wait for more data
      break;
    }

    const uint8_t *rec = &data[pos + 2];
    uint8_t source = rec[0];
    const uint8_t *hdr = &rec[1]; // 5 bytes
    // Extract header fields
    uint8_t byte0 = hdr[0];
    uint8_t byte1 = hdr[1];
    uint8_t byte2 = hdr[2];
    uint8_t byte3 = hdr[3];
    uint8_t byte4 = hdr[4];

    flexray_frame_t frame = {};
    frame.source = source;
    frame.indicators = (uint8_t)(byte0 >> 3);
    frame.frame_id = (uint16_t)(((byte0 & 0x07) << 8) | byte1);
    frame.payload_length_words = (uint8_t)(byte2 >> 1);
    frame.header_crc = (uint16_t)(((uint16_t)(byte2 & 0x01) << 10) | ((uint16_t)byte3 << 2) | ((byte4 >> 6) & 0x03));
    frame.cycle_count = (uint8_t)(byte4 & 0x3F);

    uint16_t expected_payload_bytes = (uint16_t)frame.payload_length_words * 2U;
    // Compute actual payload bytes from body length
    uint16_t actual_payload_bytes = (uint16_t)(body_len - (uint16_t)(1 + 5 + 3));

    bool length_ok = (actual_payload_bytes == expected_payload_bytes) && (expected_payload_bytes <= MAX_FRAME_PAYLOAD_BYTES);

    const uint8_t *payload_ptr = &rec[1 + 5];
    const uint8_t *crc_ptr = &payload_ptr[actual_payload_bytes];
    uint32_t crc_stream = ((uint32_t)crc_ptr[0] << 16) | ((uint32_t)crc_ptr[1] << 8) | (uint32_t)crc_ptr[2];

    if (!length_ok) {
      // Skip malformed record
      pos += 1;
      continue;
    }

    if (expected_payload_bytes > 0) {
      memcpy(frame.payload, payload_ptr, expected_payload_bytes);
    }

    // Validate header CRC and payload CRC
    bool header_crc_ok = (calculate_flexray_header_crc(frame) == frame.header_crc);
    uint32_t payload_crc = calculate_flexray_payload_crc(frame) & 0xFFFFFFu;
    bool payload_crc_ok = (payload_crc == crc_stream);

    if (header_crc_ok && payload_crc_ok) {
      can_frame &canData = out_vec.emplace_back();
      canData.address = frame.frame_id;
      canData.src = frame.source + bus_offset;
      size_t payload_len = std::min((size_t)expected_payload_bytes, sizeof(frame.payload));
      canData.dat.assign(1, frame.cycle_count);
      canData.dat.append(frame.payload, payload_len);
    }

    // advance to next record
    pos += record_len;
  }

  // move remaining bytes (partial record) to start of buffer for next read
  memmove(data, &data[pos], size - pos);
  size -= pos;

  return true;
}
