#include "selfdrive/pandad/panda.h"

#include <unistd.h>

#include <cassert>
#include <stdexcept>
#include <vector>

#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/util.h"

const bool PANDAD_MAXOUT = getenv("PANDAD_MAXOUT") != nullptr;

Panda::Panda(std::string serial, uint32_t bus_offset) : bus_offset(bus_offset) {
  // try USB first, then SPI
  try {
    handle = std::make_unique<PandaUsbHandle>(serial);
    LOGW("connected to %s over USB", serial.c_str());
  } catch (std::exception &e) {
#ifndef __APPLE__
    handle = std::make_unique<PandaSpiHandle>(serial);
    LOGW("connected to %s over SPI", serial.c_str());
#else
    throw e;
#endif
  }
  if (util::starts_with(serial, PICO_FLEXRAY_DONGLE_ID_PREFIX)) {
    handle->flexray = true;
  }

  hw_type = get_hw_type();
  can_reset_communications();
}

bool Panda::connected() {
  return handle->connected;
}

bool Panda::comms_healthy() {
  return handle->comms_healthy;
}

bool Panda::is_flexray() {
  return handle->flexray;
}

std::string Panda::hw_serial() {
  return handle->hw_serial;
}

std::vector<std::string> Panda::list(bool usb_only) {
  std::vector<std::string> serials = PandaUsbHandle::list();

#ifndef __APPLE__
  if (!usb_only) {
    for (const auto &s : PandaSpiHandle::list()) {
      if (std::find(serials.begin(), serials.end(), s) == serials.end()) {
        serials.push_back(s);
      }
    }
  }
#endif

  return serials;
}

void Panda::set_safety_model(cereal::CarParams::SafetyModel safety_model, uint16_t safety_param) {
  handle->control_write(0xdc, (uint16_t)safety_model, safety_param);
}

void Panda::set_alternative_experience(uint16_t alternative_experience, uint16_t safety_param_sp) {
  handle->control_write(0xdf, alternative_experience, safety_param_sp);
}

std::string Panda::serial_read(int port_number) {
  std::string ret;
  char buffer[USBPACKET_MAX_SIZE] = {};

  while (true) {
    int bytes_read = handle->control_read(0xe0, port_number, 0, (unsigned char *)buffer, USBPACKET_MAX_SIZE);
    if (bytes_read <= 0) {
      break;
    }
    ret.append(buffer, bytes_read);
  }

  return ret;
}

void Panda::set_uart_baud(int uart, int rate) {
  handle->control_write(0xe4, uart, int(rate / 300));
}

cereal::PandaState::PandaType Panda::get_hw_type() {
  unsigned char hw_query[1] = {0};

  handle->control_read(0xc1, 0, 0, hw_query, 1);
  return (cereal::PandaState::PandaType)(hw_query[0]);
}

void Panda::set_fan_speed(uint16_t fan_speed) {
  handle->control_write(0xb1, fan_speed, 0);
}

uint16_t Panda::get_fan_speed() {
  uint16_t fan_speed_rpm = 0;
  handle->control_read(0xb2, 0, 0, (unsigned char*)&fan_speed_rpm, sizeof(fan_speed_rpm));
  return fan_speed_rpm;
}

void Panda::set_ir_pwr(uint16_t ir_pwr) {
  handle->control_write(0xb0, ir_pwr, 0);
}

std::optional<health_t> Panda::get_state() {
  health_t health {0};
  int err = handle->control_read(0xd2, 0, 0, (unsigned char*)&health, sizeof(health));
  return err >= 0 ? std::make_optional(health) : std::nullopt;
}

std::optional<can_health_t> Panda::get_can_state(uint16_t can_number) {
  can_health_t can_health {0};
  int err = handle->control_read(0xc2, can_number, 0, (unsigned char*)&can_health, sizeof(can_health));
  return err >= 0 ? std::make_optional(can_health) : std::nullopt;
}

void Panda::set_loopback(bool loopback) {
  handle->control_write(0xe5, loopback, 0);
}

std::optional<std::vector<uint8_t>> Panda::get_firmware_version() {
  std::vector<uint8_t> fw_sig_buf(128);
  int read_1 = handle->control_read(0xd3, 0, 0, &fw_sig_buf[0], 64);
  int read_2 = handle->control_read(0xd4, 0, 0, &fw_sig_buf[64], 64);
  return ((read_1 == 64) && (read_2 == 64)) ? std::make_optional(fw_sig_buf) : std::nullopt;
}

std::optional<std::string> Panda::get_serial() {
  char serial_buf[17] = {'\0'};
  int err = handle->control_read(0xd0, 0, 0, (uint8_t*)serial_buf, 16);
  return err >= 0 ? std::make_optional(serial_buf) : std::nullopt;
}

bool Panda::up_to_date() {
  if (auto fw_sig = get_firmware_version()) {
    for (auto fn : { "panda.bin.signed", "panda_h7.bin.signed" }) {
      auto content = util::read_file(std::string("../../panda/board/obj/") + fn);
      if (content.size() >= fw_sig->size() &&
          memcmp(content.data() + content.size() - fw_sig->size(), fw_sig->data(), fw_sig->size()) == 0) {
        return true;
      }
    }
  }
  return false;
}

void Panda::set_power_saving(bool power_saving) {
  handle->control_write(0xe7, power_saving, 0);
}

void Panda::enable_deepsleep() {
  handle->control_write(0xfb, 0, 0);
}

void Panda::send_heartbeat(bool engaged, bool engaged_mads) {
  handle->control_write(0xf3, engaged, engaged_mads);
}

void Panda::set_can_speed_kbps(uint16_t bus, uint16_t speed) {
  handle->control_write(0xde, bus, (speed * 10));
}

void Panda::set_can_fd_auto(uint16_t bus, bool enabled) {
  handle->control_write(0xe8, bus, enabled);
}

void Panda::set_data_speed_kbps(uint16_t bus, uint16_t speed) {
  handle->control_write(0xf9, bus, (speed * 10));
}

void Panda::set_canfd_non_iso(uint16_t bus, bool non_iso) {
  handle->control_write(0xfc, bus, non_iso);
}

static uint8_t len_to_dlc(uint8_t len) {
  if (len <= 8) {
    return len;
  }
  if (len <= 24) {
    return 8 + ((len - 8) / 4) + ((len % 4) ? 1 : 0);
  } else {
    return 11 + (len / 16) + ((len % 16) ? 1 : 0);
  }
}

void Panda::pack_can_buffer(const capnp::List<cereal::CanData>::Reader &can_data_list,
                            std::function<void(uint8_t *, size_t)> write_func) {
  int32_t pos = 0;
  uint8_t send_buf[2 * USB_TX_SOFT_LIMIT];

  for (const auto &cmsg : can_data_list) {
    // check if the message is intended for this panda
    uint8_t bus = cmsg.getSrc();
    if (bus < bus_offset || bus >= (bus_offset + PANDA_BUS_OFFSET)) {
      continue;
    }
    auto can_data = cmsg.getDat();
    uint8_t data_len_code = len_to_dlc(can_data.size());
    assert(can_data.size() <= 64);
    assert(can_data.size() == dlc_to_len[data_len_code]);

    can_header header = {};
    header.addr = cmsg.getAddress();
    header.extended = (cmsg.getAddress() >= 0x800) ? 1 : 0;
    header.data_len_code = data_len_code;
    header.bus = bus - bus_offset;
    header.checksum = 0;

    memcpy(&send_buf[pos], (uint8_t *)&header, sizeof(can_header));
    memcpy(&send_buf[pos + sizeof(can_header)], (uint8_t *)can_data.begin(), can_data.size());
    uint32_t msg_size = sizeof(can_header) + can_data.size();

    // set checksum
    ((can_header *) &send_buf[pos])->checksum = calculate_checksum(&send_buf[pos], msg_size);

    pos += msg_size;

    if (pos >= USB_TX_SOFT_LIMIT) {
      write_func(send_buf, pos);
      pos = 0;
    }
  }

  // send remaining packets
  if (pos > 0) write_func(send_buf, pos);
}

void Panda::can_send(const capnp::List<cereal::CanData>::Reader &can_data_list) {
  if (is_flexray()) {
    pack_flexray_buffer(can_data_list, [=](uint8_t* data, size_t size) {
      handle->bulk_write(3, data, size, 5);
    });
  } else {
    pack_can_buffer(can_data_list, [=](uint8_t* data, size_t size) {
      handle->bulk_write(3, data, size, 5);
    });
  }
}

bool Panda::can_receive(std::vector<can_frame>& out_vec) {
  // Check if enough space left in buffer to store RECV_SIZE data
  assert(receive_buffer_size + RECV_SIZE <= sizeof(receive_buffer));

  int recv = handle->bulk_read(0x81, &receive_buffer[receive_buffer_size], RECV_SIZE);
  if (!comms_healthy()) {
    return false;
  }

  if (PANDAD_MAXOUT) {
    static uint8_t junk[RECV_SIZE];
    handle->bulk_read(0xab, junk, RECV_SIZE - recv);
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
  handle->control_write(0xc0, 0, 0);
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

// CRC-8 SAE J1850 Lookup Table (Poly: 0x1D)
static uint8_t flexray_crc8_table[256] = {
  0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
  0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
  0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
  0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
  0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
  0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
  0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
  0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
  0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
  0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
  0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
  0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
  0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
  0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
  0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
  0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4,
};

static uint8_t calculate_crc8(const uint8_t *p, uint8_t init_value, uint8_t len) {
  uint8_t crc = init_value;
  for (uint8_t i = 0; i < len; i++) {
    crc = flexray_crc8_table[crc ^ p[i]];
  }
  return crc;
}

void Panda::pack_flexray_buffer(const capnp::List<cereal::CanData>::Reader &can_data_list,
                                std::function<void(uint8_t *, size_t)> write_func) {
  int pos = 0;
  uint8_t send_buf[2 * USB_TX_SOFT_LIMIT];

  for (const auto &cmsg : can_data_list) {
    auto dat = cmsg.getDat();
    if (dat.size() < 1) {
      continue;
    }

    uint8_t bus = cmsg.getSrc();
    if (bus < bus_offset || bus >= (bus_offset + PANDA_BUS_OFFSET)) {
      continue;
    }

    uint16_t frame_id = (uint16_t)(cmsg.getAddress() & 0b11111111111);
    uint8_t base = dat[0];
    uint32_t payload_bytes = (uint32_t)dat.size();
    if (payload_bytes > MAX_FRAME_PAYLOAD_BYTES+3) {
      continue;
    }

    // Ensure enough space for the fixed header [0x90][u16 id][u8 base][u16 len]
    const int header_size = 1 + 2 + 1 + 2;
    if (((int)USB_TX_SOFT_LIMIT - pos) < header_size) {
      if (pos > 0) {
        write_func(send_buf, pos);
        pos = 0;
      }
    }

    // Determine how many payload bytes we can fit without exceeding the soft limit.
    int available_after_header = (int)USB_TX_SOFT_LIMIT - pos - header_size;
    if (available_after_header < 0) {
      // After flush above this should not happen, but guard just in case
      write_func(send_buf, pos);
      pos = 0;
      available_after_header = (int)USB_TX_SOFT_LIMIT - header_size;
    }
    uint16_t len_to_send = (uint16_t)std::min<uint32_t>(payload_bytes, (uint32_t)std::max(0, available_after_header));

    uint8_t crc8 = calculate_crc8(dat.begin() + 1, 0xF1, dat.size() - 1);

    // op code
    send_buf[pos++] = 0x90;
    // u16 id (little endian)
    send_buf[pos++] = (uint8_t)(frame_id & 0xFF);
    send_buf[pos++] = (uint8_t)((frame_id >> 8) & 0xb111);
    // u8 base
    send_buf[pos++] = base;
    // u16 len actually sent (little endian)
    send_buf[pos++] = (uint8_t)(len_to_send & 0xFF);
    send_buf[pos++] = (uint8_t)((len_to_send >> 8) & 0xFF);
    // override cycle_count with crc8
    send_buf[pos++] = crc8;

    // Write only the bytes that fit; drop the exceeded part
    if (len_to_send > 0) {
      memcpy(&send_buf[pos], dat.begin() + 1, (size_t)len_to_send - 1);
      pos += len_to_send;
    }

    if (pos >= (int)USB_TX_SOFT_LIMIT) {
      write_func(send_buf, pos);
      pos = 0;
    }
  }

  if (pos > 0) {
    write_func(send_buf, pos);
  }
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
