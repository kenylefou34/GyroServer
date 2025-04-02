#ifndef TYPES_HPP
#define TYPES_HPP

#include <stdlib.h>
#include <cstdint> // For uint16_t
#include <iostream>

enum EImageFormat : std::uint16_t {
  NONE = 0x0000,
  BGR_RAW = 0x002a,
  MONO = 0x001a,
  YUV = 0x01a6
};

struct FrameHeader {
 private:
  bool is_valid = false;
  std::uint8_t code = 0xb8;

 public:
  std::uint16_t frame_encoding = 0;
  std::uint16_t width = 0;
  std::uint16_t height = 0;
  std::uint16_t frame_id = 0;
  std::uint32_t encoded_total_size = 0;

 public:
  void unvalidate() { is_valid = false; }
  void validateECC(const uint8_t ecc_to_test)
  {
    int iframe_encoding = static_cast<int>(frame_encoding);
    int iwidth = static_cast<int>(width);
    int iheight = static_cast<int>(height);
    int iframe_id = static_cast<int>(frame_id);
    int iencoded_total_size = static_cast<int>(encoded_total_size);

    uint8_t ecc = code; // Start with the least significant byte of 'code'

         // XOR each byte of the fields to accumulate parity
    ecc ^= static_cast<uint8_t>(iframe_encoding & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_encoding >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_encoding >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_encoding >> 24) & 0xFF);

    ecc ^= static_cast<uint8_t>(iwidth & 0xFF);
    ecc ^= static_cast<uint8_t>((iwidth >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iwidth >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iwidth >> 24) & 0xFF);

    ecc ^= static_cast<uint8_t>(iheight & 0xFF);
    ecc ^= static_cast<uint8_t>((iheight >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iheight >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iheight >> 24) & 0xFF);

    ecc ^= static_cast<uint8_t>(iframe_id & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_id >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_id >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_id >> 24) & 0xFF);

    ecc ^= static_cast<uint8_t>(iencoded_total_size & 0xFF);
    ecc ^= static_cast<uint8_t>((iencoded_total_size >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iencoded_total_size >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iencoded_total_size >> 24) & 0xFF);

    is_valid = (ecc_to_test == ecc);
    if (!is_valid) {
      std::cerr << "Ecc " << std::hex << ecc_to_test << "unvalidated with value " << ecc << std::dec
                << std::endl;
    }
  }

 public:
  inline bool isValidEcc() const { return is_valid; }
  inline std::uint8_t beginCode() const { return code; }
};

#endif // TYPES_HPP
