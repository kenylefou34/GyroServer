#ifndef TYPES_HPP
#define TYPES_HPP

#include <stdlib.h>
#include <cstdint> // For uint16_t

enum EImageFormat : std::uint16_t {
  NONE = 0x0000,
  BGR_RAW = 0x002a,
  MONO = 0x001a,
  YUV = 0x01a6
};

#endif // TYPES_HPP
