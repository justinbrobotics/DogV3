#ifndef DOGV3_CONFIG_BLOB_H
#define DOGV3_CONFIG_BLOB_H

#include <stdint.h>

namespace dogv3 {

static constexpr uint8_t CONFIG_BLOB_MAGIC[8] = {'S', 'D', 'B', 'C', 'F', 'G', '1', '\0'};
static constexpr uint16_t CONFIG_BLOB_VERSION = 1;
static constexpr uint32_t CONFIG_BLOB_HEADER_LEN = 20;

enum ConfigBlobStatus : uint8_t {
  CONFIG_BLOB_OK = 0,
  CONFIG_BLOB_NULL = 1,
  CONFIG_BLOB_TOO_SHORT = 2,
  CONFIG_BLOB_BAD_MAGIC = 3,
  CONFIG_BLOB_BAD_VERSION = 4,
  CONFIG_BLOB_BAD_SCHEMA = 5,
  CONFIG_BLOB_LENGTH_MISMATCH = 6,
  CONFIG_BLOB_BAD_CRC = 7,
};

struct ConfigBlobHeader {
  uint8_t magic[8];
  uint16_t blob_version;
  uint16_t schema_version;
  uint32_t payload_len;
  uint32_t crc32;
};

uint32_t configBlobCrc32(const uint8_t *data, uint32_t len);

ConfigBlobStatus validateConfigBlob(
    const uint8_t *data,
    uint32_t len,
    uint16_t expected_schema_version,
    ConfigBlobHeader *out_header = nullptr);

}  // namespace dogv3

#endif  // DOGV3_CONFIG_BLOB_H
