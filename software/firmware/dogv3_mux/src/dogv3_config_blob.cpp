#include "dogv3_config_blob.h"

namespace dogv3 {

namespace {

uint16_t readU16LE(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readU32LE(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

bool magicMatches(const uint8_t *data) {
  for (uint8_t i = 0; i < 8; ++i) {
    if (data[i] != CONFIG_BLOB_MAGIC[i]) {
      return false;
    }
  }
  return true;
}

ConfigBlobHeader readHeader(const uint8_t *data) {
  ConfigBlobHeader header;
  for (uint8_t i = 0; i < 8; ++i) {
    header.magic[i] = data[i];
  }
  header.blob_version = readU16LE(data + 8);
  header.schema_version = readU16LE(data + 10);
  header.payload_len = readU32LE(data + 12);
  header.crc32 = readU32LE(data + 16);
  return header;
}

}  // namespace

uint32_t configBlobCrc32(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 1u) != 0u) {
        crc = (crc >> 1) ^ 0xEDB88320u;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

ConfigBlobStatus validateConfigBlob(
    const uint8_t *data,
    uint32_t len,
    uint16_t expected_schema_version,
    ConfigBlobHeader *out_header) {
  if (data == nullptr) {
    return CONFIG_BLOB_NULL;
  }
  if (len < CONFIG_BLOB_HEADER_LEN) {
    return CONFIG_BLOB_TOO_SHORT;
  }
  if (!magicMatches(data)) {
    return CONFIG_BLOB_BAD_MAGIC;
  }

  const ConfigBlobHeader header = readHeader(data);
  if (header.blob_version != CONFIG_BLOB_VERSION) {
    return CONFIG_BLOB_BAD_VERSION;
  }
  if (header.schema_version != expected_schema_version) {
    return CONFIG_BLOB_BAD_SCHEMA;
  }
  if (header.payload_len != len - CONFIG_BLOB_HEADER_LEN) {
    return CONFIG_BLOB_LENGTH_MISMATCH;
  }

  const uint8_t *payload = data + CONFIG_BLOB_HEADER_LEN;
  if (configBlobCrc32(payload, header.payload_len) != header.crc32) {
    return CONFIG_BLOB_BAD_CRC;
  }

  if (out_header != nullptr) {
    *out_header = header;
  }
  return CONFIG_BLOB_OK;
}

}  // namespace dogv3
