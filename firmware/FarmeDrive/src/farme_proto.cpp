// farme_proto.cpp - see farme_proto.h
#include "farme_proto.h"

namespace farme {

uint8_t crc8(const uint8_t* d, size_t n, uint8_t crc) {
  while (n--) {
    crc ^= *d++;
    for (uint8_t i = 0; i < 8; ++i) crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
  }
  return crc;
}

static const char HEXD[] = "0123456789ABCDEF";

static size_t put_int(char* out, size_t pos, size_t cap, int32_t v) {
  char tmp[12];
  uint8_t k = 0;
  uint32_t u = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
  do { tmp[k++] = (char)('0' + u % 10); u /= 10; } while (u);
  if (v < 0) tmp[k++] = '-';
  if (pos + k >= cap) return 0;
  while (k) out[pos++] = tmp[--k];
  return pos;
}

size_t encode(const Frame& fr, char* out, size_t cap) {
  if (cap < 8 || fr.n > MAX_FIELDS) return 0;
  size_t p = 0;
  out[p++] = '$';
  out[p++] = fr.type;
  for (uint8_t i = 0; i < fr.n; ++i) {
    if (p + 1 >= cap) return 0;
    out[p++] = ',';
    p = put_int(out, p, cap, fr.f[i]);
    if (!p) return 0;
  }
  uint8_t crc = crc8((const uint8_t*)out + 1, p - 1);
  if (p + 5 > cap) return 0;
  out[p++] = '*';
  out[p++] = HEXD[crc >> 4];
  out[p++] = HEXD[crc & 15];
  out[p++] = '\n';
  out[p] = 0;
  return p;
}

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool Parser::feed(uint8_t c) {
  if (c == '$') {  // (re)start: a '$' can never appear inside a frame
    if (in_ && len_) ++bad_syntax;
    in_ = true;
    len_ = 0;
    return false;
  }
  if (!in_) return false;
  if (c == '\n' || c == '\r') {
    in_ = false;
    buf_[len_] = 0;
    if (parse()) { ++ok; return true; }
    return false;
  }
  if (len_ >= MAX_LINE) {
    ++overlong;
    in_ = false;
    return false;
  }
  buf_[len_++] = (char)c;
  return false;
}

// buf_ holds "T,1,2,3*CC"
bool Parser::parse() {
  int star = -1;
  for (int i = 0; i < len_; ++i) if (buf_[i] == '*') { star = i; break; }
  if (star < 1 || len_ != star + 3) { ++bad_syntax; return false; }
  int h = hexval(buf_[star + 1]), l = hexval(buf_[star + 2]);
  if (h < 0 || l < 0) { ++bad_syntax; return false; }
  if (crc8((const uint8_t*)buf_, (size_t)star) != (uint8_t)(h * 16 + l)) { ++bad_crc; return false; }
  char type = buf_[0];
  if (!((type >= 'A' && type <= 'Z'))) { ++bad_syntax; return false; }
  Frame fr;
  fr.type = type;
  int i = 1;
  while (i < star) {
    if (buf_[i] != ',' || fr.n >= MAX_FIELDS) { ++bad_syntax; return false; }
    ++i;
    bool neg = false;
    if (i < star && buf_[i] == '-') { neg = true; ++i; }
    int digits = 0;
    int64_t v = 0;
    while (i < star && buf_[i] >= '0' && buf_[i] <= '9') {
      v = v * 10 + (buf_[i] - '0');
      if (++digits > 10) { ++bad_syntax; return false; }
      ++i;
    }
    if (!digits || v > 2147483647LL) { ++bad_syntax; return false; }
    fr.f[fr.n++] = (int32_t)(neg ? -v : v);
  }
  frame_ = fr;
  return true;
}

}  // namespace farme
