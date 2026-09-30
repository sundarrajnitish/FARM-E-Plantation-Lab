// WString.h (simulation mock) - the Arduino String class, heap-backed like
// the real one (malloc/realloc), with the methods the sketches use.
#pragma once
#include "Arduino.h"

class String {
 public:
  String(const char* s = "") { assign(s ? s : "", s ? strlen(s) : 0); }
  String(const String& o) { assign(o.buf_ ? o.buf_ : "", o.len_); }
  String(String&& o) noexcept : buf_(o.buf_), len_(o.len_), cap_(o.cap_) { o.buf_ = nullptr; o.len_ = o.cap_ = 0; }
  explicit String(char c) { char t[2] = {c, 0}; assign(t, 1); }
  explicit String(int v, int base = 10) { fromSigned(v, base); }
  explicit String(unsigned int v, int base = 10) { fromUnsigned(v, base); }
  explicit String(long v, int base = 10) { fromSigned(v, base); }
  explicit String(unsigned long v, int base = 10) { fromUnsigned(v, base); }
  explicit String(float v, int digits = 2) { fromDouble(v, digits); }
  explicit String(double v, int digits = 2) { fromDouble(v, digits); }
  ~String() { if (buf_) free(buf_); }

  String& operator=(const String& o) { if (this != &o) assign(o.buf_ ? o.buf_ : "", o.len_); return *this; }
  String& operator=(String&& o) noexcept {
    if (this != &o) { if (buf_) free(buf_); buf_ = o.buf_; len_ = o.len_; cap_ = o.cap_; o.buf_ = nullptr; o.len_ = o.cap_ = 0; }
    return *this;
  }
  String& operator=(const char* s) { assign(s ? s : "", s ? strlen(s) : 0); return *this; }

  unsigned int length() const { return len_; }
  const char* c_str() const { return buf_ ? buf_ : ""; }
  char charAt(unsigned int i) const { return i < len_ ? buf_[i] : 0; }
  char operator[](unsigned int i) const { return charAt(i); }
  bool reserve(unsigned int n) { grow(n); return true; }

  bool concat(const char* s, unsigned int n) { grow(len_ + n); memcpy(buf_ + len_, s, n); len_ += n; buf_[len_] = 0; return true; }
  bool concat(const char* s) { return concat(s, (unsigned int)strlen(s)); }
  bool concat(const String& s) { return concat(s.c_str(), s.len_); }
  bool concat(char c) { return concat(&c, 1); }
  bool concat(int v) { return concat(String(v)); }
  bool concat(unsigned int v) { return concat(String(v)); }
  bool concat(long v) { return concat(String(v)); }
  bool concat(unsigned long v) { return concat(String(v)); }
  bool concat(float v) { return concat(String(v)); }
  bool concat(double v) { return concat(String(v)); }
  template <typename T> String& operator+=(const T& v) { concat(v); return *this; }

  bool equals(const char* s) const { return strcmp(c_str(), s) == 0; }
  bool equals(const String& s) const { return len_ == s.len_ && strcmp(c_str(), s.c_str()) == 0; }
  bool operator==(const char* s) const { return equals(s); }
  bool operator==(const String& s) const { return equals(s); }
  bool operator!=(const char* s) const { return !equals(s); }
  bool operator!=(const String& s) const { return !equals(s); }
  bool startsWith(const char* s) const { size_t n = strlen(s); return n <= len_ && strncmp(c_str(), s, n) == 0; }

  int indexOf(char c, unsigned int from = 0) const {
    for (unsigned int i = from; i < len_; ++i) if (buf_[i] == c) return (int)i;
    return -1;
  }
  int indexOf(const char* s, unsigned int from = 0) const {
    if (from >= len_) return -1;
    const char* p = strstr(c_str() + from, s);
    return p ? (int)(p - c_str()) : -1;
  }
  String substring(unsigned int a, unsigned int b) const {
    if (b > len_) b = len_;
    if (a > b) a = b;
    String r; r.assign(c_str() + a, b - a); return r;
  }
  String substring(unsigned int a) const { return substring(a, len_); }

  // String::replace from the Arduino core: every occurrence, left to right
  void replace(const String& find, const String& repl) {
    if (len_ == 0 || find.len_ == 0) return;
    String out;
    const char* s = c_str();
    const char* f = find.c_str();
    unsigned int i = 0;
    while (i < len_) {
      if (i + find.len_ <= len_ && strncmp(s + i, f, find.len_) == 0) { out.concat(repl); i += find.len_; }
      else { out.concat(s[i]); ++i; }
    }
    *this = static_cast<String&&>(out);
  }
  void replace(const char* f, const char* r) { replace(String(f), String(r)); }

  long toInt() const {
    const char* s = c_str(); long v = 0; bool neg = false;
    while (*s == ' ' || *s == '\t') ++s;
    if (*s == '-') { neg = true; ++s; } else if (*s == '+') ++s;
    while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
    return neg ? -v : v;
  }

 private:
  char* buf_ = nullptr;
  unsigned int len_ = 0, cap_ = 0;
  void grow(unsigned int n) {
    if (buf_ && cap_ >= n) return;
    unsigned int c = cap_ ? cap_ : 16;
    while (c < n) c *= 2;
    buf_ = (char*)realloc(buf_, c + 1);
    if (cap_ == 0) buf_[0] = 0;
    cap_ = c;
  }
  void assign(const char* s, unsigned int n) {
    grow(n);
    memmove(buf_, s, n);
    buf_[n] = 0; len_ = n;
  }
  void fromUnsigned(unsigned long v, int base) {
    char t[34]; int i = 33; t[i] = 0;
    do { int d = (int)(v % base); t[--i] = (char)(d < 10 ? '0' + d : 'a' + d - 10); v /= base; } while (v);
    assign(t + i, 33 - i);
  }
  void fromSigned(long v, int base) {
    if (v < 0 && base == 10) { fromUnsigned((unsigned long)(-v), base); String r("-"); r.concat(*this); *this = r; }
    else fromUnsigned((unsigned long)v, base);
  }
  void fromDouble(double v, int digits) {
    struct Buf : Print { String* s; size_t write(uint8_t c) override { s->concat((char)c); return 1; } } b;
    assign("", 0);
    b.s = this;
    b.printFloat(v, digits);
  }
};

inline String operator+(const String& a, const String& b) { String r(a); r.concat(b); return r; }
inline String operator+(const String& a, const char* b) { String r(a); r.concat(b); return r; }
inline String operator+(const char* a, const String& b) { String r(a); r.concat(b); return r; }
inline String operator+(const String& a, char b) { String r(a); r.concat(b); return r; }
inline String operator+(const String& a, int b) { String r(a); r.concat(b); return r; }
inline String operator+(const String& a, long b) { String r(a); r.concat(b); return r; }
inline String operator+(const String& a, unsigned long b) { String r(a); r.concat(b); return r; }

inline size_t Print::print(const String& s) { return write(s.c_str()); }
inline size_t Print::print(const Printable& p) { return p.printTo(*this); }
