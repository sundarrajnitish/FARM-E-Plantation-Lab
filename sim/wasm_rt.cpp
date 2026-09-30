// wasm_rt.cpp - minimal C runtime for the freestanding WebAssembly build.
// Deterministic and small: string/memory helpers, a first-fit heap for the
// Arduino String class, and operator new/delete.
#if defined(__wasm__)
#include <stddef.h>
#include <stdint.h>

extern "C" {
void* memcpy(void* d, const void* s, size_t n) {
  uint8_t* o = (uint8_t*)d; const uint8_t* i = (const uint8_t*)s;
  while (n--) *o++ = *i++;
  return d;
}
void* memmove(void* d, const void* s, size_t n) {
  uint8_t* o = (uint8_t*)d; const uint8_t* i = (const uint8_t*)s;
  if (o < i) { while (n--) *o++ = *i++; }
  else { o += n; i += n; while (n--) *--o = *--i; }
  return d;
}
void* memset(void* d, int c, size_t n) {
  uint8_t* o = (uint8_t*)d;
  while (n--) *o++ = (uint8_t)c;
  return d;
}
int memcmp(const void* a, const void* b, size_t n) {
  const uint8_t* x = (const uint8_t*)a; const uint8_t* y = (const uint8_t*)b;
  for (; n; --n, ++x, ++y) if (*x != *y) return *x - *y;
  return 0;
}
size_t strlen(const char* s) { size_t n = 0; while (s[n]) ++n; return n; }
int strcmp(const char* a, const char* b) {
  while (*a && *a == *b) { ++a; ++b; }
  return (uint8_t)*a - (uint8_t)*b;
}
int strncmp(const char* a, const char* b, size_t n) {
  for (; n; --n, ++a, ++b) { if (*a != *b || !*a) return (uint8_t)*a - (uint8_t)*b; }
  return 0;
}
char* strcpy(char* d, const char* s) { char* r = d; while ((*d++ = *s++)) {} return r; }
char* strncpy(char* d, const char* s, size_t n) {
  size_t i = 0;
  for (; i < n && s[i]; ++i) d[i] = s[i];
  for (; i < n; ++i) d[i] = 0;
  return d;
}
char* strchr(const char* s, int c) {
  for (;; ++s) { if (*s == (char)c) return (char*)s; if (!*s) return nullptr; }
}
char* strstr(const char* h, const char* n) {
  if (!*n) return (char*)h;
  for (; *h; ++h) {
    const char* a = h; const char* b = n;
    while (*a && *b && *a == *b) { ++a; ++b; }
    if (!*b) return (char*)h;
  }
  return nullptr;
}

// ---- heap: 8 MiB arena, first fit with coalescing on free
static const size_t HEAP_BYTES = 8u << 20;
alignas(16) static uint8_t g_heap[HEAP_BYTES];
struct Blk { size_t size; Blk* next; int free_; int pad; };
static Blk* g_first = nullptr;

static void heap_init() {
  g_first = (Blk*)g_heap;
  g_first->size = HEAP_BYTES - sizeof(Blk);
  g_first->next = nullptr;
  g_first->free_ = 1;
}

void* malloc(size_t n) {
  if (!g_first) heap_init();
  n = (n + 15) & ~(size_t)15;
  if (n == 0) n = 16;
  for (Blk* b = g_first; b; b = b->next) {
    if (!b->free_ || b->size < n) continue;
    if (b->size >= n + sizeof(Blk) + 32) {
      Blk* rest = (Blk*)((uint8_t*)(b + 1) + n);
      rest->size = b->size - n - sizeof(Blk);
      rest->next = b->next;
      rest->free_ = 1;
      b->next = rest;
      b->size = n;
    }
    b->free_ = 0;
    return b + 1;
  }
  __builtin_trap();
}

void free(void* p) {
  if (!p) return;
  Blk* b = (Blk*)p - 1;
  b->free_ = 1;
  for (Blk* c = g_first; c; c = c->next) {  // coalesce neighbours
    while (c->free_ && c->next && c->next->free_) {
      c->size += sizeof(Blk) + c->next->size;
      c->next = c->next->next;
    }
  }
}

void* calloc(size_t n, size_t m) {
  void* p = malloc(n * m);
  memset(p, 0, n * m);
  return p;
}

void* realloc(void* p, size_t n) {
  if (!p) return malloc(n);
  Blk* b = (Blk*)p - 1;
  if (b->size >= n) return p;
  void* q = malloc(n);
  memcpy(q, p, b->size);
  free(p);
  return q;
}

void __cxa_pure_virtual() { __builtin_trap(); }
int __cxa_atexit(void (*)(void*), void*, void*) { return 0; }
void* __dso_handle = nullptr;
}

void* operator new(size_t n) { return malloc(n); }
void* operator new[](size_t n) { return malloc(n); }
void operator delete(void* p) noexcept { free(p); }
void operator delete[](void* p) noexcept { free(p); }
void operator delete(void* p, size_t) noexcept { free(p); }
void operator delete[](void* p, size_t) noexcept { free(p); }
#endif
