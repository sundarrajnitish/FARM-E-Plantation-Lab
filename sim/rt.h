// rt.h - the handful of C library functions the mocks and sketches use.
// Native builds take them from libc; the freestanding wasm build gets them
// from wasm_rt.cpp.
#pragma once
#include <stddef.h>
#include <stdint.h>

#if defined(__wasm__)
extern "C" {
size_t strlen(const char* s);
int strcmp(const char* a, const char* b);
int strncmp(const char* a, const char* b, size_t n);
char* strcpy(char* d, const char* s);
char* strncpy(char* d, const char* s, size_t n);
char* strchr(const char* s, int c);
char* strstr(const char* h, const char* n);
void* memcpy(void* d, const void* s, size_t n);
void* memmove(void* d, const void* s, size_t n);
void* memset(void* d, int c, size_t n);
int memcmp(const void* a, const void* b, size_t n);
void* malloc(size_t n);
void* calloc(size_t n, size_t m);
void* realloc(void* p, size_t n);
void free(void* p);
}
#else
#include <stdlib.h>
#include <string.h>
#endif
