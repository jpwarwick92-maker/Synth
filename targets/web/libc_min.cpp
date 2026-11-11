// The three C library routines the compiler may call on its own.
// The WebAssembly build links no C library, so they are provided here.
// Built with -fno-builtin so the loops are not turned back into calls.
#include <stddef.h>

extern "C" {

void* memcpy(void* dst, const void* src, size_t n) {
  unsigned char* d = (unsigned char*)dst;
  const unsigned char* s = (const unsigned char*)src;
  for (size_t i = 0; i < n; ++i) d[i] = s[i];
  return dst;
}

void* memmove(void* dst, const void* src, size_t n) {
  unsigned char* d = (unsigned char*)dst;
  const unsigned char* s = (const unsigned char*)src;
  if (d < s) {
    for (size_t i = 0; i < n; ++i) d[i] = s[i];
  } else {
    for (size_t i = n; i > 0; --i) d[i - 1] = s[i - 1];
  }
  return dst;
}

void* memset(void* dst, int c, size_t n) {
  unsigned char* d = (unsigned char*)dst;
  for (size_t i = 0; i < n; ++i) d[i] = (unsigned char)c;
  return dst;
}

}  // extern "C"
