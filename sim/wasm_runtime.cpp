// Freestanding WebAssembly has no C library, so the compiler's block copy and fill
// helpers are provided here. Built with -fno-builtin so the loops are not turned
// back into calls to themselves.
#include <stddef.h>

extern "C" {

__attribute__((used)) void* memcpy(void* dst, const void* src, size_t n) {
  unsigned char* d = (unsigned char*)dst;
  const unsigned char* s = (const unsigned char*)src;
  for (size_t i = 0; i < n; ++i) d[i] = s[i];
  return dst;
}

__attribute__((used)) void* memset(void* dst, int c, size_t n) {
  unsigned char* d = (unsigned char*)dst;
  for (size_t i = 0; i < n; ++i) d[i] = (unsigned char)c;
  return dst;
}

}  // extern "C"
