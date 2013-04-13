/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*
   Concepts and parts of this file have been contributed by Dmytro Milinevskyy <milinevskyy@gmail.com>
 */

#include "ch.h"

static inline int tolower(int c) {
  if ('A' <= c && c <= 'Z')
    return c + ('a' - 'A');

  return c;
}

static inline size_t strlen(const char *s) {
  int i;

  if (!s)
    return 0;

  for (i=0; *s; ++s)
    ++i;

  return i;
}

int strcasecmp(const char *s1, const char *s2) {
  while (*s1 && *s2) {
    int d = tolower(*s1) - tolower(*s2);

    if (d)
      return d;

    ++s1;
    ++s2;
  }

  return tolower(*s1) - tolower(*s2);
} 

size_t strspn(const char *s, const char *accept) {
  size_t l = 0;
  int a = 1;
  int i;
  int al = strlen(accept);

  while (a && *s) {
    for(a=i=0; !a && i<al; ++i)
      if (*s == accept[i])
        a = 1;

    if (a)
      ++l;

    ++s;
  }

  return l;
}

char *strchr(const char *s, int c) {
  while (*s != (char)c) {
    if (!*s)
      return NULL;

    ++s;
  }

  return (char *)s;
}

char *
strpbrk(const char *s1, const char *s2) 
{
  const char *s;
  int c, sc; 

  while ((c = *s1++)) {
    s = s2;
    while ((sc = *s++)) {
      if (sc == c)
        return (char *)(s1 - 1);
    }
  }   

  return NULL;
}
