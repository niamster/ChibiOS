/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * Some string stubs needed for shell
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
