#include "chardeque.h"

CharDeque::CharDeque(uint32_t size): sz(size), first(0), len(0)
{
  data=new char[sz];
}

CharDeque::~CharDeque()
{
  delete[] data;
}

char CharDeque::push(char c)
{
 if (len == sz) return 0;
 int pos = first + len;
 if (pos >= sz) pos -= sz;
 len++;
 data[pos]=c;
 return 1;
}

char CharDeque::push(const char *s)
{
 while (*s)
  {
    if (push(*s++) == 0) return 0;
  }
 return 1;
}

uint32_t CharDeque::maxsize()
{
 return sz;
}

uint32_t CharDeque::size()
{
 return len;
}

char CharDeque::pop()
{
 if (!len) return 0;
 char t = data[first];
 data[first] = 0;
 first++; len--;
 if (first >= sz) first -= sz;
 return t;
}

void CharDeque::clear()
{
 first=len=0; 
}

/*
 private:
  char *data;
  int sz,len,first;
*/
