#include "strbuf.h"
//#include <memory>

char hexdig(unsigned char b)
{
  return b>9?'A'+b-10:'0' + b;
}

StrBuf::StrBuf(uint32_t size): sz(size), len(0)
{
 buf = new char[sz];
}

StrBuf::~StrBuf()
{
 delete[] buf;
}

uint32_t StrBuf::length() const
{
  return len;
}

uint32_t StrBuf::maxsize() const
{
  return sz;
}

void StrBuf::clear()
{
  len=0;
  //memset(buf,0,sz);
}

char StrBuf::writechar(char t)
{
  if (len>=sz-1) return 0;
  buf[len++]=t;
  return 1;
}

char StrBuf::writestr(char *p)
{
  while (*p)
  {
   if (len>=sz-1) return 0;
   buf[len++]=*p++;
  }
  return 1;
}

uint32_t StrBuf::copyto(char *p, uint32_t maxlen)
{
  if (maxlen>len) maxlen=len;
  for (uint32_t i=0;i<maxlen;i++)
  {
    p[i]=buf[i];
  }
  return maxlen;
}

char StrBuf::writehexdigit(unsigned char b)
{
  return writechar(hexdig(b));
}

char StrBuf::writehexbyte(unsigned char b)
{
  if (!writehexdigit(b>>4)) return 0;
  return writehexdigit(b&15);
}

char StrBuf::writehex3dig(unsigned short w)
{
  if (!writehexdigit(w >> 8)) return 0;
  return writehexbyte(w & 255);
}

char StrBuf::writehex8dig(unsigned int d)
{
  if (!writehexbyte(d >> 24)) return 0;
  if (!writehexbyte(d >> 16)) return 0;
  if (!writehexbyte(d >> 8)) return 0;
  return writehexbyte(d & 255);
}

char StrBuf::writeuint32lz(unsigned int d)
{
  char t[10]={0};
  signed char q=0;
  while (d)
  {
    t[q]=d%10;
    d/=10;
    q++;
  }
  for (q=9;q>=0;q--)
    if (writechar('0'+t[q])==0) return 0;
  return 1;
}

char StrBuf::writeuint32(unsigned int d)
{
  if (d==0)
  {
    return writechar('0');
  }
  char t[10]={0};
  signed char q=0;
  while (d)
  {
    t[q]=d%10;
    d/=10;
    q++;
  }
  q--;
  for (;q>=0;q--)
    if (writechar('0'+t[q])==0) return 0;
  return 1;
}

const char* StrBuf::get() const
{
  buf[len]=0;
  return buf;
}

/*
class StrBuf
{
 public:
 StrBuf(uint32_t size=64);
 ~StrBuf();
 uint32_t length();
 uint32_t maxsize();
 void clear();
 uint32_t copyto(char *p, uint32_t maxlen);

 private:
  char *buf;
  uint32_t len,sz;
};

*/
