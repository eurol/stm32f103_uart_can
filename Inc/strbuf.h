#ifndef _STRBUF_H
#define _STRBUF_H

typedef unsigned int uint32_t;

class StrBuf
{
 public:
 StrBuf(uint32_t size=64);
 ~StrBuf();
 uint32_t length() const;
 uint32_t maxsize() const;
 void clear();
 char writechar(char t);
 char writestr(char *p);
 uint32_t copyto(char *p, uint32_t maxlen);
 char writehexdigit(unsigned char b);
 char writehexbyte(unsigned char b);
 char writehex3dig(unsigned short w);
 char writehex8dig(unsigned int d);
 char writeuint32lz(unsigned int d);
 char writeuint32(unsigned int d);
 const char* get() const;

 private:
  char *buf;
  uint32_t len,sz;
};

#endif
