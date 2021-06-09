#ifndef _CHARDEQUE_H
#define _CHARDEQUE_H

typedef unsigned int uint32_t;

class CharDeque
{
 public:
  CharDeque(uint32_t size = 256);
  ~CharDeque();
 char push(char c);
 char push(const char *s);
 uint32_t size();
 uint32_t maxsize();
 char pop();
 void clear();

 private:
  char *data;
  uint32_t sz,len,first;
};

#endif
