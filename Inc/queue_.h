#ifndef _QUEUE_H
#define _QUEUE_H
#include <cstdint>

template <class T>
struct FIFO{
    using value_type = T;

    virtual value_type pop() = 0;
    virtual uint16_t size() = 0;
    virtual ~FIFO() {};

    virtual bool push(value_type n) = 0;
    virtual bool isempty() = 0;
    virtual bool isfull() = 0;
    virtual value_type front() = 0;

    virtual void clear() = 0;
};

template <>
struct FIFO<unsigned char>{
    using value_type = unsigned char;

    virtual value_type pop() = 0;
    virtual uint16_t size() = 0;
    virtual bool isfull() = 0;
    virtual ~FIFO() {};

    virtual bool push(value_type n) = 0;
    virtual bool push(value_type* n) = 0;
    virtual bool isempty() = 0;
    virtual value_type front() = 0;

    virtual void clear() = 0;
};

template <>
struct FIFO<char>{
    using value_type = char;

    virtual value_type pop() = 0;
    virtual uint16_t size() = 0;
    virtual ~FIFO() {};

    virtual bool push(value_type n) = 0;
    virtual bool push(value_type* n) = 0;
    virtual bool isempty() = 0;
    virtual bool isfull() = 0;
    virtual value_type front() = 0;

    virtual void clear() = 0;
};


template <class T, std::uint16_t qsize>
struct queue
    : FIFO<T>
{
    using interface_type = FIFO<T>;
    using typename interface_type::value_type;
    value_type storage[qsize];

    using uint16_t = std::uint16_t;

    uint16_t head;
    uint16_t tail;

    queue(){
        head = 0;
        tail = 0;
    }
 
    uint16_t next(uint16_t v)
    {
      v++;
      if (v >= qsize)
       v = 0;
      return v;
    }
    
    bool push(T n){
      storage[head] = n;
      head = next(head);
      return (tail == head);
    }

    bool push(T* n){
      while (true)
       {
        if (isempty()) break;
        if (push(*n)) return true;
        ++n;
       }
      return false;
    }

    bool isempty(){
      return head == tail;
    }

    bool isfull(){
      return next(head) == tail;
    }

    T front(){
      return storage[tail];
    }
    
    T pop() final {
        T a = storage[tail];
        tail = next(tail);
        return a;
    }

    void clear(){
     head = tail;
    }

    uint16_t size(){
     int16_t len = head - tail;
     if (len < 0) len += qsize;
     return len;
    }
};
#endif
