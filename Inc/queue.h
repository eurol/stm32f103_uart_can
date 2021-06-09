#ifndef _QUEUE_H
#define _QUEUE_H
#include <cstdint>

template <class T>
struct queue
{
private:
    T* storage;
    typedef size_t counter_t;
    counter_t qsize;
    counter_t head;
    counter_t tail;

public:
    queue(counter_t size = 64){
        storage = new T[size];
        qsize = size;
        head = 0;
        tail = 0;
    }
    ~queue(){
     delete[] storage;
    }
 
    inline size_t next(counter_t v)
    {
      v++;
      if (v < qsize) return v;
      return 0;
    }
    
    inline bool push(T n){
      if (isfull()) return true;
      storage[head] = n;
      head = next(head);
      return false;
    }

    inline bool push(T* n){
      while (*n)
       {
        if (push(*n)) return true;
        ++n;
       }
      return false;
    }

    inline bool isempty(){
      return head == tail;
    }

    inline bool isfull(){
      return next(head) == tail;
    }

    inline T front(){
      return storage[tail];
    }
    
    inline T pop() {
        if (isempty()) return T();
        T tmp = storage[tail];
        tail = next(tail);
        return tmp;
    }

    inline void drop(counter_t count)  //***
    {
        if (count >= size())
        {
          clear();
          return;  
        }
        count = tail + count;
        if (count < qsize) tail = count;
         else tail = count - qsize;
    }

    inline counter_t pop(counter_t count, T* where)
    {
        counter_t result = 0;
        while (count)
         {
           if (isempty()) break;
           where[result] = pop();
           result++;
           count--;
         }
        return result;
        /*
        size_t cangive = size();
        if (count > cangive) count = cangive;
        size_t end = cangive + tail;
        if (end <= qsize)
        {
           memcpy(where, &storage[tail], sizeof(T) * count);
           tail += cangive;
           return cangive;
        }
        // data is splitted
        count = qsize - tail;
        memcpy(where, &storage[tail], sizeof(T) * count);
        memcpy(&where[count], &storage[0], sizeof(T) * (cangive - count));
        tail = tail + cangive - qsize;
        return cangive;*/
    }

    inline void clear(){
     head = tail;
    }

    inline counter_t size(){
     int len = head - tail;
     if (len < 0) len += qsize;
     return len;
    }

    inline counter_t maxsize(){
     return qsize;
    }
};
#endif
