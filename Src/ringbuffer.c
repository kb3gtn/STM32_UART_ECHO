#include "ringbuffer.h"

///////////////////////// UINT8_T RING BUFFER //////////////////////////////

//initializes the given ringbuffer with the supplied array and its length
inline void rb_init(ringbuf_uint8t *rb, uint8_t *array, unsigned char length)
{
    rb->buf = array;
    rb->length = length;
    rb->head = rb->tail = 0;
    // zero out buffer
    for (uint8_t idx=0; idx < length; ++idx){
    	rb->buf[idx] = 0;
    }
}

//returns boolean true if the ringbuffer is empty, false otherwise
inline unsigned char rb_isempty(ringbuf_uint8t *rb)
{
    return (rb->head == rb->tail);
}

//returns boolean true if the ringbuffer is full, false otherwise
inline unsigned char rb_isfull(ringbuf_uint8t *rb)
{
    return (((rb->tail + 1) % rb->length) == rb->head);
}

//consumes an element from the buffer
//returns NULL if buffer is empty or a pointer to the array element otherwise
inline uint8_t* rb_get(ringbuf_uint8t *rb)
{
    uint8_t *ret;
    if (rb->head == rb->tail)
        return 0;
    else
    {
        ret = rb->buf + rb->head;                   //index into the array at head pointer
        rb->head = (rb->head + 1) % rb->length;     //move head pointer forward one element (with wraparound)
        return ret;
    }
}

//puts an element into the buffer
//returns 0 if buffer is full, otherwise returns 1
inline unsigned char rb_put(ringbuf_uint8t *rb, uint8_t c)
{
    char newtail;
    newtail = (rb->tail + 1) % rb->length;     //calculate where the new tail would be
    if (newtail == rb->head)                        //if the new tail would make the buffer look empty, buffer is full
        return 0;
    else
    {
        rb->buf[rb->tail] = c;                      //store the data
        rb->tail = newtail;                         //move the tail pointer forward (with wraparound)
        return 1;
    }
}

