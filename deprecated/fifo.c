#include "fifo.h"

uint8_t FIFO_IsEmpty(FIFO_type *fifo)
{
  if(fifo->HeadIndex==fifo->TailIndex)
    return 1;
  else
    return 0;
}

void FIFO_Put(FIFO_type *fifo, uint8_t data)
{
  fifo->HeadIndex=(fifo->HeadIndex+1)%BUF_SIZE;
  fifo->Buf[fifo->HeadIndex]=data;
}

uint8_t FIFO_Pop(FIFO_type *fifo)
{
  fifo->TailIndex=(fifo->TailIndex+1)%BUF_SIZE;
  return fifo->Buf[fifo->TailIndex];
}

uint8_t FIFO_IsValid(FIFO_type *fifo)
{
	// Get tail index
	uint8_t sum = -3;								// Minus 3 to balance 0xFF's
	uint16_t idx = (fifo->TailIndex+1)%BUF_SIZE;

	// Sum all received data
	while (idx != fifo->HeadIndex)
	{
		sum -= fifo->Buf[idx];
		idx = (idx+1)%BUF_SIZE;
	}

	// Check it!
	if (sum == fifo->Buf[fifo->HeadIndex]) return 1;
	else return 0;
}
