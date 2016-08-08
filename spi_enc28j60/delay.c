#include "delay.h"

volatile uint32_t timestamp = 0;

void DelayMs (uint32_t delayms)
{
   uint32_t delaystamp = timestamp + delayms*1000;
   while (delaystamp>timestamp){}
}

void DelayMcs (uint32_t delaymcs)
{
   uint32_t delaystamp = timestamp + delaymcs;
   while (delaystamp>timestamp){}
}
