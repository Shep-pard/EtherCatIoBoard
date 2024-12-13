#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */

   uint16_t Inputs[4];
   uint64_t Encoder[4];
   uint16_t Analog[6];

   /* Outputs */

   uint16_t Outputs[4];

} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
