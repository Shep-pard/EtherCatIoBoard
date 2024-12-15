#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */

   uint16_t Inputs[3];
   uint32_t Encoder[4];
   uint16_t Analog[6];

   /* Outputs */

   uint16_t Outputs[3];

} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
