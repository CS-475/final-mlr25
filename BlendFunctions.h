#ifndef BLENDFUNCTIONS_H
#define BLENDFUNCTIONS_H

#include "include/GPixel.h"
#include "include/GBlendMode.h"

// Define all blend function prototypes here
GPixel clear(GPixel src, GPixel dst);
GPixel blendSrc(GPixel src, GPixel dst);
GPixel blendDst(GPixel src, GPixel dst);
GPixel blendSrcOver(GPixel src, GPixel dst);
GPixel blendDstOver(GPixel src, GPixel dst);
GPixel blendSrcIn(GPixel src, GPixel dst);
GPixel blendDstIn(GPixel src, GPixel dst);
GPixel blendSrcOut(GPixel src, GPixel dst);
GPixel blendDstOut(GPixel src, GPixel dst);
GPixel blendSrcATop(GPixel src, GPixel dst);
GPixel blendDstATop(GPixel src, GPixel dst);
GPixel blendXor(GPixel src, GPixel dst);

using BlendFunc = GPixel (*)(GPixel src, GPixel dst);
BlendFunc getBlendFunction(GBlendMode blendMode);

#endif // BLENDFUNCTIONS_H
