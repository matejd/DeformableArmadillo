/*
  Copyright (C) 2008 Martin Storsjo

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

/*
 * add '#define FORSYTH_IMPLEMENTATION' before including to create the implementation
 *
 * C99 compatibility
 * forsyth prefix
 * outIndices parameter
 * convert to header file single-file library
 *
 * Copyright (c) 2014, Ivan Vashchaev
 */

#ifndef FORSYTH_H
#define FORSYTH_H

#include <stdint.h>

typedef int32_t ForsythVertexIndexType;

#ifdef __cplusplus
extern "C" {
#endif

ForsythVertexIndexType *forsythReorderIndices(ForsythVertexIndexType *outIndices, const ForsythVertexIndexType *indices, int nTriangles, int nVertices);

#ifdef __cplusplus
}
#endif

#endif // FORSYTH_H
