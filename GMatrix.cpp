#include "include/GMatrix.h"
#include <cmath>
#include <optional>
#include <cassert>

GMatrix::GMatrix() {
    fMat[0] = 1.0f; fMat[1] = 0.0f; fMat[2] = 0.0f; 
    fMat[3] = 1.0f; fMat[4] = 0.0f; fMat[5] = 0.0f; 
}

GMatrix GMatrix::Translate(float tx, float ty) {
    return GMatrix(1, 0, tx, 0, 1, ty);
}

GMatrix GMatrix::Scale(float sx, float sy) {
    return GMatrix(sx, 0, 0, 0, sy, 0);
}

GMatrix GMatrix::Rotate(float radians) {
    float cosVal = cosf(radians);
    float sinVal = sinf(radians);
    return GMatrix(cosVal, -sinVal, 0, sinVal, cosVal, 0);
}

GMatrix GMatrix::Concat(const GMatrix& a, const GMatrix& b) {
    return GMatrix(
        a.fMat[0] * b.fMat[0] + a.fMat[2] * b.fMat[1],
        a.fMat[0] * b.fMat[2] + a.fMat[2] * b.fMat[3],
        a.fMat[0] * b.fMat[4] + a.fMat[2] * b.fMat[5] + a.fMat[4],
        a.fMat[1] * b.fMat[0] + a.fMat[3] * b.fMat[1],
        a.fMat[1] * b.fMat[2] + a.fMat[3] * b.fMat[3],
        a.fMat[1] * b.fMat[4] + a.fMat[3] * b.fMat[5] + a.fMat[5]
    );
}

nonstd::optional<GMatrix> GMatrix::invert() const {
    float det = fMat[0] * fMat[3] - fMat[1] * fMat[2];
    
    if (det == 0) {
        return nonstd::nullopt; 
    }

    float invDet = 1.0f / det;

    float a =  fMat[3] * invDet;
    float b = -fMat[1] * invDet;
    float c = -fMat[2] * invDet;
    float d =  fMat[0] * invDet;
    float e = (fMat[2] * fMat[5] - fMat[3] * fMat[4]) * invDet;
    float f = (fMat[1] * fMat[4] - fMat[0] * fMat[5]) * invDet;

    return GMatrix(a, c, e, b, d, f);
}

void GMatrix::mapPoints(GPoint dst[], const GPoint src[], int count) const {
    for (int i = 0; i < count; ++i) {
        float sx = src[i].x;
        float sy = src[i].y;
        dst[i].x = fMat[0] * sx + fMat[2] * sy + fMat[4];
        dst[i].y = fMat[1] * sx + fMat[3] * sy + fMat[5];
    }
}