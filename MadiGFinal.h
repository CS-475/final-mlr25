#ifndef MadiGFinal_DEFINED
#define MadiGFinal_DEFINED
#include "include/GFinal.h"

class MadiGFinal : public GFinal {
public:
    std::shared_ptr<GPath> strokePolygon(const GPoint points[], int count, 
                                        float width, bool isClosed) override;
	std::shared_ptr<GShader> createColorMatrixShader(const GColorMatrix& colorMatrix,
                                                             GShader* realShader) override;
	// std::shared_ptr<GShader> createVoronoiShader(const GPoint points[],
    //                                                      const GColor colors[],
    //                                                      int count) override;
	std::shared_ptr<GShader> createLinearPosGradient(GPoint p0, GPoint p1,
                                                             const GColor colors[],
                                                             const float pos[],
                                                             int count);
};


#endif