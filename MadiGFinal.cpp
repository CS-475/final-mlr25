#include "MadiGFinal.h"
#include <cmath>
#include <algorithm>
#include "include/GColor.h"
#include "include/GShader.h"
#include "MyCanvas.h"
#include "include/GPathBuilder.h"

class ColorMatrixShader : public GShader {
public:
    ColorMatrixShader(const GColorMatrix& matrix, GShader* realShader)
        : fMatrix(matrix), fRealShader(realShader) {}

    bool isOpaque() override {
        return fRealShader->isOpaque();
    }

    bool setContext(const GMatrix& ctm) override {
        return fRealShader->setContext(ctm);
    }

    void shadeRow(int x, int y, int count, GPixel row[]) override {
		fRealShader->shadeRow(x, y, count, row);

		for (int i = 0; i < count; ++i) {
			GColor color = pixelToColor(row[i]);
			color = applyMatrix(color);
			row[i] = colorToPixel(color);     
		}
	}

private:
    GColorMatrix fMatrix;
    GShader* fRealShader;

    // Convert GPixel to GColor
    GColor pixelToColor(const GPixel& pixel) {
        float a = GPixel_GetA(pixel) / 255.0f;
        float r = GPixel_GetR(pixel) / 255.0f / (a > 0 ? a : 1.0f);
        float g = GPixel_GetG(pixel) / 255.0f / (a > 0 ? a : 1.0f);
        float b = GPixel_GetB(pixel) / 255.0f / (a > 0 ? a : 1.0f);
        return GColor::RGBA(r, g, b, a);
    }

    GPixel colorToPixel(const GColor& color) {
        float r = std::clamp(color.r, 0.0f, 1.0f);
        float g = std::clamp(color.g, 0.0f, 1.0f);
        float b = std::clamp(color.b, 0.0f, 1.0f);
        float a = std::clamp(color.a, 0.0f, 1.0f);
        uint8_t ir = static_cast<uint8_t>(r * a * 255 + 0.5f);
        uint8_t ig = static_cast<uint8_t>(g * a * 255 + 0.5f);
        uint8_t ib = static_cast<uint8_t>(b * a * 255 + 0.5f);
        uint8_t ia = static_cast<uint8_t>(a * 255 + 0.5f);
        return GPixel_PackARGB(ia, ir, ig, ib);
    }

    GColor applyMatrix(const GColor& color) {
		float r = clamp(fMatrix[0] * color.r + fMatrix[1] * color.g + 
						fMatrix[2] * color.b + fMatrix[3] * color.a + fMatrix[4]);
		float g = clamp(fMatrix[5] * color.r + fMatrix[6] * color.g + 
						fMatrix[7] * color.b + fMatrix[8] * color.a + fMatrix[9]);
		float b = clamp(fMatrix[10] * color.r + fMatrix[11] * color.g + 
						fMatrix[12] * color.b + fMatrix[13] * color.a + fMatrix[14]);
		float a = clamp(fMatrix[15] * color.r + fMatrix[16] * color.g + 
						fMatrix[17] * color.b + fMatrix[18] * color.a + fMatrix[19]);

		return GColor::RGBA(r, g, b, a);
	}
    float clamp(float value) {
        return std::max(0.0f, std::min(1.0f, value));
    }
};

std::shared_ptr<GShader> MadiGFinal::createColorMatrixShader(const GColorMatrix& colorMatrix, 
                                                              GShader* realShader) {
    if (!realShader) {
        return nullptr;
    }
    return std::make_shared<ColorMatrixShader>(colorMatrix, realShader);
}

std::shared_ptr<GPath> MadiGFinal::strokePolygon(const GPoint points[], int count, float width, bool isClosed) {
    if (count < 2) return nullptr;
   
    GPathBuilder builder;
    float halfWidth = width/2;


    auto getNormal = [](GPoint p0, GPoint p1, float scale) -> std::pair<float, float> {
        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        float len = sqrt(dx*dx + dy*dy);
        if (len == 0) return {0, 0};
        return {(-dy / len) * scale, (dx / len) * scale};
    };


    auto [nx, ny] = getNormal(points[0], points[1], halfWidth);
    builder.moveTo({points[0].x + nx, points[0].y + ny});


    for (int i = 0; i < count - 1; i++) {
        GPoint curr = points[i];
        GPoint next = points[i + 1];
       
        auto [nx1, ny1] = getNormal(curr, next, halfWidth);
       
        builder.lineTo({next.x + nx1, next.y + ny1});
    }

    for (int i = count - 1; i > 0; i--) {
        GPoint curr = points[i];
        GPoint prev = points[i - 1];
       
        auto [nx, ny] = getNormal(prev, curr, halfWidth);
        builder.lineTo({curr.x - nx, curr.y - ny});
    }


    if (isClosed) {
        GPoint first = points[0];
        GPoint last = points[count-1];
        auto [nx, ny] = getNormal(last, first, halfWidth);
        builder.lineTo({first.x + nx, first.y + ny});
    } else {
        auto [nx, ny] = getNormal(points[0], points[1], halfWidth);
        builder.lineTo({points[0].x + nx, points[0].y + ny});
    }


    return builder.detach();
}

namespace {
    std::shared_ptr<GPath> createStrokedPath(GPoint start, GPoint end, float strokeWidth, bool roundedCaps) {
        float deltaX = end.x - start.x;
        float deltaY = end.y - start.y;
        float lineLength = sqrt(deltaX * deltaX + deltaY * deltaY);

        if (lineLength < 0.0001f) return nullptr;

        float normalX = (-deltaY / lineLength) * (strokeWidth / 2);
        float normalY = (deltaX / lineLength) * (strokeWidth / 2);

        GPoint p0 = {start.x + normalX, start.y + normalY};
        GPoint p1 = {start.x - normalX, start.y - normalY};
        GPoint p2 = {end.x - normalX, end.y - normalY};
        GPoint p3 = {end.x + normalX, end.y + normalY};

        GPathBuilder pathBuilder;
        pathBuilder.moveTo(p0);
        pathBuilder.lineTo(p1);
        pathBuilder.lineTo(p2);
        pathBuilder.lineTo(p3);

        return pathBuilder.detach();
    }
}

std::shared_ptr<GShader> MadiGFinal::createLinearPosGradient(GPoint start, GPoint end, const GColor gradientColors[], const float positions[], int numColors) {
    if (numColors < 1 || !gradientColors || !positions) {
        return nullptr;
    }

    std::vector<GColor> computedColors;

    for (float t = 0; t <= 1.0f; t += 0.01f) {
        int colorIndex = 0;
        while (colorIndex < numColors - 1 && t > positions[colorIndex + 1]) {
            colorIndex++;
        }

        GColor finalColor;
        if (t <= positions[0]) {
            finalColor = gradientColors[0];
        } else if (t >= positions[numColors - 1]) {
            finalColor = gradientColors[numColors - 1];
        } else {
            float startPos = positions[colorIndex];
            float endPos = positions[colorIndex + 1];
            float mixFactor = (t - startPos) / (endPos - startPos);

            const GColor& startColor = gradientColors[colorIndex];
            const GColor& endColor = gradientColors[colorIndex + 1];

            finalColor = {
                startColor.r + (endColor.r - startColor.r) * mixFactor,
                startColor.g + (endColor.g - startColor.g) * mixFactor,
                startColor.b + (endColor.b - startColor.b) * mixFactor,
                startColor.a + (endColor.a - startColor.a) * mixFactor
            };
        }
        computedColors.push_back(finalColor);
    }

    return GCreateLinearGradient(start, end, computedColors.data(), computedColors.size(), GTileMode::kClamp);
}

std::unique_ptr<GFinal> GCreateFinal() {
    return std::make_unique<MadiGFinal>();
}
