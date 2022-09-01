#pragma once
#include <ILI9341_t3n.h>
#include <gfx_core.hpp>
#include <gfx_pixel.hpp>
#include <gfx_positioning.hpp>
class ili9341_t3n : public ILI9341_t3n {
public:
    using pixel_type = gfx::rgb_pixel<16>;
    using caps = gfx::gfx_caps<false,false,false,false,false,true,false>;

    inline gfx::gfx_result point(gfx::point16 location, pixel_type color) {
        drawPixel(location.x,location.y,color.native_value);
        return gfx::gfx_result::success;
    }
    inline gfx::gfx_result point(gfx::point16 location, pixel_type* out_color) {
        if(out_color==nullptr) {
            return gfx::gfx_result::invalid_argument;
        }
        
        out_color->native_value= readPixel(location.x,location.y);
        return gfx::gfx_result::success;
    }
    
};