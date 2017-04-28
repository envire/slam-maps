//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//


// Code available at: http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients

// TODO: this can be moved to vizkit3d

#pragma once

#include <vector>

namespace vizkit3d
{
    class ColorGradient
    {
    private:
        struct ColorPoint  // Internal class used to store colors at different points in the gradient.
        {
            float r,g,b;      // Red, green and blue values of our color.
            float val;        // Position of our color along the gradient (between 0 and 1).
            ColorPoint(float red, float green, float blue, float value)
                : r(red), 
                  g(green), 
                  b(blue), 
                  val(value) 
            {}
        };
    
        std::vector<ColorPoint> color;      // An array of color points in ascending value.

    public:
        
        //-- Default constructor:
        ColorGradient()  
        {  
            createDefaultHeatMapGradient();  
        }

        //-- Inserts a new color point into its correct position:
        void addColorPoint(float red, float green, float blue, float value)
        {
            for(int i = 0; i < static_cast<int>(color.size()); i++)  
            {
                if(value < color[i].val) 
                {
                    color.insert(color.begin() + i, ColorPoint(red,green,blue, value));
                    return;  
                }
            }
            color.push_back(ColorPoint(red,green,blue, value));
        }

        //-- Inserts a new color point into its correct position:
        void clearGradient() 
        { 
            color.clear(); 
        }

        //-- Places a 5 color heapmap gradient into the "color" vector:
        void createDefaultHeatMapGradient()
        {
            color.clear();
            color.push_back(ColorPoint(0, 0, 0,   0.0f));      // black.
            color.push_back(ColorPoint(0, 0, 1,   0.1428f));   // Blue.
            color.push_back(ColorPoint(0, 1, 1,   0.2856f));   // Cyan.
            color.push_back(ColorPoint(0, 1, 0,   0.4284f));   // Green.
            color.push_back(ColorPoint(1, 1, 0,   0.5712f));   // Yellow.
            color.push_back(ColorPoint(1, 0, 0,   0.7140f));   // Red.
            color.push_back(ColorPoint(1, 1, 1,   1.0f));      // White.
        }

        //-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
        //-- values representing that position in the gradient.
        void getColorAtValue(const float value, float &red, float &green, float &blue) const
        {
            if(color.size() == 0)
                return;

            for(int i = 0; i < static_cast<int>(color.size()); i++)
            {
                const ColorPoint &currC = color[i];
                if(value < currC.val)
                {
                    const ColorPoint &prevC  = color[ std::max(0, i - 1) ];
                    float valueDiff    = (prevC.val - currC.val);
                    float fractBetween = (valueDiff == 0) ? 0 : (value - currC.val) / valueDiff;
                    red   = (prevC.r - currC.r) * fractBetween + currC.r;
                    green = (prevC.g - currC.g) * fractBetween + currC.g;
                    blue  = (prevC.b - currC.b) * fractBetween + currC.b;
                    return;
                }
            }
            
            red   = color.back().r;
            green = color.back().g;
            blue  = color.back().b;

            return;
        }
    };
}
