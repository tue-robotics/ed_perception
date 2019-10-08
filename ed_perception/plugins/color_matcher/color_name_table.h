#ifndef _COLOR_TABLE_H_
#define _COLOR_TABLE_H_

#include <string>
#include <vector>

#include <iostream>

class ColorNameTable
{

public:

    enum Color
    {
        BLACK = 0,
        BLUE = 1,
        BROWN = 2,
        GREY = 3,
        GREEN = 4,
        ORANGE = 5,
        PINK = 6,
        PURPLE = 7,
        RED = 8,
        WHITE = 9,
        YELLOW = 10
    };

    static int NUM_COLORS;

    ColorNameTable();

    ~ColorNameTable();

    bool readFromFile(const std::string& filename);

    inline const float* rgbToDistribution(int r, int g, int b) const
    {
        return &table_[NUM_COLORS * ((r / STEP) + (RESOLUTION * ((g / STEP) + (RESOLUTION * (b / STEP)))))];
    }

    static const char* intToColorName(int i);

private:

    static int RESOLUTION;
    static int STEP;

    std::vector<float> table_;

};

#endif
