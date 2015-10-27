#include "color_name_table.h"

#include <fstream>

int ColorNameTable::RESOLUTION = 32;
int ColorNameTable::NUM_COLORS = 11;
int ColorNameTable::STEP = 256 / RESOLUTION;


// ----------------------------------------------------------------------------------------------------

ColorNameTable::ColorNameTable()
{
}

// ----------------------------------------------------------------------------------------------------

ColorNameTable::~ColorNameTable()
{
}

// ----------------------------------------------------------------------------------------------------

bool ColorNameTable::readFromFile(const std::string& filename)
{
    std::fstream file(filename.c_str(), std::ios_base::in);

    if (!file.is_open())
        return false;

    table_.resize(RESOLUTION * RESOLUTION * RESOLUTION * NUM_COLORS);

    int k = 0;
    for(int b = 0; b < RESOLUTION; ++b)
    {
        for(int g = 0; g < RESOLUTION; ++g)
        {
            for(int r = 0; r < RESOLUTION; ++r)
            {
                // Skip first three values
                float v1, v2, v3;
                file >> v1 >> v2 >> v3;

                for(int i = 0; i < NUM_COLORS; ++i)
                    file >> table_[k + i];

                k += NUM_COLORS;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

const char* ColorNameTable::intToColorName(int i)
{
    static const char* color_names[11] = { "black", "blue", "brown", "grey", "green", "orange", "pink", "purple", "red", "white", "yellow" };
    return color_names[i];
}

// ----------------------------------------------------------------------------------------------------

