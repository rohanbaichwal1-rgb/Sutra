#pragma once

#include <math.h>

#define DIGITAL_FILTER_PI 3.1415926535897932384626433832795
#define DIGITAL_FILTER_E 2.7182818284590452353602874713526

class DigitalFilter
{
public:
    enum class FilterType
    {
        IIR_LOWPASS,
        IIR_HIGHPASS
    };

private:
    float _filteredValue;
    float _alpha;
    FilterType _type;
    int _nInitSamples;
    int _nPoles;

public:
    DigitalFilter(FilterType type, float samplingFreq, float filterFreq1);
    float filter(float inputSample);
};