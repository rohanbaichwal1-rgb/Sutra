#include "DigitalFilter.h"

DigitalFilter::DigitalFilter(FilterType type, float samplingFreq, float filterFreq1)
{
    _type = type;

    _alpha = pow(
        DIGITAL_FILTER_E,
        -2.f * DIGITAL_FILTER_PI * filterFreq1 / samplingFreq
    );

    _nInitSamples = 0;
    _nPoles = 1;
}

float DigitalFilter::filter(float inputSample)
{
    if (_nInitSamples < _nPoles)
    {
        // Initialize filter
        _filteredValue = inputSample;
        _nInitSamples++;
    }

    if (_type == FilterType::IIR_LOWPASS)
    {
        _filteredValue =
            inputSample * (1. - _alpha) +
            _filteredValue * _alpha;

        return _filteredValue;
    }
    else if (_type == FilterType::IIR_HIGHPASS)
    {
        _filteredValue =
            inputSample * (1. - _alpha) +
            _filteredValue * _alpha;

        return inputSample - _filteredValue;
    }
    else
    {
        // Resolve for other filter types
        // GCC must have a return type at the end
        return 0.0;
    }
}