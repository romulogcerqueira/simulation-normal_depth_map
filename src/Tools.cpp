#include "Tools.hpp"

#include <cmath>

namespace normal_depth_map {

double underwaterSignalAttenuation(
                                const double temperature,
                                const double frequency,
                                const double depth_rate) {

    // compute absorptive attenuation coefficient in db/km
    double calc_depth = depth_rate / 17000;
    double calc_temp = temperature / 27.0;
    double alpha = 0.00049 * frequency * frequency;
    alpha = alpha * exp( -( calc_temp + calc_depth ) );

    // convert db/km to db/m
    alpha = alpha/1000.0;

    // convert to pascal per meter (Pa/m)
    alpha = pow( 10, -alpha/20.0 );
    alpha = -log( alpha );

    return alpha;
}

}
