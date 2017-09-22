#ifndef SIMULATION_NORMAL_DEPTH_MAP_SRC_TOOLS_HPP_
#define SIMULATION_NORMAL_DEPTH_MAP_SRC_TOOLS_HPP_

namespace normal_depth_map {

  /**
   * @brief compute Underwater Signal Attenuation coefficient
   *
   * This method is based on paper "Development of acoustic camera-imaging
   *  simulator based on novel model". The method computes the attenuation
   *  coefficient that will be used on shader normal intensite return.
   *
   *  @param double temperature: water temperature in celsius.
   *  @param double frequency: sound frequency in kHz.
   *  @param double depth_rate: distance from water suface in meters.
   *
   *  @return double coefficient attenuation value
   */

  double underwaterSignalAttenuation( const double temperature,
                                      const double frequency,
                                      const double depth_rate);
}

#endif
