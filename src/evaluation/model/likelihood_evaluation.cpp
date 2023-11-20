#include <tsdf_localization/evaluation/model/likelihood_evaluation.h>

#include <cmath>
#include <iostream>

namespace tsdf_localization
{

LikelihoodEvaluation::LikelihoodEvaluation(FLOAT_T range_max) : m_sum(0.0), m_range_max(range_max)
{

}

void LikelihoodEvaluation::insertMeasurement(FLOAT_T measurement)
{ 
  FLOAT_T eval = 0.0;

  //eval += 0.5 * exp(-measurement * measurement / 0.08);
  //eval += 0.5 / m_range_max;

  eval = exp(-(measurement * measurement) / SIGMA_QUAD / 2) / (sqrt(2 * SIGMA_QUAD * M_PI));

  m_sum += eval * eval;
}

void LikelihoodEvaluation::insertMeasurement(FLOAT_T observed_range, FLOAT_T simulated_range)
{
  if(observed_range >= m_range_max)
  {
    return;
  }

  insertMeasurement(observed_range - simulated_range);
}

FLOAT_T LikelihoodEvaluation::evaluate()
{
  return m_sum;
}

void LikelihoodEvaluation::reset()
{
  m_sum = 0.0;
}

} // namespace tsdf_localization