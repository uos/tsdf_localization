#include <tsdf_localization/evaluation/model/omp_likelihood_evaluation.h>

#include <omp.h>

#include <cmath>
#include <iostream>

namespace tsdf_localization
{

OMPLikelihoodEvaluation::OMPLikelihoodEvaluation(FLOAT_T range_max) : m_range_max_(range_max)
{
  m_sum_.fill(0);
}

void OMPLikelihoodEvaluation::insertMeasurement(FLOAT_T measurement)
{
  FLOAT_T eval = 0.0;

  //eval += 0.5 * exp(-measurement * measurement / 0.08);
  //eval += 0.5 / m_range_max_;

  m_sum_[omp_get_thread_num()] += eval * eval * eval;
}

void OMPLikelihoodEvaluation::insertMeasurement(FLOAT_T observed_range, FLOAT_T simulated_range)
{
  if(observed_range >= m_range_max_)
  {
    return;
  }

  insertMeasurement(observed_range - simulated_range);
}

FLOAT_T OMPLikelihoodEvaluation::evaluate()
{
  return m_sum_[omp_get_thread_num()];
}

void OMPLikelihoodEvaluation::reset()
{
  m_sum_[omp_get_thread_num()] = 0;
}

} // namespace tsdf_localization