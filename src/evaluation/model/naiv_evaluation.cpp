#include <tsdf_localization/evaluation/model/naiv_evaluation.h>

#include <cmath>

namespace tsdf_localization
{

NaivEvaluation::NaivEvaluation() : m_error(0.0)
{

}

void NaivEvaluation::insertMeasurement(FLOAT_T measurement)
{
  m_error += fabs(measurement);
}

void NaivEvaluation::insertMeasurement(FLOAT_T observed_range, FLOAT_T simulated_range)
{
  //m_error += error;
  insertMeasurement(observed_range - simulated_range);
}

FLOAT_T NaivEvaluation::evaluate()
{
  return 1 / (1 + m_error);
}

void NaivEvaluation::reset()
{
  m_error = 0.0;
}

} // namespace tsdf_localization