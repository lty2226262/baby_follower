#ifndef NON_LINEAR_SYSTEM_CONDITIONAL_GAUSSIAN_ODO_H
#define NON_LINEAR_SYSTEM_CONDITIONAL_GAUSSIAN_ODO_H

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{  
class NonLinearAnalyticConditionalGaussianOdo : public AnalyticConditionalGaussianAdditiveNoise{
public:
  /// Constructor
  /** @pre:  Every Matrix should have the same amount of rows!
      This is currently not checked.  The same goes for the number
      of columns, which should be equal to the number of rows of
      the corresponding conditional argument!
      @param additiveNoise Pdf representing the additive Gaussian uncertainty
  */
  NonLinearAnalyticConditionalGaussianOdo( const Gaussian& additiveNoise);

  /// Destructor
  virtual ~NonLinearAnalyticConditionalGaussianOdo();

  // redefine virtual functions
  virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
  virtual MatrixWrapper::Matrix          dfGet(unsigned int i)       const;

private:
  mutable MatrixWrapper::Matrix df;
};
}
#endif