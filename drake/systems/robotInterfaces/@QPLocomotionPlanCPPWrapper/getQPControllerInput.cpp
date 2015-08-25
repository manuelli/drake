#include "QPLocomotionPlan.h"
#include "drakeMexUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs != 5 || nlhs != 1) {
    mexErrMsgTxt("usage: lcm_msg_data = getQPControllerInput(obj, t_global, x, contact_force_detected, byte_array);");
  }

  QPLocomotionPlan* plan = (QPLocomotionPlan*) getDrakeMexPointer(mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  double t_global = mxGetScalar(prhs[1]);
  int nq = plan->getRobot().num_positions;
  int nv = plan->getRobot().num_velocities;
  auto q = Map<const VectorXd>(mxGetPrSafe(prhs[2]), nq);
  auto v = Map<const VectorXd>(mxGetPrSafe(prhs[2]) + nq, nv);
  auto contact_force_detected = matlabToStdVector<bool>(prhs[3]);


  shared_ptr<drc::robot_state_t> robot_state(new drc::robot_state_t());
  const mxArray* lcm_message_mex = prhs[4];
  if (!mxIsInt8(lcm_message_mex))
    mexErrMsgTxt("Expected an int8 array as the qp_input argument");
  robot_state->decode(mxGetData(lcm_message_mex), 0, mxGetNumberOfElements(prhs[narg]));



  drake::lcmt_qp_controller_input qp_controller_input = plan->createQPControllerInput(t_global, q, v, contact_force_detected, robot_state);
  const size_t size = qp_controller_input.getEncodedSize();
  plhs[0] = mxCreateNumericMatrix(size, 1, mxUINT8_CLASS, mxREAL);
  qp_controller_input.encode(mxGetData(plhs[0]), 0, static_cast<int>(size));
}
