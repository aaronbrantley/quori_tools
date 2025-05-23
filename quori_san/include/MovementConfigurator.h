#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

class MovementConfigurator
{
	private:
		// https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/?answer=64043#post-id-64043
		dynamic_reconfigure::ReconfigureRequest request;
		dynamic_reconfigure::ReconfigureResponse response;
		dynamic_reconfigure::DoubleParameter parameter;
		dynamic_reconfigure::Config config;

		std::string parameterName;
		double parameterValue;

	public:
		void setAccelerationLimit (char type, double limit)
		{
			switch (type)
			{
				case 'x':
				{
					parameterName = "acc_lim_x";

					break;
				}

				case 'y':
				{
					parameterName = "acc_lim_y";

					break;
				}

				case 't':
				{
					parameterName = "acc_lim_th";

					break;
				}
			}

			parameter.name = parameterName;
			parameter.value = limit;
			config.doubles.push_back (parameter);
			request.config = config;
			ros::service::call ("move_base/TrajectoryPlannerROS/set_parameters", request, response);
		}

		void setVelocityLimit (char type, double limit)
		{
			switch (type)
			{
				case 'x':
				{
					parameterName = "max_vel_x";

					break;
				}

				case 'y':
				{
					parameterName = "max_vel_y";

					break;
				}

				case 'r':
				{
					parameterName = "max_rot_vel";

					break;
				}
			}

			parameter.name = parameterName;
			parameter.value = limit;
			config.doubles.push_back (parameter);
			request.config = config;
			ros::service::call ("move_base/TrajectoryPlannerROS/set_parameters", request, response);
		}
};
