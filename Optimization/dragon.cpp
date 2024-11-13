#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


// TODO: Implement the cost function
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& p_, const Point2D& q_, const Weight& w_)
		: p(p_), q(q_), w(w_)
	{
	}

	template<typename T>
	bool operator()(const T* const tx, const T* const ty, const T* const angle, T* residual) const
	{
		auto cosAngle = cos(*angle);
		auto sinAngle = sin(*angle);
		auto diff_x = T(p.x) * cosAngle - T(p.y) * sinAngle + *tx - T(q.x);
		auto diff_y = T(p.x) * sinAngle + T(p.y) * cosAngle + *ty - T(q.y);
		residual[0] = T(w.w) * (diff_x * diff_x + diff_y * diff_y);
		return true;
	}

	private:
		const Point2D p;
		const Point2D q;
		const Weight w;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "../../data/points_dragon_1.txt";
	const std::string file_path_2 = "../../data/points_dragon_2.txt";
	const std::string file_path_weights = "../../data/weights_dragon.txt";
	const auto points_1 = read_points_from_file<Point2D>(file_path_1);
	const auto points_2 = read_points_from_file<Point2D>(file_path_2);
	const auto weights = read_points_from_file<Weight>(file_path_weights);

	// Good initial values make the optimization easier
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;
	const double angle_initial = 0.0;

	double tx = tx_initial;
	double ty = ty_initial;
	double angle = angle_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block
	for(int i = 0; i < points_1.size(); i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
				new RegistrationCostFunction(points_1[i], points_2[i], weights[i])),
			nullptr, &tx, &ty, &angle
		);
	}


	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	auto angle_deg = angle * 180.0 / M_PI;

	// TODO: Output the final values of the translation and rotation (in degree)
	std::cout << "Initial tx: " << tx_initial << "\tty: " << ty_initial << "\tangle: " << angle_initial << std::endl;
	std::cout << "Final tx: " << tx << "\tty: " << ty << "\tangle: " << angle_deg << std::endl;

	system("pause");
	return 0;
}