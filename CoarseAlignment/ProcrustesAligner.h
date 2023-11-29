#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);
		
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// Note: The final translation is not equal to the translation of the means. Refer to the exercise sheet for more details.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;

		std::cout << estimatedPose << "\n";
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		Vector3f mean = Vector3f::Zero();
		Vector3f sum = Vector3f::Zero();

		for(Vector3f point : points)
			sum += point;
		
		mean = sum / points.size();
		std::cout << "Mean: " << "[" << mean(0) << ", " << mean(1) << ", " << mean(2) << "]" << "\n";
		return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.
		Matrix3f rotation = Matrix3f::Identity();
		int N = sourcePoints.size();
		MatrixXf X(3, N);
		MatrixXf Y(3, N);

		// First compute mean-centered points
		for(int i = 0; i < N; i++) {
			X.block(0, i, 3, 1) = sourcePoints[i] - sourceMean;
			Y.block(0, i, 3, 1) = targetPoints[i] - targetMean;
		}

		// Now use SVD to find the rotation matrix
		MatrixXf X_Yt = X * Y.transpose();
		JacobiSVD<MatrixXf> svd(X_Yt, ComputeThinU  | ComputeThinV);
		svd.compute(X_Yt);
		// R = VUt
		rotation = svd.matrixV() * svd.matrixU().transpose();

		return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.
		Vector3f translation = Vector3f::Zero();
		
		// t = y - Rx
		translation = targetMean - (rotation * sourceMean);

		return translation;
	}
};