#pragma once
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cassert>
using namespace nanoflann;

namespace netease {
	struct Correspondence {
		int index_query;
		int index_match;
		double distance;
		Correspondence() {}
		Correspondence(int index_query_, int index_match_, double distance_) :index_query(index_query_), index_match(index_match_), distance(distance_) {}
	};

	struct icpResult {
		int iterations;
		double MSE;
		double epsilon;
		icpResult(int iterations_, double MSE_, double epsilon_) : iterations(iterations_), MSE(MSE_), epsilon(epsilon_) {}
	};

	//template <class MatrixType, class Distance = nanoflann::metric_L2>
	//struct KDTreeEigenMatrixAdaptor{}
	typedef KDTreeEigenMatrixAdaptor<Eigen::Matrix<double, Eigen::Dynamic, 3>, nanoflann::metric_L2_Simple> KDTree; //3是每一个实例的维度，可以改为Eigen::Dynamic，如果你不知道输入维度

	class icp {

	public:
		icp(const Eigen::Matrix<double, Eigen::Dynamic, 3>& _source, const Eigen::Matrix<double, Eigen::Dynamic, 3>& _target);
		~icp();
		Eigen::Matrix<double, 4, 4> compute();
		void estimateRigidTransformation(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, const Eigen::Matrix<double, Eigen::Dynamic, 3>& target, const std::vector<Correspondence>& correspondences, Eigen::Matrix<double, 4, 4>& transformation);
		void determineCorrespondences(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, std::vector<Correspondence>& correspondences);
		void compute3DCentroid(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, Eigen::Matrix<double, 1, 3>& centroid);
		void demeanPointCloud(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, const Eigen::Matrix<double, 1, 3>& centroid, Eigen::Matrix<double, Eigen::Dynamic, 3>& source_demean);
		void transformCloud(Eigen::Matrix<double, Eigen::Dynamic, 3>& source, const Eigen::Matrix<double, 4, 4>& transformation, Eigen::Matrix<double, Eigen::Dynamic, 3>& destination);
		//void EigenMatrixToPointCloud(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, PointCloud::Ptr& destination);
		void EigenMatrixToTxt(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, const std::string& file_path);
		//void removeOutlier(std::vector<Correspondence>& correspondences);
		void setMaximumIterations(const int iteration_);
		void setEuclideanFitnessEpsilon(const double epsilon_);
		void setSourceSamplingProb(const double samplingProb_);
		void setTargetSamplingProb(const double samplingProb_);
		void setMedianDistOutlierFilterFactor(const double factor_);
		void setTrimmedDistOutlierFilterRatio(const double ratio_);
		void setMaxDistOutlierFilterDist(const double dist_);
		void sourceRandomSamplingFilter();
		void targetRandomSamplingFilter();
		void medianDistOutlierFilter(std::vector<Correspondence>& correspondences);
		void trimmedDistOutlierFilter(std::vector<Correspondence>& correspondences);
		void maxDistOutlierFilter(std::vector<Correspondence>& correspondences);
		bool hasConverged();
		icpResult getResult();
	private:
		Eigen::Matrix<double, Eigen::Dynamic, 3> source;
		Eigen::Matrix<double, Eigen::Dynamic, 3> target;
		KDTree* treeIndex;
		Eigen::Matrix<double, 4, 4> transformation;
		double sourceSamplingProb, targetSamplingProb; //input source and target download sampling.
		double medianDistOutlierFilterFactor;
		double trimmedDistOutlierFilterRatio;
		double maxDistOutlierFilterDist;
		int npts;
		int iteration;
		int max_iteration;
		double epsilon;  //The error is estimated as the sum of the differences between correspondences in an Euclidean sense, divided by the number of correspondences.
		double current_MSE, previous_MSE;
	};
}