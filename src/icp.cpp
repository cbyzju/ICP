#include "icp.h"
const size_t queryNum = 1;         //找到里查询点最近的前queryNum个点


namespace netease {
	bool correSort(Correspondence a, Correspondence b)
	{
		return a.distance < b.distance;
	}

	icp::icp(const Eigen::Matrix<double, Eigen::Dynamic, 3>& _target, const Eigen::Matrix<double, Eigen::Dynamic, 3>& _source)
	{
		target = _target;
		source = _source;
		iteration = 0;
		max_iteration = 100;
		epsilon = 1e-7;
		sourceSamplingProb = 0;
		targetSamplingProb = 0;
		medianDistOutlierFilterFactor = 0;
		trimmedDistOutlierFilterRatio = 0;
		maxDistOutlierFilterDist = 0;
	}

	icp::~icp() {}

	Eigen::Matrix<double, 4, 4> icp::compute()
	{
		//Eigen::Matrix<double, 1, 3>  centroid_tgt;
		//compute3DCentroid(target, centroid_tgt);
		//Eigen::Matrix<double, Eigen::Dynamic, 3> src_demean, tgt_demean;
		//demeanPointCloud(source, centroid_tgt, src_demean); // reference is express in frame <refMean>
		//demeanPointCloud(target, centroid_tgt, tgt_demean); // reading is express in frame <refMean>

		sourceRandomSamplingFilter(); //source数据点降采样
		targetRandomSamplingFilter(); //target数据点降采样

		treeIndex = new KDTree(target, 5); //5指的是构造完树时，每个根节点最多包含的点数为10个，这是一个trade-off, 构造树时，该阈值大，构造过程快，但查询时慢，反之亦然。
		//treeIndex->index->buildIndex();

		Eigen::Matrix<double, Eigen::Dynamic, 3> destination;
		transformation.setIdentity();
		Eigen::Matrix<double, 4, 4> step_transformation;
		transformCloud(source, transformation, destination);

		//EigenMatrixToTxt(destination, "query.txt");
		//EigenMatrixToTxt(tgt_demean, "match.txt");
		//std::cout << "destination size = "<<destination.rows() << std::endl;
		do
		{
			std::vector<Correspondence> correspondences;
			determineCorrespondences(destination, correspondences);
			trimmedDistOutlierFilter(correspondences);
			medianDistOutlierFilter(correspondences);
			maxDistOutlierFilter(correspondences);
			//removeOutlier(correspondences);
			//std::cout << "correspondences size = "<<correspondences.size() << std::endl;
			//for (int i = 0; i < correspondences.size(); i += 1000)
			//for (int i = 90; i < 105; i++)
			//	std::cout <<"query = "<<correspondences[i].index_query <<"[" <<destination(correspondences[i].index_query, 0)<<", "<<destination(correspondences[i].index_query, 1)<<"， "<< destination(correspondences[i].index_query, 2)
			//	<<"], match = " << correspondences[i].index_match <<"["<< tgt_demean(correspondences[i].index_match, 0 )<<", "<< tgt_demean(correspondences[i].index_match, 1) << ", " << tgt_demean(correspondences[i].index_match, 2)
			//	<<"], distance = " << correspondences[i].distance << std::endl;
			//system("pause");
			//return transformation;

			estimateRigidTransformation(destination, target, correspondences, step_transformation);
			transformCloud(destination, step_transformation, destination);

			transformation = step_transformation*transformation;
			//std::cout << "iteration = " << iteration << ", transformation = \n" << transformation << std::endl;
			//std::cout << "step transformation:\n" << step_transformation << std::endl;
			iteration++;
		} while (hasConverged());

		//Eigen::Matrix<double, 4, 4> T_refIn_refMean, T_refMean_dataIn;
		//T_refIn_refMean.setIdentity();
		//T_refMean_dataIn.setIdentity();
		//T_refIn_refMean.block(0, 3, 3, 1) = centroid_tgt.transpose();
		//T_refMean_dataIn.block(0, 3, 3, 1) = - centroid_tgt.transpose();
		//transformation = T_refIn_refMean*transformation*T_refMean_dataIn;

		//std::cout <<"T_refIn_refMean:\n"<< T_refIn_refMean << std::endl;

		//PointCloud::Ptr cloud_transformed(new PointCloud());
		//PointCloud::Ptr	cloud_tgt_demean(new PointCloud());
		//PointCloud::Ptr cloud_src(new PointCloud());
		//PointCloud::Ptr	cloud_tgt(new PointCloud());
		//EigenMatrixToPointCloud(destination, cloud_transformed);
		//EigenMatrixToPointCloud(tgt_demean, cloud_tgt_demean);
		//EigenMatrixToPointCloud(target, cloud_tgt);
		//EigenMatrixToPointCloud(source, cloud_src);
		//pcl::io::savePCDFile("target.pcd", *cloud_tgt_demean);
		//pcl::io::savePCDFile("source.pcd", *cloud_src_demean);

		return transformation;

	}

	void icp::estimateRigidTransformation(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source_, const Eigen::Matrix<double, Eigen::Dynamic, 3>& target_, const std::vector<Correspondence>& correspondences, Eigen::Matrix<double, 4, 4>& transformation)
	{
		transformation.setIdentity();
		const int npts = source.rows();
		Eigen::Matrix<double, Eigen::Dynamic, 3> cloud_src(npts, 3);
		Eigen::Matrix<double, Eigen::Dynamic, 3> cloud_tgt(npts, 3);

		//cout << cloud_src.rows() <<" "<<cloud_src.cols() << endl;
		//cout << cloud_tgt.rows() <<" "<<cloud_tgt.cols() << endl;
		//cout << "correspondences = "<<correspondences.size() << endl;
		//#pragma omp parallel for
		for (int i = 0; i < npts; ++i)
		{
			cloud_src(i, 0) = source_(i, 0);
			cloud_src(i, 1) = source_(i, 1);
			cloud_src(i, 2) = source_(i, 2);

			cloud_tgt(i, 0) = target_(correspondences[i].index_match, 0);
			cloud_tgt(i, 1) = target_(correspondences[i].index_match, 1);
			cloud_tgt(i, 2) = target_(correspondences[i].index_match, 2);
		}

		Eigen::Matrix<double, 1, 3> centroid_src, centroid_tgt;
		compute3DCentroid(cloud_src, centroid_src);
		compute3DCentroid(cloud_tgt, centroid_tgt);
		Eigen::Matrix<double, Eigen::Dynamic, 3> cloud_src_demean, cloud_tgt_demean;
		demeanPointCloud(cloud_src, centroid_src, cloud_src_demean);
		demeanPointCloud(cloud_tgt, centroid_tgt, cloud_tgt_demean);

		//cout << "centroid_src:\n" << centroid_src << endl;
		//cout << "centroid_tgt:\n" << centroid_tgt << endl;

		//for (int i = 0; i < 50; ++i)
		//{
		//	printf("cloud_src[%d] = [%f, %f, %f]\n", i, cloud_src(i, 0), cloud_src(i, 1), cloud_src(i, 2));
		//printf("cloud_src_demean[%d] = [%f, %f, %f]\n", i, cloud_src_demean(i, 0), cloud_src_demean(i, 1), cloud_src_demean(i, 2));
		//	printf("cloud_tgt[%d] = [%f, %f, %f]\n", correspondences[i].index_match, cloud_tgt(i, 0), cloud_tgt(i, 1), cloud_tgt(i, 2));
		///	printf("distance = %f", correspondences[i].distance);
		//printf("cloud_tgt_demean[%d] = [%f, %f, %f]\n", correspondences[i].index_match, cloud_tgt_demean(i, 0), cloud_tgt_demean(i, 1), cloud_tgt_demean(i, 2));
		//}

		/*
		for (int i = 1; i < 1000; ++i)
		{
		printf("cloud_src_demean[%d] = [%f, %f, %f]\n", i, cloud_src_demean(i, 0), cloud_src_demean(i, 1), cloud_src_demean(i, 2));
		printf("cloud_tgt_demean[%d] = [%f, %f, %f]\n", correspondences[i].index_match, cloud_tgt_demean(i, 0), cloud_tgt_demean(i, 1), cloud_tgt_demean(i, 2));
		}
		*/

		Eigen::Matrix<double, 3, 3> H = cloud_src_demean.transpose() * cloud_tgt_demean;
		//cout << "correlation matrix: " << endl;
		//cout << H << endl;
		// Compute the Singular Value Decomposition
		Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix<double, 3, 3> u = svd.matrixU();
		Eigen::Matrix<double, 3, 3> v = svd.matrixV();

		// Compute R = V * U'
		if (u.determinant() * v.determinant() < 0)
		{
			for (int x = 0; x < 3; ++x)
				v(x, 2) *= -1;
		}

		Eigen::Matrix<double, 3, 3> R = v * u.transpose();

		transformation.topLeftCorner(3, 3) = R;
		const Eigen::Matrix<double, 3, 1> Rc(R * centroid_src.transpose());
		transformation.block(0, 3, 3, 1) = centroid_tgt.transpose() - Rc;
	}

	void icp::transformCloud(Eigen::Matrix<double, Eigen::Dynamic, 3>& source, const Eigen::Matrix<double, 4, 4>& transformation, Eigen::Matrix<double, Eigen::Dynamic, 3>& destination)
	{
		destination.resize(source.rows(), 3);
		Eigen::Matrix<double, 3, 3> R = transformation.block(0, 0, 3, 3);
		Eigen::Matrix<double, 3, 1> T = transformation.block(0, 3, 3, 1);
		//#pragma omp parallel for
		for (int i = 0; i < source.rows(); ++i)
			destination.row(i) = (R*source.row(i).transpose() + T).transpose();

	}

	void icp::compute3DCentroid(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, Eigen::Matrix<double, 1, 3>& centroid)
	{
		centroid = source.colwise().sum() / source.rows();
		//centroid.setZero();
		//for (int i = 0; i < source.rows(); ++i)
		//{
		//	centroid(0, 0) += source(i, 0);
		//	centroid(0, 1) += source(i, 1);
		//	centroid(0, 2) += source(i, 2);
		//}
		//centroid /= source.rows();
	}

	void icp::demeanPointCloud(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source_, const Eigen::Matrix<double, 1, 3>& centroid, Eigen::Matrix<double, Eigen::Dynamic, 3>& source_demean)
	{
		source_demean.resize(source_.rows(), 3);
		//#pragma omp parallel for
		for (int i = 0; i < source_.rows(); ++i)
		{
			source_demean(i, 0) = source_(i, 0) - centroid(0, 0);
			source_demean(i, 1) = source_(i, 1) - centroid(0, 1);
			source_demean(i, 2) = source_(i, 2) - centroid(0, 2);
		}
	}

	void icp::determineCorrespondences(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, std::vector<Correspondence>& correspondences)
	{
		std::vector<size_t> indexResult(queryNum);               //存储index
		std::vector<float> squaredistResult(queryNum);           //存储和查询点的平方距离。
		nanoflann::KNNResultSet<float> resultSet(queryNum);
		resultSet.init(&indexResult[0], &squaredistResult[0]);

		//Correspondence temp_correspondence;
		current_MSE = 0;
		//#pragma omp parallel for reduction(+:current_MSE)
		for (int i = 0; i < source.rows(); ++i)
		{
			double query_pt[3] = { source(i, 0), source(i, 1), source(i, 2) };      //输入的查询数据点
			indexResult.clear();
			squaredistResult.clear();
			resultSet.init(&indexResult[0], &squaredistResult[0]);
			treeIndex->index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
			correspondences.push_back(Correspondence(i, indexResult[0], squaredistResult[0]));
			current_MSE += squaredistResult[0];
		}
		current_MSE /= source.rows();
	}

	/*
	void icp::removeOutlier(std::vector<Correspondence>& correspondences)
	{
		if (outlierPercentage == 0) return;
		std::sort(correspondences.begin(), correspondences.end(), correSort);
		int removeSize = round(correspondences.size()*outlierPercentage);
		correspondences.erase(correspondences.end() - removeSize, correspondences.end());
	}
	*/

	/*
	void icp::EigenMatrixToPointCloud(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, PointCloud::Ptr& destination)
	{
	int rows = source.rows();
	destination->height = 1;
	destination->width = rows;
	destination->points.resize(rows);
	for (int i = 0; i < rows; ++i)
	{
	PointT point;
	point.x = source(i, 0);
	point.y = source(i, 1);
	point.z = source(i, 2);
	destination->points[i] = point;
	}
	}
	*/

	void icp::EigenMatrixToTxt(const Eigen::Matrix<double, Eigen::Dynamic, 3>& source, const std::string& file_path)
	{
		std::ofstream outfile;
		outfile.open(file_path, std::ios::out);
		if (!outfile) { std::cout << "failed to open " << file_path << std::endl; return; }
		//clock_t start = clock();
		//#pragma omp parallel for
		for (int i = 0; i < source.rows(); i++)
		{
			outfile << source(i, 0) << " " << source(i, 1) << " " << source(i, 2) << std::endl;
		}
		//clock_t end = clock();
		//std::cout << "cost time = " << (float(end - start)) / CLOCKS_PER_SEC << std::endl;
		outfile.close();
	}

	void icp::setMaximumIterations(const int iteration_)
	{
		max_iteration = iteration_;
	}

	void icp::setEuclideanFitnessEpsilon(const double epsilon_)
	{
		epsilon = epsilon_;
	}

	bool icp::hasConverged()
	{
		if (iteration >= max_iteration)
			return false;

		if (iteration == 1 || fabs(current_MSE - previous_MSE) > epsilon)
		{
			//std::cout << previous_MSE << std::endl;
			previous_MSE = current_MSE;
			return true;
		}
		return false;
	}

	icpResult icp::getResult()
	{
		icpResult result(iteration, current_MSE, abs(current_MSE - previous_MSE));
		return result;
	}

	void icp::setSourceSamplingProb(const double samplingProb_)
	{
		sourceSamplingProb = samplingProb_;
	}

	void icp::setTargetSamplingProb(const double samplingProb_)
	{
		targetSamplingProb = samplingProb_;
	}

	void icp::setMedianDistOutlierFilterFactor(const double factor_)
	{
		medianDistOutlierFilterFactor = factor_;
		printf("\n----------MedianDistOutlierFilter----------\n keep factor = %f\n", medianDistOutlierFilterFactor);
	}

	void icp::setTrimmedDistOutlierFilterRatio(const double ratio_)
	{
		trimmedDistOutlierFilterRatio = ratio_;
		printf("\n----------trimmedDistOutlierFilter-----------\nkeep ratio = %f\n", trimmedDistOutlierFilterRatio);
	}

	void icp::setMaxDistOutlierFilterDist(const double dist_)
	{
		maxDistOutlierFilterDist = dist_;
		printf("\n------------MaxDistOutlierFilter--------------\nkeep dist = %f\n", maxDistOutlierFilterDist);
	}

	void icp::sourceRandomSamplingFilter()
	{
		if (sourceSamplingProb <= 0 || sourceSamplingProb >= 1) return; //Prob不在(0, 1)范围内的，默认不做降采样处理
		printf("\n----------Source Point Downsampling----------\noriginal size = %d\n", source.rows());
		int j = 0;
#pragma omp parallel for
		for (int i = 0; i < source.rows(); ++i)
		{
			const double r = (float)std::rand() / (float)RAND_MAX;
			if (r < sourceSamplingProb)
			{
#pragma omp critical
				{
					source.row(j) = source.row(i);
					j++;
				}		
			}
		}
		source.conservativeResize(j, 3);
		printf("current size = %d\n", source.rows());
	}

	void icp::targetRandomSamplingFilter()
	{
		if (targetSamplingProb <= 0 || targetSamplingProb >= 1) return; //Prob不在(0, 1)范围内的，默认不做降采样处理
		printf("\n----------Target Point Downsampling----------\noriginal size = %d\n", target.rows());
		int j = 0;
		for (int i = 0; i < target.rows(); ++i)
		{
			const double r = (float)std::rand() / (float)RAND_MAX;
			if (r < targetSamplingProb)
			{
				target.row(j) = target.row(i);
				j++;
			}
		}
		target.conservativeResize(j, 3);
		printf("current size = %d\n", target.rows());
	}

	void icp::medianDistOutlierFilter(std::vector<Correspondence>& correspondences)
	{
		if (medianDistOutlierFilterFactor < 0.01 || medianDistOutlierFilterFactor>10) return;
		//printf("\n#iteration %d - correspondence size before medianDistOutlierFilter = %d\n", iteration, correspondences.size());
		std::nth_element(correspondences.begin(), correspondences.begin() + correspondences.size()*0.5, correspondences.end(), correSort);
		double medianVal = medianDistOutlierFilterFactor*correspondences[correspondences.size()*0.5].distance;
		int j = 0;
		for (int i = 0; i < correspondences.size(); ++i)
		{
			if (correspondences[i].distance < medianVal)
			{
				correspondences[j] = correspondences[i];
				j++;
			}
		}
		npts = j;
		//printf("#iteration %d - correspondence size after medianDistOutlierFilter = %d\n", iteration, npts);
	}

	void icp::trimmedDistOutlierFilter(std::vector<Correspondence>& correspondences)
	{
		if (trimmedDistOutlierFilterRatio <= 0 || trimmedDistOutlierFilterRatio >= 1) return;
		//printf("\n#iteration %d - correspondence size before trimmedDistOutlierFilter = %d\n", iteration, correspondences.size());
		std::nth_element(correspondences.begin(), correspondences.begin() + correspondences.size()*trimmedDistOutlierFilterRatio, correspondences.end(), correSort);
		npts = floor(correspondences.size()*trimmedDistOutlierFilterRatio);
		//printf("#iteration %d - correspondence size after trimmedDistOutlierFilter = %d\n", iteration, npts);
	}

	void icp::maxDistOutlierFilter(std::vector<Correspondence>& correspondences)
	{
		if (maxDistOutlierFilterDist <= 0) return;
		//printf("\n#iteration %d - correspondence size before DistOutlierFilter = %d\n", iteration, correspondences.size());
		int j = 0;
		for (int i = 0; i < correspondences.size(); ++i)
		{
			if (correspondences[i].distance < maxDistOutlierFilterDist)
			{
				correspondences[j] = correspondences[i];
				j++;
			}
		}
		npts = j;
		//printf("#iteration %d - correspondence size after DistOutlierFilter = %d\n", iteration, npts);
	}
}