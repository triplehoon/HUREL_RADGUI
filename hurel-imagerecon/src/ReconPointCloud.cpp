#include "ReconPointCloud.h"

#include "open3d/geometry/Image.h"

// helper classes for VoxelDownSample and VoxelDownSampleAndTrace
namespace {
	class AccumulatedReconPoint {
	public:
		
		AccumulatedReconPoint()
			: num_of_points_(0),
			point_(0.0, 0.0, 0.0),
			normal_(0.0, 0.0, 0.0),
			color_(0.0, 0.0, 0.0),
			reconValue_(0){}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		void AddPoint(const HUREL::Compton::ReconPointCloud& cloud, int index) {
			point_ += cloud.points_[index];
			if (cloud.HasNormals()) {
				if (!std::isnan(cloud.normals_[index](0)) &&
					!std::isnan(cloud.normals_[index](1)) &&
					!std::isnan(cloud.normals_[index](2))) {
					normal_ += cloud.normals_[index];
				}
			}
			if (cloud.HasColors()) {
				color_ += cloud.colors_[index];
			}
			num_of_points_++;
			reconValue_ += cloud.reconValues_[index];
		}

		Eigen::Vector3d GetAveragePoint() const {
			return point_ / double(num_of_points_);
		}

		Eigen::Vector3d GetAverageNormal() const {
			// Call NormalizeNormals() afterwards if necessary
			return normal_ / double(num_of_points_);
		}

		Eigen::Vector3d GetAverageColor() const {
			return color_ / double(num_of_points_);
		}

		double GetReconValue() const {
			return reconValue_ / double(num_of_points_);
		}

	public:
		int num_of_points_;
		Eigen::Vector3d point_;
		Eigen::Vector3d normal_;
		Eigen::Vector3d color_;
		double reconValue_;
	};
};

HUREL::Compton::ReconPointCloud::ReconPointCloud(open3d::geometry::PointCloud& pc) :
	open3d::geometry::PointCloud(pc)
{
	size_t size = pc.points_.size();
	reconValues_ = std::vector<double>(size);

}

std::tuple<double, double, double> HUREL::Compton::ReconPointCloud::cartesianToSpherical(double x, double y, double z)
{
	double rho = std::sqrt(x*x + y*y + z*z);
    double az = std::atan2(y, x); //rad
   //double az = std::atan2(x, y);
   // double pol = std::acos(z/rho); //rad

    double pol = std::atan2(z, std::sqrt(x*x + y*y));
	
	return std::make_tuple(rho, RAD2DEG(az), RAD2DEG(pol));
}

void HUREL::Compton::ReconPointCloud::imspaceLim(open3d::geometry::PointCloud& totalPC, int azFOV, int polFOV, open3d::geometry::PointCloud* outFOVPC, Eigen::MatrixXd* outFOVchk)
{
	open3d::geometry::PointCloud& reconPCFOVlim = *outFOVPC;
	Eigen::MatrixXd& fovchk = *outFOVchk;
	Eigen::Vector3d azvec(3); Eigen::Vector3d polarvec(3);

	Eigen::Vector3d normvec(3); normvec[0] = 0.; normvec[1] = 0.; normvec[2] = 1.;

	for (size_t size = 0; size < totalPC.points_.size(); size++)
	{
		Eigen::Vector3d& point = totalPC.points_[size];
		//Eigen::Vector3d& normal = totalPC.normals_[size];
		Eigen::Vector3d& color = totalPC.colors_[size];
		//Eigen::Vector3d& reconValue = totalPC.reconValues_[size];

		azvec[0] = point.x(); azvec[1] = 0; azvec[2] = point.z();
		azvec.normalized();
		double az = acos(azvec.dot(normvec)) / EIGEN_PI * 180;

		polarvec[0] = 0; polarvec[1] = point.y(); polarvec[2] = point.z();
		polarvec.normalized();
		double polar = acos(polarvec.dot(normvec)) / EIGEN_PI * 180;
		
		if (az <=azFOV / 2 && polar < polFOV / 2)
		{
			fovchk(0, size) = 1;
			reconPCFOVlim.points_.push_back(point);
			reconPCFOVlim.colors_.push_back(color);
		}
		else
		{
			fovchk(0,size) = 0;
		}
	}
}

void HUREL::Compton::ReconPointCloud::imspaceLim(open3d::geometry::PointCloud& totalPC, int azFOV, int polFOV, Eigen::Matrix4d transMatrix, open3d::geometry::PointCloud* outtransFOVPC, open3d::geometry::PointCloud* outtransPC, Eigen::MatrixXd* outFOVchk)
{
	open3d::geometry::PointCloud& totalPCtrans = *outtransPC;
	totalPCtrans = totalPC;
	open3d::geometry::PointCloud& recontransPCFOVlim = *outtransFOVPC;
	Eigen::MatrixXd& fovchk = *outFOVchk;
	Eigen::Matrix4d t265toLACCPosTranslate; Eigen::Matrix4d t265toLACCPosTransformInv;
	t265toLACCPosTranslate << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;
	t265toLACCPosTransformInv = t265toLACCPosTranslate.inverse();
	totalPCtrans.Transform(t265toLACCPosTransformInv);

	for (size_t size = 0; size < totalPCtrans.points_.size(); size++)
	{
		Eigen::Vector3d& point_trans = totalPCtrans.points_[size];
		//std::cout << "point_trans is: " << point_trans <<std::endl;
		Eigen::Vector3d& color_trans = totalPCtrans.colors_[size];
		auto [rho, az, pol] = cartesianToSpherical(point_trans.z(), point_trans.x(), point_trans.y());
		//std::cout << "az is: " << az << "pol is: " << pol << std::endl;
		if (abs(az) <= azFOV && abs(pol) <= polFOV)
		{
			fovchk(0, size) = 1;
			recontransPCFOVlim.points_.push_back(point_trans);
			recontransPCFOVlim.colors_.push_back(color_trans);
		}
		else
		{
			fovchk(0,size) = 0;
		}
	}
}

void HUREL::Compton::ReconPointCloud::CalculateReconPoint(ListModeData lmData, double(*calcFunc)(ListModeData&, Eigen::Vector3d&))
{

	size_t size = points_.size();
	maxReoconValue = -DBL_MAX;
#pragma omp parallel for
	for (int i = 0; i < size; ++i)
	{
#pragma omp atomic
		reconValues_[i] += calcFunc(lmData, points_[i]);
		if (reconValues_[i] > maxReoconValue)
		{
			maxReoconValue = reconValues_[i];
		}
	}
}

void HUREL::Compton::ReconPointCloud::CalculateReconPointCoded(RadiationImage& lmImage)
{
	size_t size = points_.size();
	maxReoconValue = -DBL_MAX;
#pragma omp parallel for
	for (int i = 0; i < size; ++i)
	{
#pragma omp atomic
		reconValues_[i] += lmImage.OverlayValue(points_[i], eRadiationImagingMode::CODED);
		if (reconValues_[i] > maxReoconValue)
		{
			maxReoconValue = reconValues_[i];
		}
	}
}

void HUREL::Compton::ReconPointCloud::CalculateReconPointCompton(RadiationImage& lmImage)
{
	size_t size = points_.size();
	maxReoconValue = -DBL_MAX;
#pragma omp parallel for
	for (int i = 0; i < size; ++i)
	{
#pragma omp atomic
		reconValues_[i] += lmImage.OverlayValue(points_[i], eRadiationImagingMode::COMPTON);
		if (reconValues_[i] > maxReoconValue)
		{
			maxReoconValue = reconValues_[i];
		}
	}
}

void HUREL::Compton::ReconPointCloud::CalculateReconPointHybrid(RadiationImage& lmImage)
{
	size_t size = points_.size();
	maxReoconValue = -DBL_MAX;
#pragma omp parallel for
	for (int i = 0; i < size; ++i)
	{
#pragma omp atomic
		reconValues_[i] += lmImage.OverlayValue(points_[i], eRadiationImagingMode::HYBRID);
		if (reconValues_[i] > maxReoconValue)
		{
			maxReoconValue = reconValues_[i];
		}
	}
}

double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojection(ListModeData& lmData, Eigen::Vector3d& imgPoint)
{
	double ScatterEnergy = lmData.Scatter.InteractionEnergy;
	double AbsorberEnergy = lmData.Absorber.InteractionEnergy;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}
	double comptonCal = 1 - 511 * lmData.Scatter.InteractionEnergy / lmData.Absorber.InteractionEnergy / (lmData.Scatter.InteractionEnergy + lmData.Absorber.InteractionEnergy);
	if (comptonCal >= 1 || comptonCal <= -1)
	{
		return 0;
	}

   double comptonScatteringAngle = acos(comptonCal) / EIGEN_PI * 180; //radian to degree
   Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.TransformedInteractionPoint.head<3>());
   Eigen::Vector3d scatterToAbsorberVector = (lmData.Scatter.TransformedInteractionPoint.head<3>() - lmData.Absorber.TransformedInteractionPoint.head<3>());
   effectToScatterVector.normalize();
   scatterToAbsorberVector.normalize();
   double positionDotPord = effectToScatterVector.dot(scatterToAbsorberVector);
   double effectedAngle = acos(positionDotPord) / EIGEN_PI * 180;
   double sigmacomptonScatteringAngle = 511 / sin(comptonScatteringAngle) * sqrt(1 / pow(AbsorberEnergy, 2)) - 1 / pow(TotalEnergy, 2) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2);
   double BP_sig_thres = 2;
   if (sigmacomptonScatteringAngle*180/3.1416<1.5)
   {
	  sigmacomptonScatteringAngle=1.5*3.1416/180;
   }
   if (sigmacomptonScatteringAngle <=65)
   {
   	  if (abs(effectedAngle - comptonScatteringAngle) < BP_sig_thres* sigmacomptonScatteringAngle)
   	  {
   	     return 1;
   	  }
	  else
	  {
		return 0;
	  }
   }
   else
   {
	  return 0;
   }
}


double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojectionUntransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint)
{
	double ScatterEnergy = (lmData.Scatter.InteractionEnergy)/1000;
	double AbsorberEnergy = (lmData.Absorber.InteractionEnergy)/1000;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}
	double comptonCal =  1 - 0.511 * ScatterEnergy / AbsorberEnergy / TotalEnergy;
	if (comptonCal >= 1 || comptonCal <= -1)
	{
		return 0;
	}

	double comptonScatteringAngle = acos(comptonCal);
	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.RelativeInteractionPoint.head<3>());
	Eigen::Vector3d scatterToAbsorberVector = (lmData.Scatter.RelativeInteractionPoint.head<3>() - lmData.Absorber.RelativeInteractionPoint.head<3>());
	effectToScatterVector.normalize();
	scatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(scatterToAbsorberVector);
	double effectedAngle = acos(positionDotPord);
	double sigmacomptonScatteringAngle = 0.511 / sin(comptonScatteringAngle) * sqrt(1 / pow(AbsorberEnergy, 2)) - 1 / pow(TotalEnergy, 2) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2);
	
	double BP_sig_thres = 2;

    if (comptonScatteringAngle <= 65 * 180 / 3.14)
    {
    	  if (abs(effectedAngle - comptonScatteringAngle) < BP_sig_thres* sigmacomptonScatteringAngle)
    	  {
    	     return 1;
    	  }
	   else
	   {
	 	return 0;
	   }
    }
    else
    {
	   return 0;
    }
}

/*
double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojectionUntransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint, double FOVchk)
{
	double BP_sig_thres = 2.5;
	double outComptonScatterAngle;
	Eigen::Vector3d outScatterToAbsorberVector;
	double outSigmacomptonScatteringAngle;
	if (FOVchk != 0)
	{
		double ScatterEnergy = lmData.Scatter.InteractionEnergy;
		double AbsorberEnergy = lmData.Absorber.InteractionEnergy;
		double TotalEnergy = ScatterEnergy + AbsorberEnergy;
		if (lmData.Type != eInterationType::COMPTON)
		{
			return 0;
		}
		double comptonCal = 1 - 511 * lmData.Scatter.InteractionEnergy / lmData.Absorber.InteractionEnergy / (lmData.Scatter.InteractionEnergy + lmData.Absorber.InteractionEnergy);
		if (comptonCal >= 1 || comptonCal <= -1)
		{
			return 0;
		}

		outComptonScatterAngle = acos(comptonCal) / EIGEN_PI * 180;
		Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.RelativeInteractionPoint.head<3>());
		outScatterToAbsorberVector = (lmData.Scatter.RelativeInteractionPoint.head<3>() - lmData.Absorber.RelativeInteractionPoint.head<3>());
		effectToScatterVector.normalize();
		outScatterToAbsorberVector.normalize();
		double positionDotPord = effectToScatterVector.dot(outScatterToAbsorberVector);
		double effectedAngle = acos(positionDotPord) / EIGEN_PI * 180;
		outSigmacomptonScatteringAngle = 511 / sin(outComptonScatterAngle) * sqrt(1 / pow(AbsorberEnergy, 2)) - 1 / pow(TotalEnergy, 2) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2);

		if (abs(effectedAngle - outComptonScatterAngle) < BP_sig_thres * outSigmacomptonScatteringAngle)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
	
}
*/
double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojection(ListModeData& lmData, Eigen::Vector3d& imgPoint, double FOVchk)
{
	double BP_sig_thres = 2;
	double ComptonScatterAngle;
	Eigen::Vector3d ScatterToAbsorberVector;
	double SigmacomptonScatteringAngle;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}

	if (FOVchk != 0)
	{
		double ScatterEnergy = (lmData.Scatter.InteractionEnergy)/1000;
		double AbsorberEnergy = (lmData.Absorber.InteractionEnergy)/1000;
		double TotalEnergy = ScatterEnergy + AbsorberEnergy;

		double value = 1 - 0.511 * ScatterEnergy / AbsorberEnergy / TotalEnergy; 
		if (value >= 1 || value <= -1)
		{
			return 0;
		}
		double ComptonScatterAngle = acos(value); //rad
		Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.RelativeInteractionPoint.head<3>());
		ScatterToAbsorberVector = (lmData.Scatter.RelativeInteractionPoint.head<3>() - lmData.Absorber.RelativeInteractionPoint.head<3>());
		effectToScatterVector.normalize();
		ScatterToAbsorberVector.normalize();
		double positionDotPord = effectToScatterVector.dot(ScatterToAbsorberVector);
		double effectedAngle = acos(positionDotPord); //rad 
 		SigmacomptonScatteringAngle = 0.511 / sin(ComptonScatterAngle) * sqrt((1 / pow(AbsorberEnergy, 2) - 1 / pow(TotalEnergy, 2)) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2));
		if ((effectedAngle / EIGEN_PI * 180) > 65)
		{
			return 0;
		}
		else
		{
			if (abs(effectedAngle - ComptonScatterAngle) < BP_sig_thres * SigmacomptonScatteringAngle)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
	}
	else
	{
		return 0;
	}
	
}

double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojectionTransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint, double FOVchk)
{
	double BP_sig_thres = 2;
	double ComptonScatterAngle;
	Eigen::Vector3d ScatterToAbsorberVector;
	double SigmacomptonScatteringAngle;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}

	if (FOVchk != 0)
	{
		double ScatterEnergy = (lmData.Scatter.InteractionEnergy)/1000;
		double AbsorberEnergy = (lmData.Absorber.InteractionEnergy)/1000;
		double TotalEnergy = ScatterEnergy + AbsorberEnergy;

		double value = 1 - 0.511 * ScatterEnergy / AbsorberEnergy / TotalEnergy; 
		if (value >= 1 || value <= -1)
		{
			return 0;
		}
		double ComptonScatterAngle = acos(value); //rad
		Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.TransformedInteractionPoint.head<3>());
		ScatterToAbsorberVector = (lmData.Scatter.TransformedInteractionPoint.head<3>() - lmData.Absorber.TransformedInteractionPoint.head<3>());
		effectToScatterVector.normalize();
		ScatterToAbsorberVector.normalize();
		double positionDotPord = effectToScatterVector.dot(ScatterToAbsorberVector);
		double effectedAngle = acos(positionDotPord); //rad 
 		SigmacomptonScatteringAngle = 0.511 / sin(ComptonScatterAngle) * sqrt((1 / pow(AbsorberEnergy, 2) - 1 / pow(TotalEnergy, 2)) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2));
		if ((effectedAngle / EIGEN_PI * 180) > 65)
		{
			return 0;
		}
		else
		{
			if (abs(effectedAngle - ComptonScatterAngle) < BP_sig_thres * SigmacomptonScatteringAngle)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
	}
	else
	{
		return 0;
	}
	
}

HUREL::Compton::RGBA_t HUREL::Compton::ReconPointCloud::ColorScaleJet(double v, double vmin, double vmax)
{
	double dv;

	if (v < vmin)
	{
		v = vmin;
		RGBA_t rgba = { 0, 0, 0, 0.0001 };
		return rgba;
	}
	if (v > vmax)
	{
		v = vmax;
	}
	dv = vmax - vmin;
	if (dv == 0)
	{
		RGBA_t rgba = { 0, 0, 0, 0.0001 };
		return rgba;
	}
	double r = 1.0f, g = 1.0f, b = 1.0f;
	if (v < (vmin + 0.25 * dv))
	{
		r = 0;
		g = 4 * (v - vmin) / dv;
	}
	else if (v < (vmin + 0.5 * dv))
	{
		r = 0;
		b = 1 + 4 * (vmin + 0.25f * dv - v) / dv;
	}
	else if (v < (vmin + 0.75 * dv))
	{
		r = 4 * (v - vmin - 0.5f * dv) / dv;
		b = 0;
	}
	else
	{
		g = 1 + 4 * (vmin + 0.75f * dv - v) / dv;
		b = 0;
	}
	double a = 0.5;
	RGBA_t rgba_out = { r, g, b, a };
	return rgba_out;
}

std::shared_ptr<HUREL::Compton::ReconPointCloud> HUREL::Compton::ReconPointCloud::VoxelDownSample(double voxel_size) const
{
	auto output = std::make_shared<HUREL::Compton::ReconPointCloud>();
	if (voxel_size <= 0.0) {
		printf("[VoxelDownSample] voxel_size <= 0.");
	}
	Eigen::Vector3d voxel_size3 =
		Eigen::Vector3d(voxel_size, voxel_size, voxel_size);
	Eigen::Vector3d voxel_min_bound = GetMinBound() - voxel_size3 * 0.5;
	Eigen::Vector3d voxel_max_bound = GetMaxBound() + voxel_size3 * 0.5;
	if (voxel_size * std::numeric_limits<int>::max() <
		(voxel_max_bound - voxel_min_bound).maxCoeff()) {
		printf("[VoxelDownSample] voxel_size is too small.");
	}
	std::unordered_map<Eigen::Vector3i, AccumulatedReconPoint,
		open3d::utility::hash_eigen<Eigen::Vector3i>>
		voxelindex_to_accpoint;

	Eigen::Vector3d ref_coord;
	Eigen::Vector3i voxel_index;
	for (int i = 0; i < (int)points_.size(); i++) {
		ref_coord = (points_[i] - voxel_min_bound) / voxel_size;
		voxel_index << int(floor(ref_coord(0))), int(floor(ref_coord(1))),
			int(floor(ref_coord(2)));
		voxelindex_to_accpoint[voxel_index].AddPoint(*this, i);
	}
	bool has_normals = HasNormals();
	bool has_colors = HasColors();
	for (const auto& accpoint : voxelindex_to_accpoint) {
		output->points_.push_back(accpoint.second.GetAveragePoint());
		if (has_normals) {
			output->normals_.push_back(accpoint.second.GetAverageNormal());
		}
		if (has_colors) {
			output->colors_.push_back(accpoint.second.GetAverageColor());
		}
		output->reconValues_.push_back(accpoint.second.GetReconValue());
	}
	printf(
		"Pointcloud down sampled from {%d} points to {%d} points.",
		(int)points_.size(), (int)output->points_.size());
	return output;
}
