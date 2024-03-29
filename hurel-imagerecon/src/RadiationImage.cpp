#include "RadiationImage.h"

using namespace Eigen;

// Makting Detector Response Image
constexpr double Det_W = 0.312;
constexpr double Mask_W = 0.370;
constexpr double Mpix = 37;
constexpr double S2M = 2;
constexpr double M2D = 0.07;
constexpr double SP = S2M - M2D;// Source to Mask distance(mm)
constexpr double M = 1 + M2D / S2M; // projection ratio((a + b) / a)
constexpr double Dproj = Det_W / (Mask_W / Mpix * M); // projection mask to Detector pixel Length(mm)
constexpr double ReconPlaneWidth = S2M / M2D * Det_W;
constexpr double ResImprov = 5;
int PixelCount = static_cast<int>(round(Dproj * ResImprov));

inline int findIndex(double value, double min, double pixelSize)
{
	if (value - min <= 0)
	{
		return -1;
	}
	return static_cast<int>(floor((value - min) / pixelSize));
}

static cv::Mat CodedMaskMat()
{
	static bool isMaskMade = false;
	static cv::Mat mask;
	if (isMaskMade)
	{
		return mask;
	}
	else
	{
		mask = cv::Mat(37, 37, CV_32S);
		for (int i = 0; i < 37; ++i)
		{
			for (int j = 0; j < 37; ++j)
			{
				if (HUREL::Compton::mCodeMask[j][i])
				{
					mask.at<int>(j, 36 - i) = -1;
				}
				else
				{
					mask.at<int>(j, 36 -i) = 1;
				}
			}
		}
		isMaskMade = true;
		return mask;
	}
}

void HUREL::Compton::RadiationImage::ShowCV_32SAsJet(cv::Mat img, int size)
{
	if (img.type() != CV_32S)
	{
		return;
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, &minValue, &maxValue);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue)
				/ (maxValue - minValue) * 255);
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::Mat showImg;

	int sizeHeight = size;
	int sizeWidth = size;

	if (colorImg.size().height > colorImg.size().width)
	{
		sizeWidth = size * colorImg.size().width / colorImg.size().height;
	}
	else
	{
		sizeHeight = size * colorImg.size().height / colorImg.size().width;
	}

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST);
	cv::imshow("img", showImg);
	cv::waitKey(0);
}

cv::Mat HUREL::Compton::RadiationImage::GetCV_32SAsJet(cv::Mat img, int size)
{
	cv::Mat showImg;
	if (img.type() != CV_32S)
	{
		return showImg;
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, &minValue, &maxValue);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::cvtColor(colorImg, colorImg, COLOR_BGR2BGRA);

	int sizeHeight = size;
	int sizeWidth = size;

	if (colorImg.size().height > colorImg.size().width)
	{
		sizeWidth = size * colorImg.size().width / colorImg.size().height;
	}
	else
	{
		sizeHeight = size * colorImg.size().height / colorImg.size().width;
	}

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST);

	return showImg;
}

cv::Mat HUREL::Compton::RadiationImage::GetCV_32SAsJet(cv::Mat img, int sizeh, int sizew, double minValuePortion, double opacity)
{
	cv::Mat showImg;
	if (img.type() != CV_32S)
	{
		return showImg;
	}
	if (img.rows == 0)
	{
		return cv::Mat();
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;	
	cv::minMaxIdx(img, nullptr, &maxValue);
	minValue = maxValue * minValuePortion;
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{			
			if (img.at<int>(i, j) < maxValue* minValuePortion )
			{
				normImg.at<uchar>(i, j) = 0;
			}
			else
			{
				normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
			}			
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::cvtColor(colorImg, colorImg, COLOR_BGR2BGRA);
	for (int i = 0; i < colorImg.rows; i++)
	{
		for (int j = 0; j < colorImg.cols; j++)
		{
			auto& pixel = colorImg.at<cv::Vec4b>(i, j);
			if (pixel[0] == 128 && pixel[1] == 0 && pixel[2] == 0)
			{
				pixel[0] = 0;
				pixel[3] = 0;
			}
			else
			{
				pixel[3] = 255 * opacity;
			}
		}
	}
	int sizeHeight = sizeh;
	int sizeWidth = sizew;

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST);

	return showImg;
}

HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data)
{
	Mat responseImg(PixelCount, PixelCount, CV_32S, Scalar(0));
	Mat comptonImg(PixelCount, PixelCount, CV_32S, Scalar(1));
	int32_t* responseImgPtr = static_cast<int32_t*>(static_cast<void*>(responseImg.data));
	int32_t* comptonImgPtr = static_cast<int32_t*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;

	#pragma omp parallel for
	for (int i = 0; i < data.size(); ++i)
	{
		ListModeData& lm = data[i];
		//if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 600 || lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy > 720)
		//{
		//	continue;
		//}
		/*if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 1000 || lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy > 1500)
		{
			continue;
		}*/

		if (lm.Type == eInterationType::CODED)
		{
			//continue;
			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];

			int iX = findIndex(interactionPoseX, -Det_W / 2, Det_W / PixelCount);
			int iY = findIndex(interactionPoseY, -Det_W / 2, Det_W / PixelCount);
			if (iX >= 0 && iY >= 0 && iX < PixelCount && iY < PixelCount)
			{
				++responseImgPtr[PixelCount * iY + iX];
				++codedImageCount;
			}
		}	
				
		if (lm.Type == eInterationType::COMPTON)
		{	
			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 200)
			{
				continue;
			}
			++comptonImageCount;
			for (int i = 0; i < PixelCount; ++i)
			{
				for (int j = 0; j < PixelCount; ++j)
				{
					double imagePlaneX = ReconPlaneWidth / PixelCount * i + ReconPlaneWidth / PixelCount * 0.5 - ReconPlaneWidth / 2;
					double imagePlaneY = ReconPlaneWidth / PixelCount * j + ReconPlaneWidth / PixelCount * 0.5 - ReconPlaneWidth / 2;
					double imagePlaneZ = S2M + M2D + 0.02;
					Eigen::Vector3d imgPoint;
					imgPoint[0] = imagePlaneX;
					imgPoint[1] = imagePlaneY;
					imgPoint[2] = imagePlaneZ;
					comptonImgPtr[PixelCount * (PixelCount - j - 1) + PixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojectionUntransformed(lm, imgPoint);
				}
			}

		}
	}
	std::cout << "Lm Count: " << data.size() << " Coded count: " << codedImageCount << " Compton count: " << comptonImageCount << std::endl;
	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(37 * ResImprov, 37 * ResImprov), 0, 0, INTER_NEAREST);
	Mat reconImg;
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG);

	double maxValue;
	cv::minMaxLoc(reconImg, nullptr, &maxValue);
	Mat idxImg(PixelCount, PixelCount, CV_32S, Scalar(1));

	cv::max(reconImg, idxImg, mCodedImage);
	
	//mCodedImage = reconImg;

	mDetectorResponseImage = responseImg;
	mComptonImage = comptonImg;

	mHybridImage = mCodedImage.mul(mComptonImage);


	if (data.size() == 0)
	{
		return;
	}


	mDetectorTransformation = data[0].DetectorTransformation;
	mListedListModeData = data;

	//ShowCV_32SAsJet(mDetectorResponseImage, 1000);
	//ShowCV_32SAsJet(mCodedImage, 1000);
	//ShowCV_32SAsJet(mComptonImage, 1000);
	//ShowCV_32SAsJet(mHybridImage, 1000);
}

HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov)
{
	double m = 1 + m2D / s2M;
	double reconPlaneWidth = s2M / m2D * det_W;
	double dproj = det_W / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	int pixelCount = static_cast<int>(round(dproj * resImprov));



	Mat responseImg(pixelCount, pixelCount, CV_32S, Scalar(0));
	Mat comptonImg(pixelCount, pixelCount, CV_32S, Scalar(1));
	int32_t* responseImgPtr = static_cast<int32_t*>(static_cast<void*>(responseImg.data));
	int32_t* comptonImgPtr = static_cast<int32_t*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;

#pragma omp parallel for
	for (int i = 0; i < data.size(); ++i)
	{
		ListModeData &lm = data[i];
		if (lm.Type == eInterationType::CODED)
		{
			//if (lm.Scatter.InteractionEnergy < 600 || lm.Scatter.InteractionEnergy > 720)
			//{
			//	continue;
			//}
			double &interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
			double &interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];

			int iX = findIndex(interactionPoseX, -det_W / 2, det_W / pixelCount);
			int iY = findIndex(interactionPoseY, -det_W / 2, det_W / pixelCount);
			if (iX >= 0 && iY >= 0 && iX < pixelCount && iY < pixelCount)
			{
				++responseImgPtr[pixelCount * iY + iX];
				++codedImageCount;
			}
		}

		if (lm.Type == eInterationType::COMPTON)
		{
			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 400)
			{
				continue;
			}
			if (lm.Scatter.InteractionEnergy < 10)
			{
				continue;
			}
			++comptonImageCount;
			double comptonScatterAngle = nan("");
			double sigmacomptonScatteringAngle = nan("");
			Eigen::Vector3d sToAVector;
			double imagePlaneZ = s2M + 0.07;

			for (int i = 0; i < pixelCount; ++i)
			{
				for (int j = 0; j < pixelCount; ++j)
				{
					double imagePlaneX = reconPlaneWidth / pixelCount * i + reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;
					double imagePlaneY = reconPlaneWidth / pixelCount * j + reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;
					Eigen::Vector3d imgPoint;
					imgPoint[0] = imagePlaneX;
					imgPoint[1] = imagePlaneY;
					imgPoint[2] = imagePlaneZ;
					comptonImgPtr[pixelCount * (pixelCount - j - 1) + pixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojectionUntransformed(lm, imgPoint, &comptonScatterAngle, &sigmacomptonScatteringAngle, &sToAVector);
				}
			}
		}
	}
	std::cout << "Lm Count: " << data.size() << " Coded count: " << codedImageCount << " Compton count: " << comptonImageCount << std::endl;
	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(37 * resImprov, 37 * resImprov), 0, 0, INTER_NEAREST);
	Mat reconImg;

	Mat drawImg;
	//convert type to CV_32S
	responseImg.convertTo(drawImg, CV_32S);
	ShowCV_32SAsJet(drawImg, 1000);

	cv::filter2D(responseImg, reconImg, CV_32S, scaleG);


	

	//reconImg = -reconImg;
	double maxValue;
	//cv::minMaxLoc(reconImg, nullptr, &maxValue);
	//Mat idxImg(pixelCount, pixelCount, CV_32S, Scalar(maxValue * 0.01));
	//mCodedImage = reconImg;
	//cv::max(reconImg, idxImg, mCodedImage);


	double fovHeight = 2 * tan((hFov / 2) * M_PI / 180.0) * (s2M + m2D + 0.02);
	double fovWidth = 2 * tan((wFov / 2) * M_PI / 180.0) * (s2M + m2D + 0.02);

	//height correction
	constexpr double heightDiff = 0.28;

	double heightPixelSize = reconPlaneWidth / pixelCount;
	int offSetPixelCount = heightDiff / heightPixelSize;

	int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
	int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);

	int minHeightPixleCount = (pixelCount - heightPixelCount) / 2 + offSetPixelCount;
	int maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 + offSetPixelCount;

	if (minHeightPixleCount < 0)
	{
		minHeightPixleCount = 0;
	}
	if (maxHeightPixleCount > pixelCount)
	{
		maxHeightPixleCount = pixelCount;
	}
	if (widthPixelCount > pixelCount)
	{
		widthPixelCount = pixelCount;
	}

	mDetectorResponseImage = responseImg;
	cv::Mat nonFiltered = reconImg;
	cv::Mat Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	nonFiltered = comptonImg;
	Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 3);
	Filtered.convertTo(Filtered, CV_32S);
	comptonImg = Filtered;

	mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	nonFiltered = mCodedImage.mul(mComptonImage);
	Filtered;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 20);
	Filtered.convertTo(Filtered, CV_32S);


	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;

	minMaxLoc(Filtered, &minVal, &maxVal, &minLoc, &maxLoc);
	if (maxVal > 400)
	{
	}

	mHybridImage = Filtered;

	//spdlog::info("RadImage: Hybrid Max: " + std::to_string(maxVal));
	if (data.size() == 0)
	{
		return;
	}

	mDetectorTransformation = data[0].DetectorTransformation;
	mListedListModeData = data;
}

HUREL::Compton::RadiationImage::RadiationImage(sInteractionData &data, double s2M, double resImprov, double m2D, double hFov, double wFov)
{
	double m = 1 + m2D / s2M;
	double reconPlaneWidth = s2M / m2D * 0.3;
	double dproj = 0.3/ (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	int pixelCount = static_cast<int>(round(dproj * resImprov));



	Mat comptonImg(pixelCount, pixelCount, CV_32S, Scalar(1));
	int32_t* comptonImgPtr = static_cast<int32_t*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;

	#pragma omp parallel for
	for (int i = 0; i < data.ComptonListModeData->size(); ++i)
	{
		ListModeData &lm = data.ComptonListModeData->at(i);

		if (lm.Type == eInterationType::COMPTON)
		{
			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 400)
			{
				continue;
			}
			if (lm.Scatter.InteractionEnergy < 10)
			{
				continue;
			}
			++comptonImageCount;
			double comptonScatterAngle = nan("");
			double sigmacomptonScatteringAngle = nan("");
			Eigen::Vector3d sToAVector;
			double imagePlaneZ = s2M + 0.07;

			for (int i = 0; i < pixelCount; ++i)
			{
				for (int j = 0; j < pixelCount; ++j)
				{
					double imagePlaneX = reconPlaneWidth / pixelCount * i + reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;
					double imagePlaneY = reconPlaneWidth / pixelCount * j + reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;
					Eigen::Vector3d imgPoint;
					imgPoint[0] = imagePlaneX;
					imgPoint[1] = imagePlaneY;
					imgPoint[2] = imagePlaneZ;
					comptonImgPtr[pixelCount * (pixelCount - j - 1) + pixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojectionUntransformed(lm, imgPoint, &comptonScatterAngle, &sigmacomptonScatteringAngle, &sToAVector);
				}
			}
		}
	}
	
	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(37 * resImprov, 37 * resImprov), 0, 0, INTER_NEAREST);
	Mat reconImg;
	Mat responseImg;

	cv::eigen2cv(data.RelativeInteractionPoint, responseImg);
	cv::flip(responseImg, responseImg, 1);
	cv::rotate(responseImg, responseImg, ROTATE_90_COUNTERCLOCKWISE);
	cv::resize(responseImg, responseImg, Size(pixelCount, pixelCount), 0, 0, INTER_LINEAR);

	Mat drawImg;
	//convert type to CV_32S
	//responseImg.convertTo(drawImg, CV_32S);
	//ShowCV_32SAsJet(drawImg, 1000);

	cv::filter2D(responseImg, reconImg, CV_32S, scaleG);


	//reconImg = -reconImg;
	double maxValue;
	//cv::minMaxLoc(reconImg, nullptr, &maxValue);
	//Mat idxImg(pixelCount, pixelCount, CV_32S, Scalar(maxValue * 0.01));
	//mCodedImage = reconImg;
	//cv::max(reconImg, idxImg, mCodedImage);


	double fovHeight = 2 * tan((hFov / 2) * M_PI / 180.0) * (s2M + m2D + 0.02);
	double fovWidth = 2 * tan((wFov / 2) * M_PI / 180.0) * (s2M + m2D + 0.02);

	//height correction
	constexpr double heightDiff = 0.28;

	double heightPixelSize = reconPlaneWidth / pixelCount;
	int offSetPixelCount = heightDiff / heightPixelSize;

	int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
	int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);

	int minHeightPixleCount = (pixelCount - heightPixelCount) / 2 + offSetPixelCount;
	int maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 + offSetPixelCount;

	if (minHeightPixleCount < 0)
	{
		minHeightPixleCount = 0;
	}
	if (maxHeightPixleCount > pixelCount)
	{
		maxHeightPixleCount = pixelCount;
	}
	if (widthPixelCount > pixelCount)
	{
		widthPixelCount = pixelCount;
	}

	mDetectorResponseImage = responseImg;
	cv::Mat nonFiltered = reconImg;
	cv::Mat Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	nonFiltered = comptonImg;
	Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 3);
	Filtered.convertTo(Filtered, CV_32S);
	comptonImg = Filtered;

	mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	nonFiltered = mCodedImage.mul(mComptonImage);
	Filtered;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 20);
	Filtered.convertTo(Filtered, CV_32S);


	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;

	minMaxLoc(Filtered, &minVal, &maxVal, &minLoc, &maxLoc);
	if (maxVal > 400)
	{
	}

	mHybridImage = Filtered;

	//spdlog::info("RadImage: Hybrid Max: " + std::to_string(maxVal));

	mDetectorTransformation = data.DetectorTransformation;
	//mListedListModeData = lmData;
}

HUREL::Compton::RadiationImage::RadiationImage(std::vector<sInteractionData> &dataVector, double s2M, double resImprov, double m2D, double hFov, double wFov)
{
	double m = 1 + m2D / s2M;
	double reconPlaneWidth = s2M / m2D * 0.3;
	double dproj = 0.3 / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	int pixelCount = static_cast<int>(round(dproj * resImprov));
	if (dataVector.size() == 0)
	{
		return;
	}


	Mat comptonImg(pixelCount, pixelCount, CV_32S, Scalar(1));
	int32_t* comptonImgPtr = static_cast<int32_t*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;
	Eigen::MatrixXd interationDataSum = Eigen::MatrixXd::Zero(INTERACTION_GRID_SIZE, INTERACTION_GRID_SIZE);
	for (int index = 0; index < dataVector.size(); ++index)
	{
		sInteractionData &data = dataVector.at(index);
		#pragma omp parallel for
		for (int i = 0; i < data.ComptonListModeData->size(); ++i)
		{
			ListModeData &lm = data.ComptonListModeData->at(i);

			if (lm.Type == eInterationType::COMPTON)
			{
				if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 400)
				{
					continue;
				}
				if (lm.Scatter.InteractionEnergy < 50)
				{
					continue;
				}
				++comptonImageCount;
				double comptonScatterAngle = nan("");
				double sigmacomptonScatteringAngle = nan("");
				Eigen::Vector3d sToAVector;
				double imagePlaneZ = s2M + 0.07;

				for (int i = 0; i < pixelCount; ++i)
				{
					for (int j = 0; j < pixelCount; ++j)
					{
						double imagePlaneX = reconPlaneWidth / pixelCount * i + reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;
						double imagePlaneY = reconPlaneWidth / pixelCount * j + reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;
						Eigen::Vector3d imgPoint;
						imgPoint[0] = imagePlaneX;
						imgPoint[1] = imagePlaneY;
						imgPoint[2] = imagePlaneZ;
						comptonImgPtr[pixelCount * (pixelCount - j - 1) + pixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojectionUntransformed(lm, imgPoint, &comptonScatterAngle, &sigmacomptonScatteringAngle, &sToAVector);
					}
				}
			}
		}
	
		interationDataSum += data.RelativeInteractionPoint;
	}

	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(37 * resImprov, 37 * resImprov), 0, 0, INTER_NEAREST);
	Mat reconImg;
	Mat responseImg;
	

	cv::eigen2cv(interationDataSum, responseImg);

	Mat drawImg;
	//convert type to CV_32S
	bool isDraw = false;
	if (isDraw)
	{
		responseImg.convertTo(drawImg, CV_32S);
		ShowCV_32SAsJet(drawImg, 1000);
	}

	cv::flip(responseImg, responseImg, 1);
	cv::rotate(responseImg, responseImg, ROTATE_90_COUNTERCLOCKWISE);
	cv::resize(responseImg, responseImg, Size(pixelCount, pixelCount), 0, 0, INTER_LINEAR);

	cv::filter2D(responseImg, reconImg, CV_32S, scaleG);


	//reconImg = -reconImg;
	double maxValue;
	//cv::minMaxLoc(reconImg, nullptr, &maxValue);
	//Mat idxImg(pixelCount, pixelCount, CV_32S, Scalar(maxValue * 0.01));
	//mCodedImage = reconImg;
	//cv::max(reconImg, idxImg, mCodedImage);


	double fovHeight = 2 * tan((hFov / 2) * M_PI / 180.0) * (s2M + m2D + 0.02);
	double fovWidth = 2 * tan((wFov / 2) * M_PI / 180.0) * (s2M + m2D + 0.02);

	//height correction
	constexpr double heightDiff = 0.28;

	double heightPixelSize = reconPlaneWidth / pixelCount;
	int offSetPixelCount = heightDiff / heightPixelSize;

	int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
	int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);

	int minHeightPixleCount = (pixelCount - heightPixelCount) / 2 + offSetPixelCount;
	int maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 + offSetPixelCount;

	if (minHeightPixleCount < 0)
	{
		minHeightPixleCount = 0;
	}
	if (maxHeightPixleCount > pixelCount)
	{
		maxHeightPixleCount = pixelCount;
	}
	if (widthPixelCount > pixelCount)
	{
		widthPixelCount = pixelCount;
	}

	mDetectorResponseImage = responseImg;
	cv::Mat nonFiltered = reconImg;
	cv::Mat Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	nonFiltered = comptonImg;
	Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 3);
	Filtered.convertTo(Filtered, CV_32S);
	comptonImg = Filtered;

	mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	nonFiltered = mCodedImage.mul(mComptonImage);
	Filtered;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 20);
	Filtered.convertTo(Filtered, CV_32S);


	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;

	minMaxLoc(Filtered, &minVal, &maxVal, &minLoc, &maxLoc);
	if (maxVal > 400)
	{
	}

	mHybridImage = Filtered;

	///spdlog::info("RadImage: Hybrid Max: " + std::to_string(maxVal));

	mDetectorTransformation = dataVector[0].DetectorTransformation;
	//mListedListModeData = lmData;
}

double HUREL::Compton::RadiationImage::OverlayValue(Eigen::Vector3d point, eRadiationImagingMode mode)
{

	constexpr double imagePlaneZ = S2M + M2D + 0.011;
	Eigen::Vector3d detectorNormalVector(0, 0, 1);
	Eigen::Vector4d point4d(point.x(), point.y(), point.z(), 1);
	Eigen::Vector4d transformedPoint = ((mDetectorTransformation).inverse()* point4d);
	if (transformedPoint.z() <= imagePlaneZ || transformedPoint.z() >= 5)
	{
		//std::cout << transformedPoint << std::endl;
		return 0;
	}
	double xPoseOnImgPlane = transformedPoint.x() * imagePlaneZ / transformedPoint.z();
	double yPoseOnImgPlane = transformedPoint.y() * imagePlaneZ / transformedPoint.z();


	int iY = findIndex(xPoseOnImgPlane, -ReconPlaneWidth / 2, ReconPlaneWidth / PixelCount);
	int iX = findIndex(yPoseOnImgPlane, -ReconPlaneWidth / 2, ReconPlaneWidth / PixelCount);
	int tempiY = iY;
	iY = iX;
	iX = tempiY;

	if (iX >= 0 && iY >= 0 && iX < PixelCount && iY < PixelCount)
	{
		int32_t value = 0;
		switch (mode)
		{
		case HUREL::Compton::eRadiationImagingMode::CODED:
			value = static_cast<int32_t*>(static_cast<void*>(mCodedImage.ptr()))[PixelCount * (PixelCount - iY) + PixelCount - iX];
			break;
		case HUREL::Compton::eRadiationImagingMode::COMPTON:
			value = static_cast<int32_t*>(static_cast<void*>(mComptonImage.ptr()))[PixelCount * (PixelCount - iY) + PixelCount - iX];
		case HUREL::Compton::eRadiationImagingMode::HYBRID:
			value = static_cast<int32_t*>(static_cast<void*>(mHybridImage.ptr()))[PixelCount * (PixelCount - iY) + PixelCount - iX];
			break;
		default:
			assert(false);
			return 0.0;
			break;
		}
		return static_cast<double>(value);
	}
	else
	{
		return 0.0;
	}
	



	
}

void HUREL::Compton::RadiationImage::OverlayRadimgToP3(cv::Mat& p3, const cv::Mat& radImg)
{


}

