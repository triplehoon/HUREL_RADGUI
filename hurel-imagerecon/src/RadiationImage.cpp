#include "RadiationImage.h"

using namespace Eigen;

// Makting Detector Response Image
constexpr double Det_W = 0.312;
constexpr double Mask_W = 0.370;
constexpr double Mpix = 37;
constexpr double S2M = 2;
constexpr double M2D = 0.073;
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

inline int converPositionIndex(int originalPosition, int originalSpaceCount, int newSpaceCount)
{
	return static_cast<int>(round(static_cast<double>(originalPosition) / originalSpaceCount * newSpaceCount));
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
				if (HUREL::Compton::mCodeMask[i][j])
				{
					//mask.at<int>(j, 36 - i) = -1;
					mask.at<int>(i, j) = 1;
				}
				else
				{
					//mask.at<int>(j, 36 - i) = 1;
					mask.at<int>(i, j) = -1;
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

HUREL::Compton::RadiationImage::RadiationImage(std::vector<sInteractionData> &dataVector, double s2M, double resImprov, double m2D, double hFov, double wFov)
{
	//정지된 상태에서 영상화 가능
	if (dataVector.size() == 0)
	{
		return;
	}
	
	double m = 1 + m2D / s2M;
	double reconPlaneWidth = s2M / m2D * Det_W;
	double dproj = Det_W / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	int pixelCount = static_cast<int>(round(dproj * resImprov));
	int pixelCountcoded = static_cast<int>(round(dproj * 1));
	double det_w_div2 = -Det_W / 2;
	double pixelSize = Det_W / pixelCountcoded;
	
	double preCalc1 = reconPlaneWidth / pixelCount;	//영상화 영역 한 픽셀 길이
	double preCalc2 = reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;	//영상화 영역의 반 픽셀 길이 - 중심을 (0,0)으로 만들기 위해

	double imagePlaneZ = s2M + M2D;


	int pixelCountcompton = pixelCount;
	Mat comptonImg(pixelCountcompton, pixelCountcompton, CV_32S, Scalar(1));
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
				double imagePlaneZ = s2M + M2D;

				for (int i = 0; i < pixelCountcompton; ++i)
				{
					double imagePlaneX = preCalc1 * i + preCalc2;
					for (int j = 0; j < pixelCountcompton; ++j)
					{
						double imagePlaneY = preCalc1 * j + preCalc2;
						Eigen::Vector3d imgPoint;
						imgPoint[0] = imagePlaneX ;
						imgPoint[1] = imagePlaneY ;
						imgPoint[2] = imagePlaneZ ;
						comptonImgPtr[pixelCountcompton * (pixelCountcompton - j - 1) + pixelCountcompton - i - 1] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
					}
				}
			}
		}
		
		interationDataSum += data.RelativeInteractionPoint;
	}

	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(Mpix, Mpix), 0, 0, INTER_NEAREST);			

	Mat responseImg;
	cv::eigen2cv(interationDataSum, responseImg);
	cv::resize(responseImg, responseImg, Size(pixelCountcoded, pixelCountcoded), 0, 0, INTER_LINEAR);
	//cv::rotate(responseImg, responseImg, ROTATE_90_CLOCKWISE);
	
	Mat reconImg; Mat codedreconImg;
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG, cv::Point(-1,-1), 0.0, cv::BORDER_CONSTANT);
	
	reconImg.convertTo(reconImg, CV_32F);
	if (resImprov > 1.0)
		cv::resize(reconImg, reconImg, Size(pixelCount, pixelCount), 0, 0, INTER_NEAREST);	//compton size로 변경 : 하이브리드 영상용

	double maxValue;

	double fovHeight = 2 * tan((hFov / 2) * M_PI / 180.0) * (s2M + m2D);
	double fovWidth = 2 * tan((wFov / 2) * M_PI / 180.0) * (s2M + m2D);

	//height correction
	//constexpr double heightDiff = 0.35;
	constexpr double heightDiff = 0.035;

	double heightPixelSize = reconPlaneWidth / pixelCount;
	int offSetPixelCount = heightDiff / heightPixelSize;

	int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
	int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);

	int minHeightPixleCount = (pixelCount - heightPixelCount) / 2 - offSetPixelCount;
	int maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 - offSetPixelCount;

	int minWidthPixleCount = (pixelCount - widthPixelCount) / 2 - offSetPixelCount;
	int maxWidthPixleCount = (pixelCount + widthPixelCount) / 2 - offSetPixelCount;

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

	if (minWidthPixleCount < 0)
	{
		minWidthPixleCount = 0;
	}
	if (maxWidthPixleCount > pixelCount)
	{
		maxWidthPixleCount = pixelCount;
	}

	for (int i = 0; i < reconImg.rows; ++i)
	{
		for (int j = 0; j < reconImg.cols; ++j)
		{
			if (reconImg.at<float>(i, j) < 0)
			{
				reconImg.at<float>(i, j) = 0;
			}
		}
	}
	cv::Mat nonFiltered = reconImg;
	cv::Mat Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	double reconImgpixelValue; 
	codedreconImg = reconImg;

	mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));


	nonFiltered = comptonImg;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 2);
	Filtered.convertTo(Filtered, CV_32S);
	comptonImg = Filtered;
	
	mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	
	//normalize 해서 픽셀곱해주는 부분 추가 필요
	Mat mCodedImagenorm;
	Mat mComptonImagenorm;
	cv::normalize(mCodedImage, mCodedImagenorm, 0, 255, cv::NORM_MINMAX);
	cv::normalize(mComptonImage, mComptonImagenorm, 0, 255, cv::NORM_MINMAX);
	mHybridImage = mCodedImagenorm.mul(mComptonImagenorm);
	
	nonFiltered = mHybridImage;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	mHybridImage = Filtered;

	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	//cv::minMaxLoc(mHybridImage, &minVal, &maxVal, &minLoc, &maxLoc);		
	///spdlog::info("RadImage: Hybrid Max: " + std::to_string(maxVal));
	//mDetectorTransformation = dataVector[0].DetectorTransformation;
	//mListedListModeData = lmData;

}

HUREL::Compton::RadiationImage::RadiationImage(std::vector<sInteractionData> &dataVector, double s2M, double resImprov, double hFov, double wFov)
{
	double m2D = 0.073;
	//정지된 상태에서 영상화 가능
	if (dataVector.size() == 0)
	{
		return;
	}
	
	double m = 1 + m2D / s2M;
	double reconPlaneWidth = s2M / m2D * Det_W;
	double dproj = Det_W / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	int pixelCount = static_cast<int>(round(dproj * resImprov));
	int pixelCountcoded = static_cast<int>(round(dproj * 1));
	double det_w_div2 = -Det_W / 2;
	double pixelSize = Det_W / pixelCountcoded;
	
	double preCalc1 = reconPlaneWidth / pixelCount;	//영상화 영역 한 픽셀 길이
	double preCalc2 = reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;	//영상화 영역의 반 픽셀 길이 - 중심을 (0,0)으로 만들기 위해

	double imagePlaneZ = s2M + M2D;


	int pixelCountcompton = pixelCount;
	Mat comptonImg(pixelCountcompton, pixelCountcompton, CV_32S, Scalar(1));
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
				double imagePlaneZ = s2M + M2D;

				for (int i = 0; i < pixelCountcompton; ++i)
				{
					double imagePlaneX = preCalc1 * i + preCalc2;
					for (int j = 0; j < pixelCountcompton; ++j)
					{
						double imagePlaneY = preCalc1 * j + preCalc2;
						Eigen::Vector3d imgPoint;
						imgPoint[0] = imagePlaneX ;
						imgPoint[1] = imagePlaneY ;
						imgPoint[2] = imagePlaneZ ;
						comptonImgPtr[pixelCountcompton * (pixelCountcompton - j - 1) + pixelCountcompton - i - 1] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
					}
				}
			}
		}
		
		interationDataSum += data.RelativeInteractionPoint;
	}

	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(Mpix, Mpix), 0, 0, INTER_NEAREST);			
	cv::rotate(scaleG, scaleG, cv::ROTATE_90_COUNTERCLOCKWISE);			

	Mat responseImg;
	cv::eigen2cv(interationDataSum, responseImg);
	cv::resize(responseImg, responseImg, Size(pixelCountcoded, pixelCountcoded), 0, 0, INTER_LINEAR);
	
	Mat reconImg; Mat codedreconImg;
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG, cv::Point(-1,-1), 0.0, cv::BORDER_CONSTANT);
	//cv::flip(reconImg, reconImg, 1);			

	reconImg.convertTo(reconImg, CV_32F);
	if (resImprov > 1.0)
		cv::resize(reconImg, reconImg, Size(pixelCount, pixelCount), 0, 0, INTER_NEAREST);	//compton size로 변경 : 하이브리드 영상용

	double maxValue;

	double fovHeight = 2 * tan((hFov / 2) * M_PI / 180.0) * (s2M + m2D);
	double fovWidth = 2 * tan((wFov / 2) * M_PI / 180.0) * (s2M + m2D);

	//height correction
	//constexpr double heightDiff = 0.035;
	constexpr double heightDiff = abs(T265_TO_LAHGI_OFFSET_Y);
	constexpr double widthDiff = abs(T265_TO_LAHGI_OFFSET_X);;

	double heightPixelSize = reconPlaneWidth / pixelCount;
	int offSetPixelCount = heightDiff / heightPixelSize;
	int offSetWidthPixelCount = widthDiff / heightPixelSize;

	int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
	int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);

	int minHeightPixleCount; 
	int maxHeightPixleCount; 
	int minWidthPixleCount;
	int maxWidthPixleCount;

	if (T265_TO_LAHGI_OFFSET_Y < 0)
	{
		minHeightPixleCount = (pixelCount - heightPixelCount) / 2 - offSetPixelCount;
		maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 - offSetPixelCount;
	}
	else
	{
		minHeightPixleCount = (pixelCount - heightPixelCount) / 2 + offSetPixelCount;
		maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 + offSetPixelCount;
	}
	
	if (T265_TO_LAHGI_OFFSET_X < 0)
	{
		minWidthPixleCount = (pixelCount - widthPixelCount) / 2 - offSetWidthPixelCount;
		maxWidthPixleCount = (pixelCount + widthPixelCount) / 2 - offSetWidthPixelCount;
	}
	else
	{
		minWidthPixleCount = (pixelCount - widthPixelCount) / 2 + offSetWidthPixelCount;
		maxWidthPixleCount = (pixelCount + widthPixelCount) / 2 + offSetWidthPixelCount;
	}

	
	if (minHeightPixleCount < 0)
	{
		minHeightPixleCount = 0;
	}
	if (maxHeightPixleCount > pixelCount)
	{
		maxHeightPixleCount = pixelCount;
	}
	/*
	if (widthPixelCount > pixelCount)
	{
		widthPixelCount = pixelCount;
	}
	*/

	if (minWidthPixleCount < 0)
	{
		minWidthPixleCount = 0;
	}
	if (maxWidthPixleCount > pixelCount)
	{
		maxWidthPixleCount = pixelCount;
	}

	for (int i = 0; i < reconImg.rows; ++i)
	{
		for (int j = 0; j < reconImg.cols; ++j)
		{
			if (reconImg.at<float>(i, j) < 0)
			{
				reconImg.at<float>(i, j) = 0;
			}
		}
	}
	cv::Mat nonFiltered = reconImg;
	cv::Mat Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	double reconImgpixelValue; 
	codedreconImg = reconImg;

	//mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range(minWidthPixleCount, maxWidthPixleCount));

	nonFiltered = comptonImg;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 2);
	Filtered.convertTo(Filtered, CV_32S);
	comptonImg = Filtered;
	//mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range(minWidthPixleCount, maxWidthPixleCount));
	
	//normalize 해서 픽셀곱해주는 부분 추가 필요
	Mat mCodedImagenorm;
	Mat mComptonImagenorm;
	cv::normalize(mCodedImage, mCodedImagenorm, 0, 255, cv::NORM_MINMAX);
	cv::normalize(mComptonImage, mComptonImagenorm, 0, 255, cv::NORM_MINMAX);
	mHybridImage = mCodedImagenorm.mul(mComptonImagenorm);
	
	nonFiltered = mHybridImage;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	mHybridImage = Filtered;

	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	//cv::minMaxLoc(mHybridImage, &minVal, &maxVal, &minLoc, &maxLoc);		
	///spdlog::info("RadImage: Hybrid Max: " + std::to_string(maxVal));
	//mDetectorTransformation = dataVector[0].DetectorTransformation;
	//mListedListModeData = lmData;

}

HUREL::Compton::RadiationImage::RadiationImage(std::vector<sInteractionData> &dataVector, double s2M, double resImprov, open3d::geometry::PointCloud& reconPointCloud, Eigen::Matrix4d transMatrix, double* outmaxValue, double* outmaxLocx, double* outmaxLocy, double* outmaxLocz)
{
	//이동하면서 pcl에 영상화, 콤프턴 완료
	if (dataVector.size() == 0)
	{
		return;
	}
	open3d::geometry::PointCloud recontransPC;
	recontransPC = HUREL::Compton::RtabmapSlamControl::instance().RTPointCloudTransposed(reconPointCloud, transMatrix);
	
	HUREL::Compton::ReconPointCloud reconPCtrans = HUREL::Compton::ReconPointCloud(recontransPC);
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(reconPointCloud);

	open3d::geometry::PointCloud recontransPCFOVlim;
	double m = 1 + M2D / s2M;
	double imagePlaneZ = s2M + M2D;
	double dproj = Det_W/ (Mask_W / Mpix * m); 
	int pixelCount = static_cast<int>(round(dproj * resImprov));
	int pixelCountcoded = static_cast<int>(round(dproj * 1));
	int pixelLength = static_cast<int>(round(dproj * 1)); // pixel length of detector
	
	Mat comptonImg(480, 848, CV_32S, Scalar(1)); //848*480
	int32_t* comptonImgPtr = static_cast<int32_t*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;

	Eigen::MatrixXd interationDataSum = Eigen::MatrixXd::Zero(INTERACTION_GRID_SIZE, INTERACTION_GRID_SIZE); //response image should be rebuilt
	//const int32_t i32_min = 1u << 31;
	int32_t maxVal = 0;
	Eigen::Vector3d maxValLoc;
	maxValLoc[0] = 0;
	maxValLoc[1] = 0;
	maxValLoc[2] = 0;
	for (int index = 0; index < dataVector.size(); ++index)
	{
		sInteractionData &data = dataVector.at(index);
		interationDataSum += data.RelativeInteractionPoint;

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

				int pointCount = 0;
				Eigen::Vector3d imgPoint;
				
				for (int l = 0; l < 480; ++l)
				{
					for (int j = 0; j < 848; ++j)
					{
						if(pointCount >= reconPCtrans.points_.size())
						{
							std::cerr << "Pont count is over the point cloud size" << std::endl;
						}

						imgPoint[0] = reconPCtrans.points_[pointCount].x();
						imgPoint[1] = reconPCtrans.points_[pointCount].y();
						imgPoint[2] = reconPCtrans.points_[pointCount].z();

						//comptonImgPtr[480 * (848 - j - 1) + 480 - l - 1] += ReconPointCloud::SimpleComptonBackprojectionTransformed(lm, imgPoint, 1);
						comptonImg.at<int32_t>(l, j)+= ReconPointCloud::SimpleComptonBackprojectionTransformed(lm, imgPoint, 1);
						if (maxVal < comptonImg.at<int32_t>(l, j))
						{
							maxVal = comptonImg.at<int32_t>(l, j);
							maxValLoc = imgPoint;
						}
						++pointCount;
					}
				}
			}
		}
	}
	cv::Mat nonFiltered = comptonImg;
	cv::Mat Filtered;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 3);
	Filtered.convertTo(Filtered, CV_32S);
	mComptonImage = Filtered;
	
	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(Mpix * 1, Mpix * 1), 0, 0, INTER_NEAREST);
	
	Mat responseImg;
	cv::eigen2cv(interationDataSum, responseImg);
	cv::resize(responseImg, responseImg, Size(pixelCountcoded, pixelCountcoded), 0, 0, INTER_LINEAR);

	Mat drawImg;
	//convert type to CV_32S
	bool isDraw = false;
	if (isDraw)
	{
		responseImg.convertTo(drawImg, CV_32S);
		ShowCV_32SAsJet(drawImg, 1000);
	}	
	
	Mat reconImg;
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG);
	cv::rotate(reconImg, reconImg, ROTATE_90_COUNTERCLOCKWISE);
	reconImg.convertTo(reconImg, CV_32F);
	cv::resize(reconImg, reconImg, Size(pixelCountcoded*resImprov, pixelCountcoded*resImprov), 0, 0, INTER_LINEAR); //직교 좌표계에 그린 부호화구경 영상

	nonFiltered = reconImg;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered; //직교 좌표계에 그린 뒤 필터링한 부호화구경 영상


	mCodedImage = Filtered;
	mHybridImage = Filtered;
// ------Compton imaging done---------------

	mDetectorTransformation = dataVector[0].DetectorTransformation;

	maxVal = *outmaxValue;
	maxValLoc[0] = *outmaxLocx;
	maxValLoc[1] = *outmaxLocy;
	maxValLoc[2] = *outmaxLocz; 
	//std::cout << "maxValue is: " << maxVal << std::endl;
    //std::cout << "maxLocx is: " << maxValLoc[0]  << " " << maxValLoc[1]  << " " << maxValLoc[2]  << std::endl;
}

HUREL::Compton::RadiationImage::RadiationImage(std::vector<sInteractionData> &dataVector, double s2M, double resImprov, open3d::geometry::PointCloud& reconPointCloud, double* outmaxValue, double* outmaxLocx, double* outmaxLocy, double* outmaxLocz)
{
	//정지상태에서 compton 영상화 가능 완료
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(reconPointCloud);
	double m = 1 + M2D / s2M;
	double imagePlaneZ = s2M + M2D;
	double dproj = Det_W/ (Mask_W / Mpix * m); 
	int pixelCount = static_cast<int>(round(dproj * resImprov));
	int pixelCountcoded = static_cast<int>(round(dproj * 1));
	int pixelLength = static_cast<int>(round(dproj * 1)); // pixel length of detector

    // should be fixed
	/*
	open3d::geometry::PointCloud recontransPC;
	open3d::geometry::PointCloud recontransPCFOVlim;
	Eigen::MatrixXd fovchk(1, reconPC.points_.size());
	reconPC.imspaceLim(reconPointCloud, wFov, hFov, &reconPCFOVlim, &fovchk);
	reconPC.imspaceLim(reconPointCloud, 50, 50, transMatrix, &recontransPCFOVlim, &recontransPC, &fovchk);
	*/	
	if (dataVector.size() == 0)
	{
		return;
	}
	Mat comptonImg(480, 848, CV_32S, Scalar(1)); //848*480
	int32_t* comptonImgPtr = static_cast<int32_t*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;

	Eigen::MatrixXd interationDataSum = Eigen::MatrixXd::Zero(INTERACTION_GRID_SIZE, INTERACTION_GRID_SIZE); //response image should be rebuilt
	double maxVal = 0;
	Eigen::Vector3d maxValLoc;
	for (int index = 0; index < dataVector.size(); ++index)
	{
		sInteractionData &data = dataVector.at(index);
		interationDataSum += data.RelativeInteractionPoint;

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

				int pointCount = 0;
				Eigen::Vector3d imgPoint;

				for (int l = 0; l < 480; ++l)
				{
					for (int j = 0; j < 848; ++j)
					{
						if(pointCount >= reconPC.points_.size())
						{
							std::cerr << "Pont count is over the point cloud size" << std::endl;
						}
						imgPoint[0] = reconPC.points_[pointCount].x();
						imgPoint[1] = reconPC.points_[pointCount].y()+0.35;
						imgPoint[2] = reconPC.points_[pointCount].z();
						//comptonImgPtr[480 * (848 - j - 1) + 480 - l - 1] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
						comptonImg.at<int32_t>(l, j)+= ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
						if (maxVal < comptonImg.at<int32_t>(l, j))
						{
							maxVal = comptonImg.at<int32_t>(l, j);
							maxValLoc = imgPoint;
						}
						++pointCount;
					}
				}
			}
		}
	}
	cv::Mat nonFiltered = comptonImg;
	cv::Mat Filtered;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 3);
	Filtered.convertTo(Filtered, CV_32S);
	mComptonImage = Filtered;

	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(Mpix * 1, Mpix * 1), 0, 0, INTER_NEAREST);
	
	Mat responseImg;
	cv::eigen2cv(interationDataSum, responseImg);
	cv::resize(responseImg, responseImg, Size(pixelCountcoded, pixelCountcoded), 0, 0, INTER_LINEAR);

	Mat drawImg;
	//convert type to CV_32S
	bool isDraw = false;
	if (isDraw)
	{
		responseImg.convertTo(drawImg, CV_32S);
		ShowCV_32SAsJet(drawImg, 1000);
	}	
	
	Mat reconImg;
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG);
	cv::rotate(reconImg, reconImg, ROTATE_90_COUNTERCLOCKWISE);
	reconImg.convertTo(reconImg, CV_32F);
	cv::resize(reconImg, reconImg, Size(pixelCountcoded*resImprov, pixelCountcoded*resImprov), 0, 0, INTER_LINEAR); //직교 좌표계에 그린 부호화구경 영상

	nonFiltered = reconImg;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered; //직교 좌표계에 그린 뒤 필터링한 부호화구경 영상


	mCodedImage = Filtered;
	mHybridImage = Filtered;
// ------Compton imaging done---------------

	maxVal = *outmaxValue;
	maxValLoc[0] = *outmaxLocx;
	maxValLoc[1] = *outmaxLocy;
	maxValLoc[2] = *outmaxLocz; 

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
