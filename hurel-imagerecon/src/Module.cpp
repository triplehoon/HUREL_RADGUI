#include "Module.h"


using namespace std;
using namespace HUREL::Compton;
HUREL::Compton::Module::Module() :
    mModuleType(HUREL::Compton::eMouduleType::MONO),
    mModuleName(""),
    mModuleOffsetX(0),
    mModuleOffsetY(0),
    mModuleOffsetZ(0),
    mIsModuleSet(false),
    mEnergySpectrum(EnergySpectrum())
{
}

HUREL::Compton::Module::Module(eMouduleType moduleType,
    std::string configDir, std::string moduleName,
    double moduleOffsetX, double moduleOffsetY, double moduleOffsetZ,
    unsigned int binSize, double maxEnergy):
    mModuleType(moduleType),
    mModuleName(moduleName),
	mModuleOffsetX(moduleOffsetX),
	mModuleOffsetY(moduleOffsetY),
    mModuleOffsetZ(moduleOffsetZ),
	mIsModuleSet(false),
    mEnergySpectrum(EnergySpectrum())
{
    mLutFileName = configDir + "/LUT" + "/" + moduleName + ".csv";
    mGainFileName = configDir + "/Gain" + "/" + moduleName + ".csv";
    mEcalFileName = configDir + "/Ecal" + "/" + moduleName + ".csv";

    if (moduleType == eMouduleType::QUAD)
    {
        double gain[10];
        if (LoadGain(mGainFileName, eMouduleType::QUAD, gain))
        {
            
            for (int i = 0; i < 10; ++i)
            {
                mGain[i] = gain[i];
            }
            for (int i = 0; i < 9; ++i)
            {
           
                mGainEigen(i) = gain[i];
            }

            string msg = "Successfuly to load a gain file : " + moduleName;
           //spdlog::info("C++::HUREL::Compton::Module: {0}", msg);
        }        
        else
        {
            string msg = "FAIL to load a gain file: " + moduleName;
            
            spdlog::error("C++::HUREL::Compton::Module: {0}", msg);
            mIsModuleSet = false;
            return;
        }
    }        	
	if (LoadLUT(mLutFileName))
	{
		//cout << "Module.cpp: Successfuly to load a lut file: " << moduleName << endl;
        string msg = "Successfuly to load a lut file: " + moduleName;
        //spdlog::info("C++::HUREL::Compton::Module: {0}", msg);
		mIsModuleSet = true;
	}
	else
	{
		//cout << "Module.cpp: FAIL to load a lut file: " << moduleName << endl;
        string msg = "FAIL to load a lut file: " + moduleName;
        spdlog::error("C++::HUREL::Compton::Module: {0}", msg);
		assert(false);
	}	
    if (LoadEcal(mEcalFileName))
    {
        //cout << "Module.cpp: Successfuly to load a lut file: " << moduleName << endl;
        string msg = "Successfuly to load a ecal file: " + moduleName;
        //spdlog::info("C++::HUREL::Compton::Module: {0}", msg);

        mIsModuleSet = true;
    }
    else
    {
        //cout << "Module.cpp: FAIL to load a lut file: " << moduleName << endl;
        string msg = "FAIL to load a ecal file: " + moduleName;
        spdlog::error("C++::HUREL::Compton::Module: {0}", msg);
        assert(false);
    }
    assert(mModuleType != eMouduleType::MONO);
}

std::tuple<unsigned int, unsigned int> HUREL::Compton::Module::FastMLPosEstimationFindMaxIndex(const unsigned int gridSize, int minX, int maxX, int minY, int maxY, const double(&normalizePMTValue)[9]) const
{
    assert(minX < maxX);
    assert(minX < maxX);
    minX = minX + gridSize / 2;
    maxX = maxX - gridSize / 2;
    minY = minY + gridSize / 2;
    maxY = maxY - gridSize / 2;
    constexpr int activeIdex = 0;
    if (static_cast<int>(mLutSize) - 1 - activeIdex < maxX)
    {
        maxX = mLutSize - 1 - activeIdex;
    }
    if (static_cast<int>(mLutSize) - 1 - activeIdex < maxY)
    {
        maxY = mLutSize - 1 - activeIdex;
    }
    if (static_cast<int>(minX) < 0 + activeIdex)
    {
        minX = 0 + activeIdex;
    }
    if (static_cast<int>(minY) < 0 + activeIdex)
    {
        minY = 0 + activeIdex;
    }

    double valMaxChk = -numeric_limits<double>::max();
    std::tuple<unsigned int, unsigned int> maxPoint;
    for (unsigned int x = static_cast<unsigned int>(minX); x <= static_cast<unsigned int>(maxX); x += gridSize)
    {
        for (unsigned int y = static_cast<unsigned int>(minY); y <= static_cast<unsigned int>(maxY); y += gridSize)
        {
            double val = 0;
            if (isnan(mXYLogMue[x][y][0]))
            {
                continue;
            }
            for (int i = 0; i < 9; i++)
            {
               /* if (mXYLogMue[x][y] == NULL)
                {
                    continue;
                }*/
                val += mXYLogMue[x][y][i] * normalizePMTValue[i];

            }
            val -= mXYSumMu[x][y];

            if (val > valMaxChk)
            {
                valMaxChk = val;
                get<0>(maxPoint) = x;
                get<1>(maxPoint) = y;
            }
        }
    }
    
    return maxPoint;
}
std::tuple<unsigned int, unsigned int> HUREL::Compton::Module::FastMLPosEstimationFindMaxIndex(const unsigned int gridSize, int minX, int maxX, int minY, int maxY, const Eigen::Array<float, 1, 9>& pmtADCValue) const
{
    assert(minX < maxX);
    assert(minX < maxX);
    minX = minX + gridSize / 2;
    maxX = maxX - gridSize / 2;
    minY = minY + gridSize / 2;
    maxY = maxY - gridSize / 2;
    constexpr int activeIdex = 0;
    if (static_cast<int>(mLutSize) - 1 - activeIdex < maxX)
    {
        maxX = mLutSize - 1 - activeIdex;
    }
    if (static_cast<int>(mLutSize) - 1 - activeIdex < maxY)
    {
        maxY = mLutSize - 1 - activeIdex;
    }
    if (static_cast<int>(minX) < 0 + activeIdex)
    {
        minX = 0 + activeIdex;
    }
    if (static_cast<int>(minY) < 0 + activeIdex)
    {
        minY = 0 + activeIdex;
    }

    double valMaxChk = -numeric_limits<float>::max();
    std::tuple<unsigned int, unsigned int> maxPoint;
    for (unsigned int x = static_cast<unsigned int>(minX); x <= static_cast<unsigned int>(maxX); x += gridSize)
    {
        for (unsigned int y = static_cast<unsigned int>(minY); y <= static_cast<unsigned int>(maxY); y += gridSize)
        {
            double val = 0;
            if (isnan(mXYSumMu[x][y]))
            {
                continue;
            }
            Eigen::Array < float , 1, 9 > valArray = mXYLogMueEigen[x][y] * pmtADCValue;
            val = valArray.sum();
            val -= mXYSumMu[x][y];

            if (val > valMaxChk)
            {
                valMaxChk = val;
                get<0>(maxPoint) = x;
                get<1>(maxPoint) = y;
            }
        }
    }

    return maxPoint;
}

HUREL::Compton::Module::~Module()
{
    for (int i = 0; i < static_cast<int>(mLutSize); ++i)
    {
        for (int j = 0; j < static_cast<int>(mLutSize); ++j)
        {
            delete[] mXYLogMue[i][j];
            
        }
        delete[] mXYLogMue[i];
        delete[] mXYSumMu[i];         
        delete[] mXYLogMueEigen;
    }
}

bool HUREL::Compton::Module::LoadLUT(std::string fileName)
{
    std::ifstream io;
    io.open(fileName.c_str());
    vector<std::array<double, 80>> lutData;
    lutData.reserve(100000);


    if (!io.is_open())
    {
        //cerr << "Cannot open a file" << endl;

        io.close();
        return false;
    }
    unsigned int lutSize = 0;
    while (!io.eof())
    {   
        string line;

        std::getline(io, line);

        /* for each value */
        string val;        
        std::stringstream lineStream(line);

        std::array<double, 80> row;

        int i = 0;
        while (std::getline(lineStream, val, ','))
        {
            row[i] = stod(val);
            ++lutSize;
            ++i;
        }                        
        
        lutData.push_back(row);
    }
    lutData.shrink_to_fit();
    mLutSize = static_cast<unsigned int>(sqrt(lutData.size()));
    
    //Memory alloc
    mXYLogMue = new double**[mLutSize];
    mXYLogMueEigen = new Eigen::Array<float, 1, 9> * [mLutSize];
    mXYSumMu = new double*[mLutSize];
    for (int i = 0; i < static_cast<int>(mLutSize); ++i)
    {
        mXYLogMue[i] = new double*[mLutSize];
        mXYSumMu[i] = new double[mLutSize]; 
        mXYLogMueEigen[i] = new Eigen::Array<float, 1, 9>[mLutSize];
    }
 
    for (int i = 0; i < static_cast<int>(mLutSize* mLutSize); ++i)
    {
        int x = static_cast<int>(lutData[i][0]);
        int y = static_cast<int>(lutData[i][1]);
        int indexOffset = (mLutSize - 1) / 2;

        assert((x + indexOffset) >= 0);

        unsigned int indexX = x + indexOffset;
        unsigned int indexY = y + indexOffset;
        mXYLogMue[indexX][indexY] = new double[9] {lutData[i][3], lutData[i][4], lutData[i][5],
        lutData[i][6],lutData[i][7],lutData[i][8],lutData[i][9],lutData[i][10],lutData[i][11] };
        mXYSumMu[indexX][indexY] = lutData[i][12];
        for (int iInEigne = 0; iInEigne < 9; ++iInEigne)
        {
            mXYLogMueEigen[indexX][indexY](iInEigne) = static_cast<float>(lutData[i][iInEigne + 3]);
        }
    }

    //cout << "Done loading look up table" << endl;

    io.close();
    return true;
}

bool HUREL::Compton::Module::LoadEcal(std::string FileName)
{
    std::ifstream io;
    io.open(FileName);

    if (io.fail())
    {
        //cout << "Cannot open a file" << endl;
        string msg = "Cannot open a file LoadEcal";
        spdlog::error("C++::HUREL::Compton::Module: {0}", msg);
        io.close();
        return false;
    }


    while (!io.eof())
    {
        string line;

        std::getline(io, line);

        /* for each value */
        string val;
        vector<double> row;
        std::stringstream lineStream(line);
        unsigned int i = 0;
        std::getline(lineStream, val, ',');
        mEnergyCalibrationA = stod(val);
        std::getline(lineStream, val, ',');
        mEnergyCalibrationB = stod(val);
        std::getline(lineStream, val, ',');
        mEnergyCalibrationC = stod(val);
        break;
        
    }
    io.close();

    return true;
}

const bool HUREL::Compton::Module::IsModuleSet() const
{
    return mIsModuleSet;
}

bool HUREL::Compton::Module::LoadGain(std::string fileName, eMouduleType moduleType, double* outEGain)
{
    std::ifstream io;
    io.open(fileName);
    vector<vector<double>> lutData;

    assert(outEGain != NULL);
    if (io.fail())
    {
        //cout << "Cannot open a file" << endl;
        string msg = "Cannot open a file";
        spdlog::error("C++::HUREL::Compton::Module: {0}", msg);
        io.close();        
        return false;
    }

    unsigned int pmtCounts = 0;

    switch (moduleType)
    {
    case HUREL::Compton::eMouduleType::MONO:
        pmtCounts = 36;
        break;
    case HUREL::Compton::eMouduleType::QUAD:
        pmtCounts = 9;
        break;
    case HUREL::Compton::eMouduleType::QUAD_DUAL:
        pmtCounts = 9;
        break;
    default:
        pmtCounts = 0;
        return false;
        break;
    }

    while (!io.eof())
    {
        string line;

        std::getline(io, line);

        /* for each value */
        string val;
        vector<double> row;
        std::stringstream lineStream(line);
        unsigned int i = 0;
        while (std::getline(lineStream, val, ','))
        {       
            assert(i < pmtCounts + 1);
            outEGain[i] = stod(val);
            ++i;
        }      
    }
    io.close();

    return true;
}

const Eigen::Vector4d HUREL::Compton::Module::FastMLPosEstimation(const unsigned short pmtADCValue[]) const
{

    std::tuple<unsigned int, unsigned int> maxPoint;

    const double normalizedPMTValue[9] { 
    static_cast<double>(pmtADCValue[0]) * mGain[0],
    static_cast<double>(pmtADCValue[1]) * mGain[1],
    static_cast<double>(pmtADCValue[2]) * mGain[2],
    static_cast<double>(pmtADCValue[3]) * mGain[3],
    static_cast<double>(pmtADCValue[4]) * mGain[4],
    static_cast<double>(pmtADCValue[5]) * mGain[5],
    static_cast<double>(pmtADCValue[6]) * mGain[6],
    static_cast<double>(pmtADCValue[7]) * mGain[7],
    static_cast<double>(pmtADCValue[8]) * mGain[8]};
    constexpr double gridShrink = 1.1;
    int gridSize = 36;//45
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize, 0, mLutSize - 1, 0, mLutSize - 1, normalizedPMTValue);
    gridSize *= gridShrink;
    int gridSize2 = 18;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);
    
    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 9;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 5;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 3;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 2;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 1;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);


    Eigen::Vector4d point;

    point[0] = (static_cast<double>(get<0>(maxPoint)) - static_cast<double>(mLutSize - 1) / 2.0) / 1000 + mModuleOffsetX;
    point[1] = (static_cast<double>(get<1>(maxPoint)) - static_cast<double>(mLutSize - 1) / 2.0) / 1000 + mModuleOffsetY;
    point[2] = mModuleOffsetZ;
    point[3] = 1;
    return point;   
}

const Eigen::Vector4d HUREL::Compton::Module::FastMLPosEstimation(const Eigen::Array<float, 1, 9>& pmtADCValue) const
{

    std::tuple<unsigned int, unsigned int> maxPoint;

    Eigen::Array<float, 1, 9> normalizedPMTValue = (mGainEigen * pmtADCValue);
    normalizedPMTValue = normalizedPMTValue / normalizedPMTValue.sum();
    constexpr double gridShrink = 1.1;
    int gridSize = 36;//45
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize, 0, mLutSize - 1, 0, mLutSize - 1, normalizedPMTValue);
    gridSize *= gridShrink;
    int gridSize2 = 18;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 9;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 5;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 3;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 2;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);

    gridSize = gridSize2;
    gridSize *= gridShrink;
    gridSize2 = 1;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);


    Eigen::Vector4d point;

    point[0] = (static_cast<double>(get<0>(maxPoint)) - static_cast<double>(mLutSize - 1) / 2.0) / 1000 + mModuleOffsetX;
    point[1] = (static_cast<double>(get<1>(maxPoint)) - static_cast<double>(mLutSize - 1) / 2.0) / 1000 + mModuleOffsetY;
    point[2] = mModuleOffsetZ;
    point[3] = 1;
    return point;
}


const Eigen::Vector4d HUREL::Compton::Module::FastMLPosEstimationVerification(const unsigned short pmtADCValue[]) const
{

    std::tuple<unsigned int, unsigned int> maxPoint;

    const double normalizedPMTValue[9]{
    static_cast<double>(pmtADCValue[0]) * mGain[0],
    static_cast<double>(pmtADCValue[1]) * mGain[1],
    static_cast<double>(pmtADCValue[2]) * mGain[2],
    static_cast<double>(pmtADCValue[3]) * mGain[3],
    static_cast<double>(pmtADCValue[4]) * mGain[4],
    static_cast<double>(pmtADCValue[5]) * mGain[5],
    static_cast<double>(pmtADCValue[6]) * mGain[6],
    static_cast<double>(pmtADCValue[7]) * mGain[7],
    static_cast<double>(pmtADCValue[8]) * mGain[8] };

    int gridSize = mLutSize;//45
    maxPoint = FastMLPosEstimationFindMaxIndex(1, 0, mLutSize - 1, 0, mLutSize - 1, normalizedPMTValue);    
    Eigen::Vector4d point;
    point[0] = (static_cast<double>(get<0>(maxPoint)) - static_cast<double>(mLutSize - 1) / 2.0) / 1000 + mModuleOffsetX;
    point[1] = (static_cast<double>(get<1>(maxPoint)) - static_cast<double>(mLutSize - 1) / 2.0) / 1000 + mModuleOffsetY;
    point[2] = mModuleOffsetZ;
    point[3] = 1;
    return point;
}


const Eigen::Vector4d HUREL::Compton::Module::FastMLPosEstimation(unsigned short(&pmtADCValue)[36]) const
{
    assert(false && "FastMLPosEstimation NOT IMplented");
    return Eigen::Vector4d(0,0,0, 1);
}

const double HUREL::Compton::Module::GetEcal(const unsigned short pmtADCValue[]) const
{
    double sumEnergy = 0;
    unsigned short checkZero = 0;
    for (int i = 0; i < 9; ++i)
    {
        sumEnergy += static_cast<double>(pmtADCValue[i]) * mGain[i];
        checkZero += pmtADCValue[i];
    }
    if (checkZero == 0)
    {
        return static_cast<double>(NAN);
    }
    sumEnergy += mGain[9];
    return mEnergyCalibrationA * sumEnergy * sumEnergy + mEnergyCalibrationB * sumEnergy + mEnergyCalibrationC;
}
const double HUREL::Compton::Module::GetEcal(const Eigen::Array<float, 1, 9>& pmtADCValue) const
{
    double sumEnergy = (pmtADCValue * mGainEigen).sum();
    if (sumEnergy <= 0)
    {
        return static_cast<double>(NAN);
    }
    return mEnergyCalibrationA * sumEnergy * sumEnergy + mEnergyCalibrationB * sumEnergy + mEnergyCalibrationC;
}

void HUREL::Compton::Module::SetEnergyCalibration(double a, double b, double c)
{
    mEnergyCalibrationA = a;
    mEnergyCalibrationB = b;
    mEnergyCalibrationC = c;

    std::ofstream io;
    io.open(mEcalFileName);

    if (io.fail())
    {
        //cout << "Cannot open a file" << endl;
        string msg = "Cannot open a file SetEnergyCalibration";
        spdlog::error("C++::HUREL::Compton::Module: {0}", msg);
        io.close();
	}

	string line;
	io << mEnergyCalibrationA << "," << mEnergyCalibrationB << "," << mEnergyCalibrationC << ",";

    io.close();



}

std::tuple<double, double, double>  HUREL::Compton::Module::GetEnergyCalibration()
{
    return make_tuple(mEnergyCalibrationA,  mEnergyCalibrationB, mEnergyCalibrationC);
}

EnergySpectrum& HUREL::Compton::Module::GetEnergySpectrum()
{
    return mEnergySpectrum;
}

const std::string HUREL::Compton::Module::GetModuleName() const
{
    return mModuleName;
}

bool HUREL::Compton::Module::SetGain(eMouduleType type, std::vector<double> gain)
{
    if (type != eMouduleType::QUAD)
    {
        return false;
    }
    else
    {
        for (int i = 0; i < 10; ++i)
        {
            mGain[i] = gain[i];
        }
    }
    return true;
}