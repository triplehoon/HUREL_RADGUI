#include "EnergySpectrum.h"
#include <cassert>
#include <mutex>

using namespace HUREL::Compton;

std::mutex resetMutex;

HUREL::Compton::EnergySpectrum::EnergySpectrum() : EnergySpectrum(ENERGY_SPECTRUM_BIN_SIZE, ENERGY_SPECTRUM_MAX)
{    
}

HUREL::Compton::EnergySpectrum::EnergySpectrum(unsigned int binSize, double maxEnergy)
{
    assert(binSize > 0);
    int binCount = static_cast<unsigned int>(maxEnergy / binSize);
    mBinSize = static_cast<double>(binSize);
    mMaxEnergy = maxEnergy;
    for (unsigned long long i = 0; i < binCount + 1; ++i)
    {
        double energy = static_cast<double>(i * binSize + binSize / 2.0);

        mEnergyBin.push_back(energy);
        mHistogramEnergy.push_back(BinningEnergy{energy, 0});
    }
    static std::mutex initMutex;

    if (!mIsPythonInitialized)
    {
        initMutex.lock();
        if (mIsPythonInitialized)
        {
            initMutex.unlock();
            return;
        }
        Py_Initialize();
        // set python print to stdout

        PyObject *sys = PyImport_ImportModule("sys");
        PyObject *stdout = PySys_GetObject("stdout");
        PySys_SetObject("stdout", stdout);
        Py_DECREF(sys);
        if (_import_array() < 0)
        {
            PyErr_Print();
            PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");
            initMutex.unlock();
            return;
        }

        // print python hello world
        PyObject *pPrint = PyImport_ImportModule("builtins");
        PyObject *pPrintFunc = PyObject_GetAttrString(pPrint, "print");
        PyObject *pArgs = PyTuple_New(1);
        PyTuple_SetItem(pArgs, 0, PyUnicode_FromString("Hello Python!!!"));
        PyObject_CallObject(pPrintFunc, pArgs);
        Py_DECREF(pPrint);
        Py_DECREF(pPrintFunc);
        Py_DECREF(pArgs);

        mIsPythonInitialized = true;
        initMutex.unlock();
         std::vector<double> peaks;
        EnergySpectrum testSpect;
        spdlog::info("Initiate nasagamma it took about 3 seconds");
        GetSpectrumPeaks(testSpect, &peaks);

        spdlog::info("Done initiate nasagamma.");

    }
}

std::vector<BinningEnergy> HUREL::Compton::EnergySpectrum::GetHistogramEnergy()
{
    return mHistogramEnergy;
}

void HUREL::Compton::EnergySpectrum::AddEnergy(double energy)
{
    if (energy >= mMaxEnergy || energy < 0)
    {
        return;
    }
    ++mHistogramEnergy[floor(energy / mBinSize)].Count;
}

void HUREL::Compton::EnergySpectrum::Reset()
{
    for (unsigned int i = 0; i < mHistogramEnergy.size(); ++i)
    {
        mHistogramEnergy[i].Count = 0;
    }
}

EnergySpectrum HUREL::Compton::EnergySpectrum::operator+(const EnergySpectrum &rhs) const
{
    if (this->mBinSize != rhs.mBinSize || this->mMaxEnergy != rhs.mMaxEnergy)
    {
        return EnergySpectrum();
    }
    EnergySpectrum value = EnergySpectrum(this->mBinSize, this->mMaxEnergy);
    for (unsigned int i = 0; i < value.mHistogramEnergy.size(); ++i)
    {
        value.mHistogramEnergy[i].Count = this->mHistogramEnergy[i].Count + rhs.mHistogramEnergy[i].Count;
    }

    return value;
}
static int gil_init = 0;

std::vector<double> HUREL::Compton::EnergySpectrum::GetSpectrumPeaks(EnergySpectrum &spectrum,
                                                                     std::vector<double> *outSnrVector,
                                                                     double ref_x,
                                                                     double ref_fwhm,
                                                                     double fwhm_at_0,
                                                                     double min_snr)
{

    if (!gil_init)
    {
        gil_init = 1;
        PyEval_InitThreads();
        PyEval_SaveThread();
    }

    PyGILState_STATE state = PyGILState_Ensure();
    // Call Python/C API functions...    
    
    //print thread id
    std::stringstream ss;
    ss << std::this_thread::get_id();
    //spdlog::info("Thread id: {}", ss.str());
    assert(outSnrVector != nullptr);

    // check speed of python
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    // from nasagamma import spectrum as sp
    //lockPython.lock();

    PyObject *pModule = PyImport_ImportModule("nasagamma");

    // get spectrum module from nasagamma
    PyObject *pSpectrum = PyObject_GetAttrString(pModule, "spectrum");
    PyObject *pSearch = PyObject_GetAttrString(pModule, "peaksearch");
    Py_DECREF(pModule);

    // get spectrum data as numpy
    PyObject *pEnergyBinNumpy = PyList_New(spectrum.GetHistogramEnergy().size());
    PyObject *pEnergyCountNumpy = PyList_New(spectrum.GetHistogramEnergy().size());

    for (unsigned int i = 0; i < spectrum.GetHistogramEnergy().size(); ++i)
    {
        PyList_SetItem(pEnergyBinNumpy, i, PyFloat_FromDouble(spectrum.GetHistogramEnergy()[i].Energy));
        PyList_SetItem(pEnergyCountNumpy, i, PyFloat_FromDouble(spectrum.GetHistogramEnergy()[i].Count));
    }

    // print pEnergyBinNumpy and pEnergyCountNumpy
    /*PyObject* pPrint = PyImport_ImportModule("builtins");
    PyObject* pPrintFunc = PyObject_GetAttrString(pPrint, "print");
    PyObject* pArgs = PyTuple_New(1);
    PyTuple_SetItem(pArgs, 0, pEnergyBinNumpy);
    PyObject_CallObject(pPrintFunc, pArgs);
    PyTuple_SetItem(pArgs, 0, pEnergyCountNumpy);
    PyObject_CallObject(pPrintFunc, pArgs);



    Py_DECREF(pPrint);
    Py_DECREF(pPrintFunc);
    Py_DECREF(pArgs);*/

    // create spectrum object argument count, null, ennergybin, "keV"
    PyObject *pArgs = PyTuple_New(4);
    PyTuple_SetItem(pArgs, 0, pEnergyCountNumpy);
    PyTuple_SetItem(pArgs, 1, Py_None);
    PyTuple_SetItem(pArgs, 2, pEnergyBinNumpy);
    PyTuple_SetItem(pArgs, 3, PyUnicode_FromString("keV"));
    PyObject *pSpectrumPython = PyObject_GetAttrString(pSpectrum, "Spectrum");
    PyObject *pSpectrumObject = PyObject_CallObject(pSpectrumPython, pArgs);
    Py_DECREF(pEnergyBinNumpy);
    Py_DECREF(pEnergyCountNumpy);
    Py_DECREF(pSpectrumPython);
    Py_DECREF(pArgs);

    PyObject *pPeackSearchFunc = PyObject_GetAttrString(pSearch, "PeakSearch");
    // create serchResult object argument (spect, ref_x, ref_fwhm, fwhm_at_0, min_snr);
    pArgs = PyTuple_New(5);
    PyTuple_SetItem(pArgs, 0, pSpectrumObject);
    PyTuple_SetItem(pArgs, 1, PyFloat_FromDouble(ref_x));
    PyTuple_SetItem(pArgs, 2, PyFloat_FromDouble(ref_fwhm));
    PyTuple_SetItem(pArgs, 3, PyFloat_FromDouble(fwhm_at_0));
    PyTuple_SetItem(pArgs, 4, PyFloat_FromDouble(min_snr));
    PyObject *pSearchResultObject = PyObject_CallObject(pPeackSearchFunc, pArgs);
    Py_DECREF(pArgs);


    if (pSearchResultObject == nullptr)
    {
        spdlog::error("pSearchResultObject is nullptr");
        Py_DECREF(pSpectrumObject);
        Py_DECREF(pSpectrum);
        Py_DECREF(pSearch);

        PyGILState_Release(state);
        return std::vector<double>();
    }

    Py_DECREF(pPeackSearchFunc);
    Py_DECREF(pSpectrumObject);
    // get snr and peaks_idx in pSearchResultObject
    PyObject *pSnr = PyObject_GetAttrString(pSearchResultObject, "snr");
    PyObject *pPeaksIdx = PyObject_GetAttrString(pSearchResultObject, "peaks_idx");
    Py_DECREF(pSearchResultObject);

    // pritn pSnrs and pPeaksIdx

    // set numpy arry snr to outSnrVector
    for (unsigned int i = 0; i < PyArray_Size(pSnr); ++i)
    {
        npy_intp index = i;
        double value = PyFloat_AsDouble(PyArray_GETITEM(pSnr, PyArray_GetPtr((PyArrayObject *)pSnr, &index)));
        outSnrVector->push_back(value);
    }

    // set peaks_idx to return vector
    std::vector<double> peaks;
    for (unsigned int i = 0; i < PyArray_Size(pPeaksIdx); ++i)
    {
        npy_intp index = i;
        double value = PyFloat_AsDouble(PyArray_GETITEM(pPeaksIdx, PyArray_GetPtr((PyArrayObject *)pPeaksIdx, &index)));
        if (value < spectrum.GetHistogramEnergy().size() && value >= 0)
        {
            int int_value = static_cast<int>(value);
            peaks.push_back(spectrum.GetHistogramEnergy()[value].Energy);
        }        
    }
    Py_DECREF(pSnr);
    Py_DECREF(pPeaksIdx);
    Py_DECREF(pSpectrum);
    Py_DECREF(pSearch);
    PyGILState_Release(state);

    end = std::chrono::system_clock::now();
    // elapsed time in miliseconds
    std::chrono::duration<double> elapsed_seconds = end - start;
    //spdlog::info("Python elapsed time: {} ms", elapsed_seconds.count() * 1000);
    //lockPython.unlock();
    return peaks;
}

#include "SessionData.h"

void HUREL::Compton::EnergySpectrum::EnergySpectrumTestCode()
{

    // Load session data
    SessionData sessionData = SessionData("../202211130509Cs137_Right_30degree");

    // check loaded data
    if (sessionData.GetListedListModeData().size() == 0)
    {
        spdlog::error("No data loaded");
        return;
    }

    EnergySpectrum spectrum = sessionData.GetEnergySpectrum(5);
    std::vector<double> snr;
    for (int i = 0; i < 100; ++i)
    {
        // auto peaks = GetSpectrumPeaks(spectrum, &snr);
    }
    // test on not main thread non void function
    double ref_x = 662,
           ref_fwhm = 50,
           fwhm_at_0 = 10,
           min_snr = 5;
    std::vector<std::thread> threads;
    std::vector<double> snrs[100];
    for (int i = 0; i < 100; ++i)
    {
        threads.push_back(std::thread(EnergySpectrum::GetSpectrumPeaks, std::ref(spectrum), &snrs[i], ref_x, ref_fwhm, fwhm_at_0, min_snr));
    }
    for (int i = 0; i < 100; ++i)
    {
        threads[i].join();
    }
    for (int i = 0; i < 100; ++i)
    {
        auto peaks = GetSpectrumPeaks(spectrum, &snr);
    }
}

double HUREL::Compton::EnergySpectrum::GetDoseRate(EnergySpectrum &spect, double timeInSecond)
{
    InitDoseRate();
    if (!mIsDoseRateInitComplete)
    {
         return 0.0 / 0.0; 
    }
    if (spect.mHistogramEnergy.size() != 301)
    {
        return 0.0;
    }
    double doseRate = 0.0;
    for (int i = 0; i < spect.GetHistogramEnergy().size(); ++i)
    {
        doseRate += spect.GetHistogramEnergy()[i].Count * H10Coeff[i];
    }
//    return doseRate * DOSE_RATE_COEFFICIENT / timeInSecond;
    return doseRate / timeInSecond;
}

void HUREL::Compton::EnergySpectrum::InitDoseRate()
{
    if (mIsDoseRateInitComplete)
    {
        return;
    }

    // load doseRate.csv
    std::ifstream doseRateFile("doseRate.csv");
    if (!doseRateFile.is_open())
    {
        spdlog::error("doseRate.csv not found");
        return;
    }
    std::string line;
    if (std::getline(doseRateFile, line))
    {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> record;
        while (std::getline(ss, item, ','))
        {
            record.push_back(item);
        }
        for (int i = 0; i < record.size(); ++i)
        {
            GCoeff.push_back(std::stod(record[i]));
        }
    }
    if (std::getline(doseRateFile, line))
    {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> record;
        while (std::getline(ss, item, ','))
        {
            record.push_back(item);
        }
        for (int i = 0; i < record.size(); ++i)
        {
            H10Coeff.push_back(std::stod(record[i]));
        }
    }
    if (std::getline(doseRateFile, line))
    {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> record;
        while (std::getline(ss, item, ','))
        {
            record.push_back(item);
        }
        for (int i = 0; i < record.size(); ++i)
        {
            KermaCoeff.push_back(std::stod(record[i]));
        }
    }
        mIsDoseRateInitComplete = true;

}
