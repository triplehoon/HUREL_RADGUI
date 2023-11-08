#include "ListModeData.h"

using namespace HUREL::Compton;
using namespace std;

string PrintEigenInOneLine(Eigen::Matrix4d m);

string ListModeData::WriteListModeData()
{
	string data = "";
	data += std::to_string(this->InteractionTimeInMili.count());	data += ",";
	data += std::to_string(this->Scatter.RelativeInteractionPoint[0]);	data += ",";
	data += std::to_string(this->Scatter.RelativeInteractionPoint[1]);	data += ",";
	data += std::to_string(this->Scatter.RelativeInteractionPoint[2]);	data += ",";
	data += std::to_string(this->Scatter.InteractionEnergy);	data += ",";
	data += std::to_string(this->Absorber.RelativeInteractionPoint[0]);	data += ",";
	data += std::to_string(this->Absorber.RelativeInteractionPoint[1]); data += ",";
	data += std::to_string(this->Absorber.RelativeInteractionPoint[2]); data += ",";
	data += std::to_string(this->Absorber.InteractionEnergy); data += ",";
    data += PrintEigenInOneLine(this->DetectorTransformation); data += ",";
    data += this->EnergyCheck.sourceName;

    return data;
}

string ListModeData::WriteListModeTransData()
{
	string data = "";
	data += std::to_string(this->InteractionTimeInMili.count());	data += ",";
	data += std::to_string(this->Scatter.TransformedInteractionPoint[0]);	data += ",";
	data += std::to_string(this->Scatter.TransformedInteractionPoint[1]);	data += ",";
	data += std::to_string(this->Scatter.TransformedInteractionPoint[2]);	data += ",";
	data += std::to_string(this->Scatter.InteractionEnergy);	data += ",";
	data += std::to_string(this->Absorber.TransformedInteractionPoint[0]);	data += ",";
	data += std::to_string(this->Absorber.TransformedInteractionPoint[1]); data += ",";
	data += std::to_string(this->Absorber.TransformedInteractionPoint[2]); data += ",";
	data += std::to_string(this->Absorber.InteractionEnergy); data += ",";
    data += PrintEigenInOneLine(this->DetectorTransformation); data += ",";
    data += this->EnergyCheck.sourceName;

    return data;
}
bool ListModeData::ReadListModeData(string data)
{
    std::vector<string> words;
    stringstream sstream(data);
    string word;
    while (getline(sstream, word, ','))
    {
        words.push_back(word);
    }
    if (words.size() <= 24 || words.size() > 26)
    {
        return false;
    }
    this->InteractionTimeInMili = chrono::milliseconds(stoll(words[0]));
    this->Scatter.RelativeInteractionPoint[0] = stod(words[1]);
    this->Scatter.RelativeInteractionPoint[1] = stod(words[2]);
    this->Scatter.RelativeInteractionPoint[2] = stod(words[3]);
    this->Scatter.InteractionEnergy = stod(words[4]);

    this->Absorber.RelativeInteractionPoint[0] = stod(words[5]);
    if (!isnan(this->Absorber.RelativeInteractionPoint[0]) &&  !(isnan(this->Scatter.RelativeInteractionPoint[0])))
    {
        this->Type = eInterationType::COMPTON;        
    }
    else if (!isnan(this->Scatter.RelativeInteractionPoint[0]))
    {
        this->Type = eInterationType::CODED;
    }
    else
    {
        this->Type = eInterationType::NONE;
    }
    this->Absorber.RelativeInteractionPoint[1] = stod(words[6]);
    this->Absorber.RelativeInteractionPoint[2] = stod(words[7]);
    this->Absorber.InteractionEnergy = stod(words[8]);
    Eigen::Matrix4d& m = this->DetectorTransformation;
    int k = 9;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            m(i, j) = stod(words[k++]);
        }
    }
    if (words.size() == 26)
    {
        switch (this->Type)
        {
        case eInterationType::COMPTON:
            this->EnergyCheck = IsotopeAnalysis::GetEnergyCheck(words[25], this->Scatter.InteractionEnergy + this->Absorber.InteractionEnergy);
            break;   
        case eInterationType::CODED:
            this->EnergyCheck = IsotopeAnalysis::GetEnergyCheck(words[25], this->Scatter.InteractionEnergy);
            break;     
        default:
            break;
        }            
    }        
    
    this->Scatter.TransformedInteractionPoint = m * this->Scatter.RelativeInteractionPoint;
    this->Absorber.TransformedInteractionPoint = m * this->Absorber.RelativeInteractionPoint;
    return true;
}
string PrintEigenInOneLine(Eigen::Matrix4d m)
{
    string data;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            data += std::to_string(m(i, j));
            if (i == 3 && j == 3)
            {
                break;
            }
            data += ",";
        }
    }
    return data;
}