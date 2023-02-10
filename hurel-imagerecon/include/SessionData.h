

#include "EnergySpectrum.h"
#include "ListModeData.h"


namespace HUREL
{
    namespace Compton
    {
        class SessionData
        {
        private:
            EnergySpectrum mSumEnergySpectrum;

            std::vector<ListModeData> mListModeData;

        public:
            SessionData();
            ~SessionData();
        };        
    } // namespace Compton
    

    
} // namespace HUREL

