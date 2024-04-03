"""Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the
distribution.

Neither the name of Texas Instruments Incorporated nor the names of
its contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."""
import os
import sys
import subprocess
import argparse

if __name__ == "__main__":
    success = 0
    kwlpFile = ".kwlp"
    kwpsFile = ".kwps"
    kwReport = "kw_report.xml"
    kwOutput = "kwinject.out"

    parser = argparse.ArgumentParser("kwsetup")
    parser.add_argument('kwCfgPath', help='Path to Klockwork configuration file directory.')
    args = parser.parse_args()

    # Get all KW file paths relative to where the script is run 
    kwCfgPath = args.kwCfgPath
    pconfFile = os.path.join(kwCfgPath, "analysis_profile_SA_MISRA_HIS_2022.pconf")
    mconfFile = os.path.join(kwCfgPath, "his_metrics_community.mconf")
    sconfFile = os.path.join(kwCfgPath, "kw_filter.sconf")
    hisMetricsTConfFile =  os.path.join(kwCfgPath, "TI_HIS_Metrics_2022.4.tconf")
    misracTConfFile = os.path.join(kwCfgPath, "TI_MISRAC_2012_2022.4.tconf")
    saTConfFile = os.path.join(kwCfgPath, "TI_StaticAnalysis_2022.4.tconf")

    # If Klocwork local project does not exist...
    if ((not os.path.exists(kwlpFile)) and (not os.path.exists(kwpsFile))):
        # Create local project
        subprocess.run(["kwcheck", "create"], check = True)
        
        # Set license host and port
        subprocess.run(["kwcheck", "set", "license.host=kw-lic.ent.ti.com", "license.port=27005"], check = True)

        # Import .pconf file
        if (os.path.exists(pconfFile)):
            subprocess.run(["kwcheck", "import", pconfFile], check = True)
        else:
            raise FileNotFoundError(pconfFile)

        # Import .mconf file
        if (os.path.exists(mconfFile)):
            subprocess.run(["kwcheck", "import", mconfFile], check = True)
        else:
            raise FileNotFoundError(mconfFile)
            
        # Import .sconf file
        if (os.path.exists(sconfFile)):
            subprocess.run(["kwcheck", "import", sconfFile], check = True)
        else:
            raise FileNotFoundError(sconfFile)
        
        # Import HIS Metrics .tconf file
        if (os.path.exists(hisMetricsTConfFile)):
            subprocess.run(["kwcheck", "import", hisMetricsTConfFile], check = True)
        else:
            raise FileNotFoundError(hisMetricsTConfFile)

        # Import MISRA-C .tconf file
        if (os.path.exists(misracTConfFile)):
            subprocess.run(["kwcheck", "import", misracTConfFile], check = True)
        else:
            raise FileNotFoundError(misracTConfFile)
            
        # Import static analysis .tconf file
        if (os.path.exists(saTConfFile)):
            subprocess.run(["kwcheck", "import", saTConfFile], check = True)
        else:
            raise FileNotFoundError(saTConfFile)
    
    # Delete existing KW report and KW build specification (i.e., KW output)
    try: os.remove(kwReport)
    except: print("Could not find an existing " + kwReport + " to delete")
    try: os.remove(kwOutput)
    except: print("Could not find an existing " + kwOutput + " to delete")
        
