#############################################################################
# Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
# Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# 
# Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the
# distribution.
# 
# Neither the name of Texas Instruments Incorporated nor the names of
# its contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#############################################################################
import os
import platform
import subprocess

if __name__ == "__main__":
    success = 0
    kwlpFile = ".kwlp"
    kwpsFile = ".kwps"
    kwReport = "kw_report.xml"
    kwOutput = "kwinject.out"

    if (platform.system() == "Windows"):
        pconfFile = "..\\pmic-lld\\docs\\kw_cfg\\analysis_profile_SA_MISRA_HIS_2022.pconf"
        mconfFile = "..\\pmic-lld\\docs\\kw_cfg\\his_metrics_community.mconf"
        sconfFile = "..\\pmic-lld\\docs\\kw_cfg\\kw_filter.sconf"
    elif (platform.system() == "Linux"):
        pconfFile = "../pmic-lld/docs/kw_cfg/analysis_profile_SA_MISRA_HIS_2022.pconf"
        mconfFile = "../pmic-lld/docs/kw_cfg/his_metrics_community.mconf"
        sconfFile = "../pmic-lld/docs/kw_cfg/kw_filter.sconf"
    else:
        print("Error: unsupported system")
        exit(-1)    

    # If Klocwork local project does not exist...
    if ((not os.path.exists(kwlpFile)) and (not os.path.exists(kwpsFile))):
        try:
            # Create local project
            status = subprocess.run(["kwcheck", "create"])
            status.check_returncode()
            
            # Set license host and port
            status = subprocess.run(["kwcheck", "set", "license.host=kw-lic.ent.ti.com", "license.port=27005"])
            status.check_returncode()

            # Import .pconf file
            if (os.path.exists(pconfFile)):
                status = subprocess.run(["kwcheck", "import", pconfFile])
                status.check_returncode()
            else:
                raise FileNotFoundError(pconfFile)

            # Import .mconf file
            if (os.path.exists(mconfFile)):
                status = subprocess.run(["kwcheck", "import", mconfFile])
                status.check_returncode()
            else:
                raise FileNotFoundError(mconfFile)
                
            # Import .sconf file
            if (os.path.exists(sconfFile)):
                status = subprocess.run(["kwcheck", "import", sconfFile])
                status.check_returncode()
            else:
                raise FileNotFoundError(sconfFile)
        except Exception as e:
            print("\"" + str(e) + "\" exception at line number " + str(e.__traceback__.tb_lineno))
            exit(-1)
    
    # Delete existing KW report and KW build specification (i.e., KW output)
    try: os.remove(kwReport)
    except: print("Could not find an existing " + kwReport + " to delete")
    try: os.remove(kwOutput)
    except: print("Could not find an existing " + kwOutput + " to delete")
        