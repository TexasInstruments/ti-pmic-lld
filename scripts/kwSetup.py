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
            subprocess.run(["kwcheck", "create"])

            # Set license host and port
            subprocess.run(["kwcheck", "set", "license.host=kw-lic.ent.ti.com", "license.port=27005"])

            # Import .pconf file
            subprocess.run(["kwcheck", "import", pconfFile])
            
            # Import .mconf file
            subprocess.run(["kwcheck", "import", mconfFile])

            # Import .sconf file
            subprocess.run(["kwcheck", "import", sconfFile])
        except Exception as e:
            print("\"" + str(e) + "\" exception at line number " + str(e.__traceback__.tb_lineno))
            exit(-1)
    
    # Delete existing KW report and KW build specification (i.e., KW output)
    try: os.remove(kwReport)
    except: print("Could not find an existing " + kwReport + " to delete")
    try: os.remove(kwOutput)
    except: print("Could not find an existing " + kwOutput + " to delete")
        