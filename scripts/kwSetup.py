import os
import subprocess

if __name__ == "__main__":
    kwlpFile = ".kwlp"
    kwpsFile = ".kwps"
    pconfFile = "..\\pmic_drv\\kw_cfg\\analysis_profile_SA_MISRA_HIS_2022.pconf"
    mconfFile = "..\\pmic_drv\\kw_cfg\\his_metrics_community.mconf"
    sconfFile = "..\\pmic_drv\\kw_cfg\\kw_filter.sconf"
    kwReport = "kw_report.xml"
    kwOutput = "kwinject.out"

    # If Klocwork local project does not exist...
    if ((not os.path.exists(kwlpFile)) and (not os.path.exists(kwpsFile))):
        # Create local project, set license host and port
        subprocess.run(["kwcheck", "create"])
        subprocess.run(["kwcheck", "set", "license.host=kw-lic.ent.ti.com", "license.port=27005"])

        # Import .pconf file
        if (os.path.exists(pconfFile)):
            subprocess.run(["kwcheck", "import", pconfFile])
        else:
            print(pconfFile + " not found.")
            exit(-1)
        
        # Import .mconf file
        if (os.path.exists(mconfFile)):
            subprocess.run(["kwcheck", "import", mconfFile])
        else:
            print(mconfFile + " not found.")
            exit(-2)

        # Import .sconf file
        if (os.path.exists(sconfFile)):
            subprocess.run(["kwcheck", "import", sconfFile])
        else:
            print(sconfFile + " not found.")
            exit(-3)
    
    # Delete existing KW report
    if (os.path.exists(kwReport)):
        os.remove(kwReport)
    else:
        print("Existing " + kwReport + " file not found")
    
    # Delete existing KW build specification (i.e., KW output)
    if (os.path.exists(kwOutput)):
        os.remove(kwOutput)
    else:
        print("Existing " + kwOutput + " file not found")
        