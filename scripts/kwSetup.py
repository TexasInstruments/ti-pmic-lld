import os
import subprocess

if __name__ == "__main__":
    kwlpFile = ".kwlp"
    kwpsFile = ".kwps"
    pconfFile = "analysis_profile_SA_plus_MISRAC_2012_HIS_modified.pconf"
    mconfFile = "his_metrics_community_old.mconf"
    sconfFile = "kw_filter.sconf"
    kwReport = "kw_report.xml"
    kwOutput = "kwinject.out"

    if ((not os.path.exists(kwlpFile)) and (not os.path.exists(kwpsFile))):
        subprocess.run(["kwcheck", "create"])
        subprocess.run(["kwcheck", "set", "license.host=kw-lic.ent.ti.com", "license.port=27005"])
        if (os.path.exists(pconfFile)):
            subprocess.run(["kwcheck", "import", pconfFile])
        else:
            print(pconfFile + " not found.")
            exit(-1)
        if (os.path.exists(mconfFile)):
            subprocess.run(["kwcheck", "import", mconfFile])
        else:
            print(mconfFile + " not found.")
            exit(-1)
        if (os.path.exists(sconfFile)):
            subprocess.run(["kwcheck", "import", sconfFile])
        else:
            print(sconfFile + " not found.")
            exit(-1)
    
    if (os.path.exists(kwReport)):
        os.remove(kwReport)
    else:
        print("Existing " + kwReport + " file not found")
    if (os.path.exists(kwOutput)):
        os.remove(kwOutput)
    else:
        print("Existing " + kwOutput + " file not found")
        